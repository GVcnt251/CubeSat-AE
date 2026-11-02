"""Webcam fixa + observação óptica real + vista virtual, sem gravação/envio externo."""
import ctypes as C
import math
import os
import threading
import time
import numpy as np
from sil import lib,DetectorConfig,DetectorWorkspace,DetectionT
from gimbal import rotation


def opencv():
    try:
        import cv2
        return cv2
    except ImportError as exc:
        raise RuntimeError('Falta OpenCV. Executa: python -m pip install -r requirements-webcam.txt') from exc


class Capture:
    """O dispositivo pertence à thread; o simulador nunca bloqueia à espera de read()."""
    def __init__(self,index):
        self.cv=opencv();self.index=index;self.lock=threading.Lock()
        self.stop=threading.Event();self.latest=None;self.error='A abrir a webcam…';self.sequence=0
        self.thread=threading.Thread(target=self._read,daemon=True,name='CubeSat webcam')
        self.thread.start()

    def _read(self):
        capture=None
        try:
            backend=self.cv.CAP_DSHOW if os.name=='nt' else self.cv.CAP_ANY
            capture=self.cv.VideoCapture(self.index,backend)
            if not capture.isOpened():
                raise RuntimeError('Não foi possível abrir a webcam. Verifica o índice, as permissões e outras aplicações de vídeo.')
            capture.set(self.cv.CAP_PROP_FRAME_WIDTH,640)
            capture.set(self.cv.CAP_PROP_FRAME_HEIGHT,480)
            capture.set(self.cv.CAP_PROP_BUFFERSIZE,1)
            failures=0
            while not self.stop.is_set():
                ok,image=capture.read()
                if not ok:
                    failures+=1
                    self.error='A webcam deixou de enviar imagens.'
                    if failures>=10:break
                    self.stop.wait(.03);continue
                failures=0
                with self.lock:
                    self.sequence+=1
                    self.latest=(image,self.sequence,time.monotonic())
                    self.error=''
        except Exception as exc:
            self.error=str(exc)
        finally:
            if capture is not None:capture.release()

    def snapshot(self,now=None):
        with self.lock:
            if self.latest is None:return None
            frame,seq,stamp=self.latest
            return frame,seq,max(0,time.monotonic()-stamp)

    def close(self):
        self.stop.set();self.thread.join(timeout=1)


class WebcamView:
    def __init__(self,index=0,hfov=60,virtual_fov=40,capture=None):
        self.cv=opencv()
        self.cv.setNumThreads(1) # Frames pequenos: evitar concorrência excessiva com a física e a GUI.
        self.capture=capture if capture is not None else Capture(index)
        self.hfov,self.virtual_fov=hfov,virtual_fov
        self.f_source=160/math.tan(math.radians(hfov/2))
        self.f_virtual=160/math.tan(math.radians(virtual_fov/2))
        self.cfg=DetectorConfig();self.workspace=DetectorWorkspace()
        lib().cc_det_cfg_default(C.byref(self.cfg))
        self.last_sequence=None;self.last_age=-1.;self.last_status=1
        self.message='A abrir a webcam…';self.found=False;self.inside=False
        self.raw=None;self.virtual=None;self.source_xy=(math.nan,math.nan)
        self.target=(math.nan,math.nan);self.coverage=0.
        yy,xx=np.mgrid[0:240,0:320].astype(np.float32)
        self.rays=np.stack((np.ones_like(xx),(xx-159.5)/self.f_virtual,(yy-119.5)/self.f_virtual),axis=-1)

    def close(self):self.capture.close()

    def prepare(self,image):
        if image.ndim==2:image=self.cv.cvtColor(image,self.cv.COLOR_GRAY2BGR)
        elif image.shape[2]==4:image=self.cv.cvtColor(image,self.cv.COLOR_BGRA2BGR)
        h,w=image.shape[:2];scale=min(320/w,240/h)
        nw,nh=max(1,round(w*scale)),max(1,round(h*scale))
        small=self.cv.resize(image,(nw,nh),interpolation=self.cv.INTER_AREA)
        result=np.full((240,320,3),225,np.uint8)
        x,y=(320-nw)//2,(240-nh)//2
        result[y:y+nh,x:x+nw]=small
        # FoV é o campo horizontal do vídeo original. Preserve pixels quadrados no letterbox.
        focal=(w/2)/math.tan(math.radians(self.hfov/2))*scale
        return result,focal,(x,y,nw,nh)

    def detection_image(self):
        """Isola marcas pretas/neutras dentro de superfícies claras, preservando o vídeo a cores."""
        hsv=self.cv.cvtColor(self.raw,self.cv.COLOR_BGR2HSV)
        saturation,value=hsv[:,:,1],hsv[:,:,2]
        white=((value>=165)&(saturation<=100)).astype(np.uint8)*255
        x,y,w,h=self.active_rect
        valid=np.zeros_like(white);valid[y:y+h,x:x+w]=255
        white &= valid
        contours,_=self.cv.findContours(white,self.cv.RETR_EXTERNAL,self.cv.CHAIN_APPROX_SIMPLE)
        paper=np.zeros_like(white)
        for contour in contours:
            if self.cv.contourArea(contour)>=1200:
                self.cv.drawContours(paper,[contour],-1,255,self.cv.FILLED)
        gray=self.cv.cvtColor(self.raw,self.cv.COLOR_BGR2GRAY)
        neutral_black=(value<=110)&((saturation<=100)|(value<=40))
        image=np.full_like(gray,225)
        allowed=(paper>0)&(valid>0)
        image[allowed]=gray[allowed]
        image[~neutral_black]=np.maximum(image[~neutral_black],130)
        return image

    def acquire(self,now):
        sample=self.capture.snapshot(now)
        if sample is None or sample[2]>.35:
            self.last_age=-1 if sample is None else sample[2]
            self.found=False;self.inside=False
            self.message=self.capture.error or 'Sem imagem recente da webcam.'
            thread=getattr(self.capture,'thread',None)
            alive=thread is not None and thread.is_alive()
            self.last_status=1 if sample is None and alive else (4 if self.capture.error and not alive else 5)
            return 'missing',None
        image,sequence,age=sample
        self.last_age=age
        if sequence==self.last_sequence:return 'duplicate',None
        self.last_sequence=sequence
        self.raw,self.f_source,self.active_rect=self.prepare(image)
        d=DetectionT()
        begin=time.perf_counter()
        detector_image=self.detection_image()
        lib().cc_detect(C.byref(self.cfg),C.byref(self.workspace),detector_image.ctypes.data_as(C.POINTER(C.c_uint8)),C.byref(d))
        proc=(time.perf_counter()-begin)*1e6
        self.found=bool(d.found)
        if self.found:
            self.source_xy=(d.x,d.y)
            ray=np.array([1.,(d.x-159.5)/self.f_source,(d.y-119.5)/self.f_source])
            self.target=(math.atan2(ray[1],ray[0]),-math.atan2(ray[2],math.hypot(ray[0],ray[1])))
        else:
            self.source_xy=(math.nan,math.nan)
        self.last_status=2 if self.found else 3
        self.message='Ponto detetado na webcam.' if self.found else 'Mostra um círculo preto num papel branco, bem iluminado.'
        return 'fresh',(max(0,now-age),d,proc)

    def observe(self,angles,d):
        """Transforma a direção medida pela webcam fixa para o corpo virtual."""
        r=rotation(angles)
        world=self.rays @ r.T
        depth=world[:,:,0]
        safe=np.maximum(depth,1e-6)
        mx=(159.5+self.f_source*world[:,:,1]/safe).astype(np.float32)
        my=(119.5+self.f_source*world[:,:,2]/safe).astype(np.float32)
        x,y,w,h=self.active_rect
        mask=(depth>0)&(mx>=x)&(mx<x+w)&(my>=y)&(my<y+h)
        self.coverage=float(np.mean(mask))
        self.virtual=self.cv.remap(self.raw,mx,my,self.cv.INTER_LINEAR,borderMode=self.cv.BORDER_CONSTANT,borderValue=(225,225,225))
        self.virtual[~mask]=225
        yaw,pitch=DetectionT(),DetectionT()
        for out in (yaw,pitch):
            out.n_cand=d.n_cand;out.area=d.area;out.contrast=d.contrast
        self.inside=False
        if d.found:
            world_ray=np.array([1.,(d.x-159.5)/self.f_source,(d.y-119.5)/self.f_source])
            body=r.T @ world_ray
            if body[0]>.01:
                u=159.5+self.f_virtual*body[1]/body[0]
                v=119.5+self.f_virtual*body[2]/body[0]
                self.inside=0<=u<320 and 0<=v<240
                yaw.found=pitch.found=1
                yaw.x,yaw.y=u,v
                pitch.x=159.5-self.f_virtual*body[2]/math.hypot(body[0],body[1])
                pitch.y=119.5
        return yaw,pitch
