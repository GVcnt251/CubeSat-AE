#include "cube_core.h"
#include "cc_proto.h"
#include "moteus_proto.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static cc_t c;
static cc_meas_t m;
static cc_out_t o;
static uint64_t t;
static int fails;
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n",__LINE__,#x); fails++; } } while(0)
static void cmd(unsigned type,float a,unsigned id) { cc_cmd_t v={type,id,a,0}; cc_command(&c,&v); }
static void ready(void) {
    cc_init(&c,NULL); m=(cc_meas_t){0}; m.accel[2]=9.81f;
    for(t=0;t<=1600000;t+=5000) cc_step_with_imu(&c,t,&m,1,&o);
}
static void tick(int valid) { t+=5000; cc_step_with_imu(&c,t,&m,valid,&o); }
static void vision(void) {
    ready(); tick(1);
    cc_det_t d={0}; d.found=1; d.x=160;d.y=120;d.area=100;d.contrast=180;
    cc_vision_t v;
    CHECK(cc_on_detection(&c,t,t,&d,&v)==1);
    uint64_t capture=t;
    cmd(CC_CMD_MODE,CC_MODE_TRACK,0);tick(1);
    tick(0);
    CHECK(cc_on_detection(&c,t,t,&d,&v)==0);
    CHECK(v.found==1 && v.x==160 && v.t_capture_ms==t/1000 && !v.accepted);
    tick(1);
    CHECK(!c.target_valid);
    CHECK(cc_on_detection(&c,capture,t,&d,&v)==0);
    CHECK(!c.target_valid);
    CHECK(!(c.tlm.flags & CC_FLAG_TARGET_VALID) && c.tlm.trk==CC_TRK_NONE);
    tick(1);
    CHECK(cc_on_detection(&c,t,t,&d,&v)==1);
    CHECK(cc_on_detection(&c,t,t,&d,&v)==0); /* duplicate */
    uint64_t accepted=t;
    /* Model invalidation must not change cam_age to 'never'. */
    for(int i=0;i<5;i++) { tick(1); d.x=310; cc_on_detection(&c,t,t,&d,&v); }
    tick(1);
    CHECK(!c.target_valid && c.tlm.cam_age_s>0 && c.last_cam_us==accepted);
    cmd(CC_CMD_RECAL,0,0);tick(1);
    CHECK(c.tlm.cam_age_s>0 && c.last_cam_us==accepted);
}
static void recal_clock(void) {
    ready(); tick(0); uint64_t last=c.imu_last_valid_us;
    cmd(CC_CMD_RECAL,0,0);
    t=last+50001;
    CHECK(cc_step_with_imu(&c,t,&m,0,&o)==CC_IMU_RECAL_REQUIRED);
    CHECK(!o.enable && c.gcount==0);
}
static void motor_independence(void) {
    ready(); m.wheel_rps=NAN; m.gyro[2]=0.5f;
    float psi=c.x[0]; tick(1);
    CHECK(c.x[0]>psi && c.last_us==t);
    CHECK(isfinite(c.rate_I));
    m.wheel_rps=10; tick(1);
    cmd(CC_CMD_GAIN,FLT_MIN,CC_GAIN_RATE_KI);tick(1);
    CHECK(isfinite(c.rate_I));
    cmd(CC_CMD_MODE,CC_MODE_RATE,0);tick(1);
    CHECK(isfinite(o.wheel_cmd_rps));
}
static void held_tlm(void) {
    ready();cmd(CC_CMD_MODE,CC_MODE_WHEEL,0);cmd(CC_CMD_WHEEL,20,0);tick(1);
    m.wheel_rps=30;m.wheel_torque=0.1f;m.vbat_v=21;
    tick(0);
    CHECK(o.enable && c.tlm.wheel_rps==30 && c.tlm.vbat_v==21);
    CHECK(c.tlm.wheel_cmd_rps==o.wheel_cmd_rps);
    CHECK(c.tlm.rate_ref==0);
    ready();cmd(CC_CMD_MODE,CC_MODE_RATE,0);cmd(CC_CMD_RATE,0.7f,0);tick(1);
    CHECK(c.tlm.rate_ref==c.w_ref_prev && c.tlm.rate_ref<0.7f);
    c.trk=CC_TRK_LOCKED;
    cmd(CC_CMD_MODE,CC_MODE_HEADING,0);tick(1);
    CHECK(c.tlm.trk==CC_TRK_NONE);
}
static void detection_islands(void) {
    static uint8_t img[CC_IMG_N];static cc_det_ws_t ws;
    cc_det_cfg_t cfg;cc_det_cfg_default(&cfg);cc_det_t d;
    memset(img,220,sizeof(img));
    for(int y=20;y<220;y++)for(int x=20;x<300;x++)
        if(x<28||x>=292||y<28||y>=212)img[y*CC_IMG_W+x]=10;
    for(int y=110;y<=130;y++)for(int x=150;x<=170;x++)
        if((x-160)*(x-160)+(y-120)*(y-120)<=100)img[y*CC_IMG_W+x]=10;
    cc_detect(&cfg,&ws,img,&d);
    CHECK(d.found && fabsf(d.x-160)<0.1f && fabsf(d.y-120)<0.1f);
}
static void motor_command(void) {
    uint8_t b[64]; size_t n=moteus_pack_velocity(b,NAN,0.2f,1,1);
    CHECK(n>=3 && b[0]==1 && b[1]==0 && b[2]==0);
    n=moteus_pack_velocity(b,10,-1,1,0);
    CHECK(n>=3 && b[2]==0);
}
static void recal_camera_age(void) {
    ready();tick(1);
    CHECK(c.tlm.cam_age_s == -1.0f);
    cc_det_t d={0};cc_vision_t v;
    d.found=1;d.x=200;d.y=120;d.area=100;d.contrast=180;
    CHECK(cc_on_detection(&c,t,t,&d,&v)==1);
    uint64_t accepted=t;
    cmd(CC_CMD_RECAL,0,0);
    for(int i=0;i<330;i++) tick(1);
    CHECK(c.calibrated && c.mode==CC_MODE_IDLE);
    CHECK(c.last_cam_us==accepted);
    CHECK(fabsf(c.tlm.cam_age_s-(float)((t-accepted)*1e-6))<1e-5f);
    cmd(CC_CMD_MODE,CC_MODE_TRACK,0);
    for(int i=0;i<1020;i++) tick(1);
    CHECK(c.trk==CC_TRK_SEARCH);
    CHECK(c.search_span==c.cfg.search_span_unknown);
    CHECK(cc_on_detection(&c,t,t,&d,&v)==1);
    for(int i=0;i<1020;i++) tick(1);
    CHECK(c.trk==CC_TRK_SEARCH && c.search_span==c.cfg.search_span_known);
}
static void start_unloading(void) {
    ready();cc_set_gimbal_unload(&c,1);
    cmd(CC_CMD_MODE,CC_MODE_HEADING,0);
    m.wheel_rps=250;tick(1);tick(1);
    CHECK(o.enable && c.unloading && (c.tlm.flags&CC_FLAG_UNLOADING));
    CHECK(c.unload_offset!=0);
}
static void check_unload_stopped(void) {
    CHECK(!c.unloading && !c.unload_returning && c.unload_offset==0);
    CHECK(!(c.tlm.flags&CC_FLAG_UNLOADING));
}
static void unloading_imu(void) {
    start_unloading();tick(0);
    CHECK(!o.enable);check_unload_stopped();
    tick(1);CHECK(!o.enable);check_unload_stopped(); /* rebase only */
    tick(1);CHECK(o.enable && c.unloading); /* fresh restart */
    for(int mode=CC_MODE_IDLE;mode<=CC_MODE_WHEEL;mode++) {
        start_unloading();tick(0);cmd(CC_CMD_MODE,(float)mode,0);
        cmd(CC_CMD_WHEEL,20,0);tick(0);check_unload_stopped();
        CHECK(o.enable==(unsigned)(mode==CC_MODE_WHEEL));
    }
    start_unloading();uint64_t last=c.imu_last_valid_us;
    t=last+50001;
    CHECK(cc_step_with_imu(&c,t,&m,0,&o)==CC_IMU_RECAL_REQUIRED);
    CHECK(!o.enable);check_unload_stopped();
    tick(1);tick(1);CHECK(!o.enable && c.imu_recal_required);check_unload_stopped();
    cmd(CC_CMD_MODE,CC_MODE_WHEEL,0);tick(0);
    CHECK(o.enable && c.imu_recal_required);check_unload_stopped();
    cmd(CC_CMD_RECAL,0,0);
    for(int i=0;i<330;i++) tick(1);
    CHECK(c.calibrated && !c.imu_recal_required);check_unload_stopped();
}
int main(void) {
    vision();recal_clock();motor_independence();held_tlm();detection_islands();motor_command();
    recal_camera_age();unloading_imu();
    printf("Review regressions: %d failures\n",fails);return fails?1:0;
}
