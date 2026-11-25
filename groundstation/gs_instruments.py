"""Operational telemetry and workstation controls for the PyQt ground station."""
from collections import defaultdict, deque
import math
import time
import struct

from PyQt5 import QtCore, QtWidgets as W
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

import gs_protocol as P


# A missing value stays blank until its source reports it.
GROUPS = {
    'Identificação': 'Nome|MAC|Firmware|Build|ESP-IDF|Partição|Tempo ligado@s|Memória livre@kB|Memória mínima@kB|Flash / PSRAM|Módulos',
    'Wi-Fi e ligação': 'SSID|Endereço|Clientes|Sinal do cliente@dBm|Sinal da rede@dBm|Páginas abertas|Telemetria recebida@Hz|Pacotes perdidos|Latência PING@ms|Imagens recebidas@Hz|Tramas RX|Tramas TX',
    'IMU': 'Velocidade X@°/s|Velocidade Y@°/s|Velocidade Z@°/s|Aceleração X@m/s²|Aceleração Y@m/s²|Aceleração Z@m/s²',
    'USB JSON': 'Estado|Validade|Taxa X|Taxa Y|Taxa Z|Acelerómetro@Hz|Giro bruto X@°/s|Giro bruto Y@°/s|Giro bruto Z@°/s',
    'Magnetómetro': 'Campo X@µT|Campo Y@µT|Campo Z@µT|Magnitude@µT',
    'Atitude': 'Roll@°|Pitch@°|Yaw@°|Roll (acelerómetro)@°|Pitch (acelerómetro)@°|Quaternion w|Quaternion x|Quaternion y|Quaternion z|Erro X@°|Erro Y@°|Erro Z@°',
    'Bateria': 'Tensão recebida@V|Tensão bruta@V|Carga@%|Limiar local@V|Alarme local',
    'Controlo': 'Período@ms|Pior período@ms|Modo atual|Modo pedido|Roda no limite|Flags',
    'CAN-FD': 'Tramas enviadas|Tramas recebidas|Tramas perdidas',
    'Câmara': 'Capturas|Capturas falhadas|Imagens enviadas|Dados enviados@kB|Tempo de captura@ms|Tempo JPEG@ms|Luminância média|Luminância máxima|Foco|Campo horizontal@°|Resolução recebida|Última imagem@kB',
    'Deteção': 'Alvo detetado|Mensagens com alvo|Processamento@ms|Luminância máxima|Píxeis acima do limiar@px|Área detetada@px|Ângulo ao eixo@°|Ângulo X@°|Ângulo Y@°|Centro U@px|Centro V@px|Limiares',
    'Buzzer local': 'Modo|Ativo',
}
for axis in 'XYZ':
    GROUPS['Motor ' + axis] = 'Velocidade@rpm|Modo de comando|Referência recebida@rpm|Binário@N·m|Corrente Iq@A|Tensão@V|Temperatura@°C|Posição@rev|Modo moteus|Falha'

USB_WHEEL_STATES = {
    'ok': 'Telemetria recebida',
    'no_reply': 'Sem resposta',
    'controller_unavailable': 'Controlador indisponível',
    'query_not_sent': 'Consulta não enviada',
    'queue_recovery_failed': 'Falha na fila de transmissão',
}
USB_WHEEL_PRECISION = {'Velocidade': 3, 'Binário': 6, 'Posição': 6}


def button(text, slot=None):
    widget = W.QPushButton(text)
    if slot:
        widget.clicked.connect(slot)
    return widget


def disabled(text):
    widget = button(text)
    widget.setEnabled(False)
    widget.setToolTip('O protocolo atual não disponibiliza esta operação.')
    return widget


def quat(angles):
    r, p, y = (math.radians(a) / 2 for a in angles)
    cr, cp, cy = map(math.cos, (r, p, y))
    sr, sp, sy = map(math.sin, (r, p, y))
    return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)


class Instruments:
    def __init__(self, window):
        self.window = window
        self.monitor_group = 'Wi-Fi de fábrica' if window.factory_wifi else 'USB JSON'
        self.items, self.units, self.latest = {}, {}, {}
        self.history = defaultdict(lambda: deque(maxlen=3600))
        self.routes, self.panels = {}, {}
        self.next_panel = 1
        self.image_times, self.state_times = deque(), deque()
        self.usb_sent_times = deque(maxlen=40)
        self.usb_accel_times = deque(maxlen=12)
        self.usb_accel_rx = None
        self.hits = 0
        self.threshold = None
        self.alarm = False
        self.tone = 'OFF'
        self.next_beep = 0
        self.pulse_end = 0
        self.last_refresh = self.last_vector = 0
        self.motor_mask = 0
        self.vector_manual = False
        self.echo = None
        self.tree = W.QTreeWidget()
        self.tree.setHeaderLabels(['Grandeza', 'Leitura', 'Unidade', 'Gráfico'])
        self.tree.setAlternatingRowColors(True)
        for group, fields in GROUPS.items():
            if group == 'USB JSON' and window.factory_wifi:
                group = self.monitor_group
                fields = 'Estado|Validade|Taxa X|Taxa Y|Taxa Z'
            if window.read_only and group.startswith('Motor '):
                fields = 'Estado CAN|' + fields
            parent = W.QTreeWidgetItem(self.tree, [group])
            for field in fields.split('|'):
                name, _, unit = field.partition('@')
                key = (group, name)
                item = W.QTreeWidgetItem(parent, [name, '—', unit, ''])
                item.setData(0, QtCore.Qt.UserRole, key)
                self.items[key], self.units[key] = item, unit
            parent.setExpanded(True)
        self.tree.header().setSectionResizeMode(0, W.QHeaderView.Stretch)
        self.tree.setColumnWidth(1, 180)
        self.tree.itemClicked.connect(self.choose_plot)
        self.monitor = W.QWidget()
        layout = W.QVBoxLayout(self.monitor)
        search = W.QLineEdit()
        search.setPlaceholderText('Filtrar telemetria por grupo, nome ou unidade…')
        search.textChanged.connect(self.filter)
        layout.addWidget(search)
        layout.addWidget(self.tree)
        window.tabs.addTab(self.monitor, 'Telemetria')
        self.build_charts()
        self.build_console()
        for number, group, field in [(1, 'Motor '+a, 'Velocidade') for a in 'XYZ'] + [(2,'IMU','Velocidade '+a) for a in 'XYZ']:
            self.assign((group, field), number, 1)

    def filter(self, query):
        for key, item in self.items.items():
            item.setHidden(query.casefold() not in ' '.join((*key, self.units[key])).casefold())
        for i in range(self.tree.topLevelItemCount()):
            parent = self.tree.topLevelItem(i)
            parent.setHidden(all(parent.child(j).isHidden() for j in range(parent.childCount())))
            if query:
                parent.setExpanded(True)

    def put(self, group, name, value, now):
        key = (group, name)
        if isinstance(value, (int, float)):
            if not math.isfinite(value):
                return
            self.history[key].append((now, value))
        self.latest[key] = (now, value)

    def _monitor_wheel(self, axis, status, readings, now):
        group = 'Motor ' + axis
        self.put(group, 'Estado CAN', status, now)
        for field, value in readings:
            if isinstance(value, (int, float)) and not math.isfinite(value):
                value = None
            series = self.history.get((group, field))
            if series and math.isfinite(series[-1][1]) and (value is None or now - series[-1][0] > 2):
                series.append((now, math.nan))
            self.put(group, field, value, now)

    def _factory_telemetry(self, data, now):
        self.state_times.append(now)
        for axis, gyro, accel, mag in zip('XYZ', data.gyro, data.accel, data.mag):
            self.put('IMU', 'Velocidade ' + axis, gyro, now)
            self.put('IMU', 'Aceleração ' + axis, accel, now)
            self.put('Magnetómetro', 'Campo ' + axis, mag, now)
        for field, value in zip(('Tensão recebida', 'Tensão bruta', 'Carga'), data.battery):
            self.put('Bateria', field, value if math.isfinite(value) and data.battery[0] > 0 else None, now)
        for field, value in zip(('Tramas enviadas', 'Tramas recebidas', 'Tramas perdidas'), data.can):
            self.put('CAN-FD', field, value, now)
        self.put('Identificação', 'Tempo ligado', data.uptime_ms / 1000, now)
        fields = ('Posição', 'Velocidade', 'Binário', 'Tensão', 'Corrente Iq', 'Temperatura', 'Falha', 'Modo moteus')
        for axis, readings in zip('XYZ', data.wheels):
            pos, speed, torque, voltage, current, temp, fault, mode = readings
            present = any(math.isfinite(value) and value != 0 for value in readings)
            values = (pos, 60 * speed, torque, voltage, current, temp, fault, mode) if present else (None,) * len(fields)
            status = 'Telemetria recebida' if present else 'Sem leitura da roda'
            self._monitor_wheel(axis, status, zip(fields, values), now)

    def ingest(self, kind, obj, now):
        def put(group, name, value):
            self.put(group, name, value, now)
        if kind == 'usb_wheel':
            readings = (('Velocidade', obj.rpm), ('Binário', obj.torque_nm), ('Tensão', obj.voltage_v),
                        ('Temperatura', obj.temperature_c), ('Posição', obj.position_rev),
                        ('Modo moteus', obj.mode), ('Falha', obj.fault))
            self._monitor_wheel(obj.axis, USB_WHEEL_STATES[obj.status], readings, now)
        elif kind == 'wifi_telemetry':
            self._factory_telemetry(obj, now)
        elif kind == 'state':
            self.state_times.append(now)
            put('Bateria','Tensão recebida',obj.vbat)
            put('Identificação','Tempo ligado',obj.t_ms/1000)
            put('Controlo','Modo atual',P.MODES.get(obj.mode, str(obj.mode)))
            put('Controlo','Flags',hex(obj.flags))
            put('Controlo','Roda no limite','Sim' if obj.flag(1) else 'Não')
            put('Motor Z','Velocidade',obj.wheel_rpm)
            put('Motor Z','Referência recebida',obj.wheel_cmd_rpm)
            put('Motor Z','Binário',obj.torque)
            put('Atitude','Yaw',obj.heading)
        elif kind in ('state3', 'sim3'):
            if kind == 'state3':
                status, mask, states = obj
                self.vector_manual = all(s.mode==2 for s in states)
                self.motor_mask = mask if status == 0 else 0
                self.last_vector = now
                if status != 0:
                    return
                angles, rates, errors, speeds, refs, torques = (
                    tuple(getattr(s, attr) for s in states)
                    for attr in ('heading','rate','err','wheel_rpm','wheel_cmd_rpm','torque'))
            else:
                mask = 7
                angles, rates, errors = obj.attitude, obj.rates, obj.errors
                speeds, refs, torques = obj.wheels, obj.commands, obj.torques
            for i, axis in enumerate('XYZ'):
                group = 'Motor ' + axis
                if mask & (1 << i):
                    put(group,'Velocidade',speeds[i])
                    put(group,'Referência recebida',refs[i])
                    put(group,'Binário',torques[i])
                put('IMU','Velocidade '+axis,rates[i])
                put('Atitude','Erro '+axis,errors[i])
            self.attitude(angles, now)
        elif kind == 'gimbal':
            self.attitude(obj.attitude, now)
        elif kind == 'vision':
            self.hits += int(bool(obj.found))
            put('Deteção','Alvo detetado','Sim' if obj.found else 'Não')
            put('Deteção','Mensagens com alvo',self.hits)
            put('Deteção','Processamento',obj.proc_us/1000)
            if obj.found:
                for field, value in [('Centro U',obj.x),('Centro V',obj.y),('Área detetada',obj.area),('Ângulo X',obj.err_x),('Ângulo Y',obj.err_y)]:
                    put('Deteção', field, value)
                horizontal, vertical = map(math.radians, (obj.err_x, obj.err_y))
                put('Deteção','Ângulo ao eixo',math.degrees(math.atan(math.hypot(math.tan(horizontal),math.tan(vertical)))))
        elif kind == 'camera':
            put('Câmara','Campo horizontal',obj.hfov)
        elif kind == 'ack' and self.echo and tuple(obj[:2]) == (P.TC_PING, self.echo[0]):
            put('Wi-Fi e ligação','Latência PING',(now-self.echo[1])*1000)
            self.echo = None
        elif kind == 'ack' and obj[0] == 0x89:
            self.manual_status.setText('Referências aceites.' if obj[2]==0 else 'Referências recusadas pelo cubo.')

    @staticmethod
    def _record_usb_time(times, timestamp):
        if times and (timestamp < times[-1] or timestamp - times[-1] > 2_000_000):
            times.clear()
        if not times or timestamp > times[-1]:
            times.append(timestamp)

    def usb_acceleration_rate(self, now):
        times = self.usb_accel_times
        if len(times) < 2 or self.usb_accel_rx is None or now - self.usb_accel_rx > 2:
            return 0.
        return (len(times) - 1) * 1e6 / (times[-1] - times[0])

    def usb_attitude(self, sample, angles, now):
        self._record_usb_time(self.usb_sent_times, sample.sent_us)
        self.put(self.monitor_group, 'Estado', str(sample.state), now)
        self.put(self.monitor_group, 'Validade', hex(sample.validity), now)
        angle_names = ('Roll', 'Pitch', 'Yaw')
        q_names = tuple('Quaternion ' + axis for axis in 'wxyz')
        rate_names = tuple('Taxa ' + axis for axis in 'XYZ')
        if angles is None:
            for group, names in (('Atitude', angle_names + q_names), (self.monitor_group, rate_names)):
                for name in names:
                    key = (group, name)
                    self.latest.pop(key, None)
                    self.items[key].setText(1, '—')
            return
        for group, names, values in (
                ('Atitude', angle_names, angles), ('Atitude', q_names, sample.q),
                (self.monitor_group, rate_names, sample.rates)):
            for name, value in zip(names, values):
                self.put(group, name, value, now)

    def usb_acceleration(self, sample, tilt, now):
        self._record_usb_time(self.usb_accel_times, sample.sample_us)
        self.usb_accel_rx = now
        if sample.body_rates is not None:
            for axis, rate in zip('XYZ', sample.body_rates):
                self.put('USB JSON', 'Giro bruto ' + axis, math.degrees(rate), now)
        for axis, value in zip('XYZ', sample.body_acceleration):
            self.put('IMU', 'Aceleração ' + axis, value, now)
        for index, name in enumerate(('Roll (acelerómetro)', 'Pitch (acelerómetro)')):
            if tilt is None:
                key = ('Atitude', name)
                self.latest.pop(key, None)
                self.items[key].setText(1, '—')
            else:
                self.put('Atitude', name, tilt[index], now)

    def attitude(self, angles, now):
        if not all(math.isfinite(a) for a in angles):
            return
        for key, value in zip(('Roll','Pitch','Yaw'),angles):
            self.put('Atitude',key,value,now)
        for axis, value in zip('wxyz',quat(angles)):
            self.put('Atitude','Quaternion '+axis,value,now)

    def image(self, array, size):
        now = time.monotonic()
        self.image_times.append(now)
        h, w = array.shape[:2]
        self.put('Câmara','Resolução recebida',f'{w} × {h}',now)
        self.put('Câmara','Última imagem',size/1000,now)
        lum = array if array.ndim==2 else array @ [0.299,0.587,0.114]
        self.put('Câmara','Luminância média',float(lum.mean()),now)
        self.put('Câmara','Luminância máxima',int(lum.max()),now)
        if self.window.thresholds.isChecked():
            vis = self.window.pending_vis
            u,v = w//2,h//2
            if vis and vis.found and 0<=vis.x<320 and 0<=vis.y<240:
                u,v = int(vis.x*w/320),int(vis.y*h/240)
            radius = int(self.window.radius.value())
            region = lum[max(0,v-radius):v+radius+1,max(0,u-radius):u+radius+1]
            peak = float(region.max())
            fraction = self.window.fraction.value()
            minimum = self.window.peak.value()
            count = int((region>=peak*fraction).sum()) if peak>=minimum else 0
            self.put('Deteção','Luminância máxima',peak,now)
            self.put('Deteção','Píxeis acima do limiar',count,now)
            self.put('Deteção','Limiares',f'{minimum:.0f} / {fraction:.2f} / ±{radius} px (local)',now)

    def build_charts(self):
        self.charts = W.QWidget()
        col = W.QVBoxLayout(self.charts)
        bar = W.QHBoxLayout()
        self.duration = W.QComboBox()
        self.duration.addItems(['10','30','60','120'])
        self.duration.setCurrentText('30')
        bar.addWidget(W.QLabel('Janela (s)'))
        bar.addWidget(self.duration)
        self.pause = button('Pausa')
        self.pause.setCheckable(True)
        bar.addWidget(self.pause)
        bar.addWidget(button('+ Painel',self.add_panel))
        bar.addStretch()
        col.addLayout(bar)
        scroll = W.QScrollArea()
        scroll.setWidgetResizable(True)
        inner = W.QWidget()
        self.chart_layout = W.QVBoxLayout(inner)
        self.chart_layout.setAlignment(QtCore.Qt.AlignTop)
        scroll.setWidget(inner)
        col.addWidget(scroll)
        self.window.tabs.addTab(self.charts,'Gráficos')
        self.add_panel()
        self.add_panel()

    def add_panel(self):
        number = self.next_panel
        self.next_panel += 1
        box = W.QGroupBox(f'Painel {number}')
        layout = W.QVBoxLayout(box)
        layout.addWidget(button('Remover painel',lambda: self.remove_panel(number)), 0, QtCore.Qt.AlignRight)
        canvas = FigureCanvasQTAgg(Figure(figsize=(7,2.4)))
        canvas.setMinimumHeight(225)
        layout.addWidget(canvas)
        self.chart_layout.addWidget(box)
        self.panels[number] = (box,canvas)
        self.draw_plots()

    def remove_panel(self, number):
        box, _ = self.panels.pop(number)
        box.deleteLater()
        for key, route in list(self.routes.items()):
            if route[0] == number:
                self.assign(key,None,None)

    def assign(self, key, panel, axis):
        if panel is None:
            self.routes.pop(key,None)
            self.items[key].setText(3,'Escolher…')
        else:
            self.routes[key] = (panel,axis)
            self.items[key].setText(3,f'P{panel} · Y{axis}')

    def choose_plot(self,item,column):
        key = item.data(0,QtCore.Qt.UserRole)
        if column != 3 or not key or tuple(key) not in self.history:
            return
        key = tuple(key)
        menu = W.QMenu(self.tree)
        menu.addAction('Retirar do gráfico',lambda: self.assign(key,None,None))
        for number in self.panels:
            sub = menu.addMenu(f'Painel {number}')
            units = {axis:self.units[k] for k,(n,axis) in self.routes.items() if n==number and k!=key}
            for axis in range(1,5):
                unit = units.get(axis, self.units[key])
                action = sub.addAction(f'Y{axis} · {unit or "sem unidade"}',lambda _,n=number,a=axis:self.assign(key,n,a))
                action.setEnabled(unit == self.units[key])
        from PyQt5.QtGui import QCursor
        menu.exec_(QCursor.pos())

    def draw_plots(self):
        if self.pause.isChecked() or not self.charts.isVisible():
            return
        now = time.monotonic()
        span = int(self.duration.currentText())
        for number, (_,canvas) in self.panels.items():
            fig = canvas.figure
            fig.clear()
            base = fig.add_subplot(111)
            axes = {1:base}
            count = 0
            for key,(panel,axis) in self.routes.items():
                if panel != number:
                    continue
                if axis not in axes:
                    other = base.twinx()
                    other.spines['right'].set_position(('axes',1+.09*(len(axes)-1)))
                    axes[axis] = other
                data = [(t-now,v) for t,v in self.history.get(key,()) if t >= now-span]
                x,y = zip(*data) if data else ([],[])
                axes[axis].plot(x,y,color=f'C{count%10}',label=' · '.join(key),linewidth=1.3)
                axes[axis].set_ylabel(f'Y{axis} ({self.units[key] or "1"})',fontsize=8)
                count += 1
            base.set_xlim(-span,0)
            base.set_xlabel('Tempo (s)')
            base.grid(alpha=.2)
            lines = [line for a in axes.values() for line in a.lines]
            if lines:
                base.legend(lines,[l.get_label() for l in lines],fontsize=7,loc='upper left',ncol=3)
            fig.subplots_adjust(left=.09,right=.78 if len(axes)>2 else .9,bottom=.24,top=.94)
            canvas.draw_idle()


    def build_console(self):
        page = W.QWidget()
        col = W.QVBoxLayout(page)
        row = W.QHBoxLayout()
        self.editors, self.readouts = [], []
        for axis in 'XYZ':
            box = W.QGroupBox('Roda '+axis)
            form = W.QVBoxLayout(box)
            mode = W.QComboBox()
            mode.addItems(['Velocidade (rpm)', 'Binário (N·m) · pré-visualização'])
            slider = W.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(-3000,3000)
            value = W.QDoubleSpinBox()
            value.setRange(-3000,3000)
            value.setDecimals(0)
            value.setSingleStep(50)
            slider.valueChanged.connect(lambda n,v=value,m=mode:v.setValue(n/1000 if m.currentIndex() else n))
            value.valueChanged.connect(lambda n,s=slider,m=mode:s.setValue(round(n*1000 if m.currentIndex() else n)))
            def configure(index,s=slider,v=value):
                s.setRange(-200 if index else -3000,200 if index else 3000)
                v.setDecimals(3 if index else 0)
                limit = .2 if index else 3000
                v.setRange(-limit,limit)
                v.setSingleStep(.01 if index else 50)
                v.setValue(0)
            mode.currentIndexChanged.connect(configure)
            for widget in (mode,slider,value,button('Zero',lambda _,v=value:v.setValue(0))):
                if self.window.read_only:
                    widget.setEnabled(False)
                form.addWidget(widget)
            reading = W.QLabel('—')
            form.addWidget(reading)
            self.readouts.append(reading)
            self.editors.append((mode,value))
            row.addWidget(box)
        col.addLayout(row)
        row = W.QHBoxLayout()
        self.manual_status = W.QLabel(f'Telemetria {self.window.monitor_transport} · só leitura' if self.window.read_only else
                                      'A aguardar ligação com controlo individual das rodas.')
        self.manual_send = button('Aplicar velocidades X/Y/Z',self.apply_wheels)
        self.manual_send.setEnabled(False)
        row.addWidget(self.manual_send)
        row.addWidget(self.manual_status)
        row.addStretch()
        col.addLayout(row)
        row = W.QHBoxLayout()
        alarm = W.QGroupBox('Alarmes no computador')
        form = W.QFormLayout(alarm)
        tone = W.QComboBox()
        tone.addItems(['OFF','ON','Auto'])
        tone.currentTextChanged.connect(self.set_tone)
        form.addRow('Buzzer local',tone)
        pulse = W.QDoubleSpinBox()
        pulse.setRange(.1,10)
        pulse.setValue(.5)
        pulse.setSuffix(' s')
        form.addRow(button('Pulso',lambda:self.pulse(pulse.value())),pulse)
        limit = W.QDoubleSpinBox()
        limit.setRange(.1,60)
        limit.setSuffix(' V')
        limit.setValue(10)
        form.addRow('Limiar de tensão',limit)
        form.addRow(button('Aplicar limiar local',lambda:self.set_threshold(limit.value())))
        self.alarm_label = W.QLabel('Alarme desativado')
        form.addRow(self.alarm_label)
        row.addWidget(alarm)
        firmware = W.QGroupBox('Firmware e identidade')
        form = W.QFormLayout(firmware)
        self.identity = W.QLabel('—')
        form.addRow('Ligação',self.identity)
        self.file_label = W.QLabel('Nenhum ficheiro')
        form.addRow(button('Escolher firmware…',self.choose_file),self.file_label)
        form.addRow(disabled('Atualizar por Wi-Fi'),disabled('Voltar à partição de fábrica'))
        self.rename = W.QLineEdit()
        self.rename.setPlaceholderText('Nome do CubeSat')
        form.addRow(self.rename,disabled('Renomear'))
        form.addRow(disabled('Reiniciar CubeSat'))
        form.addRow(W.QLabel('Operações OTA e identidade aguardam suporte do firmware.'))
        row.addWidget(firmware)
        col.addLayout(row)
        col.addStretch()
        self.window.tabs.addTab(page,'Consola')

    def choose_file(self):
        from pathlib import Path
        path,_ = W.QFileDialog.getOpenFileName(self.window,'Selecionar imagem de firmware','','Firmware (*.bin)')
        if path:
            self.file_label.setText(Path(path).name)
            self.file_label.setToolTip(path)

    def apply_wheels(self):
        # Only the versioned three-axis protocol addresses independent wheels.
        if not self.manual_ready(time.monotonic()):
            return
        values = [v.value() for _,v in self.editors]
        self.window.send(lambda seq:P.pack_frame(0x89,seq,struct.pack('<BB3f',1,2,*values)))
        self.manual_status.setText('Referências enviadas · a aguardar confirmação')

    def manual_ready(self,now):
        return (now-self.last_vector < 1 and self.motor_mask == 7 and
                self.vector_manual and all(m.currentIndex()==0 for m,_ in self.editors))

    def set_tone(self,tone):
        self.tone = tone
        self.next_beep = 0
        if tone=='OFF':
            self.pulse_end = 0

    def pulse(self,seconds):
        self.pulse_end = time.monotonic()+seconds
        W.QApplication.beep()
        self.next_beep = time.monotonic()+1

    def set_threshold(self,value):
        self.threshold = value
        self.alarm_label.setText(f'Limiar local aplicado · {value:.1f} V')

    def refresh(self,now):
        if now-self.last_refresh < .5:
            return
        self.last_refresh = now
        link = self.window.link
        addr = getattr(link,'addr',None)
        endpoint = f'{addr[0]}:{addr[1]}' if addr else getattr(getattr(link,'ser',None),'port','—')
        self.identity.setText(endpoint)
        self.put('Wi-Fi e ligação','Endereço',endpoint,now)
        for name,queue in [('Imagens recebidas',self.image_times),('Telemetria recebida',self.state_times)]:
            while queue and now-queue[0]>2:
                queue.popleft()
            rate = (len(queue)-1)/(queue[-1]-queue[0]) if len(queue)>1 and queue[-1]>queue[0] else 0.
            self.put('Wi-Fi e ligação',name,rate,now)
        if self.window.read_only and not self.window.factory_wifi:
            times = self.usb_sent_times
            fresh = self.window.last_rx is not None and now - self.window.last_rx <= 2
            quaternion_rate = (len(times) - 1) * 1e6 / (times[-1] - times[0]) if fresh and len(times) > 1 else 0.0
            accel_rate = self.usb_acceleration_rate(now)
            self.put('USB JSON', 'Acelerómetro', accel_rate, now)
            attitude_rx = self.window.usb_attitude_rx
            active_rate = quaternion_rate if attitude_rx is not None and now - attitude_rx <= 2 else accel_rate
            self.put('Wi-Fi e ligação', 'Telemetria recebida', active_rate, now)
        self.put('Wi-Fi e ligação','Tramas RX',link.n_rx,now)
        self.put('Wi-Fi e ligação','Tramas TX',link.n_tx,now)
        def read(group,name):
            t,v = self.latest.get((group,name),(0,None))
            return v if now-t<=2 else None
        voltage = read('Bateria','Tensão recebida')
        self.alarm = self.threshold is not None and voltage is not None and voltage < self.threshold
        if self.threshold is not None:
            self.put('Bateria','Limiar local',self.threshold,now)
            msg = 'Sem leitura' if voltage is None else ('Tensão baixa' if self.alarm else 'Normal')
            self.put('Bateria','Alarme local',msg,now)
            self.alarm_label.setText(f'{msg} · limiar {self.threshold:.1f} V')
        sounding = self.tone=='ON' or now<self.pulse_end or (self.tone=='Auto' and self.alarm)
        if sounding and now>=self.next_beep:
            W.QApplication.beep()
            self.next_beep = now+1
        self.put('Buzzer local','Modo',self.tone,now)
        self.put('Buzzer local','Ativo','Sim' if sounding else 'Não',now)
        for key, (stamp,v) in self.latest.items():
            # Ping is intermittent. Every other live reading expires after two seconds.
            fresh = now-stamp <= (3 if key[1]=='Latência PING' else 2)
            item = self.items[key]
            precision = 5 if key[0] == self.monitor_group or key[1].startswith('Quaternion ') else 2
            if self.window.read_only and key[0].startswith('Motor '):
                precision = USB_WHEEL_PRECISION.get(key[1], precision)
            reading = f'{v:.{precision}f}'.rstrip('0').rstrip('.') if isinstance(v, float) else str(v)
            item.setText(1, reading if fresh and v is not None else '—')
            if key in self.history and not item.text(3):
                item.setText(3,'Escolher…')
        usb_wheel_summary = []
        for axis, label in zip('XYZ', self.readouts):
            group = 'Motor ' + axis
            speed, torque = (read(group, field) for field in ('Velocidade', 'Binário'))
            if self.window.read_only:
                status = read(group, 'Estado CAN')
                if status is None:
                    status = 'Sem dados recentes' if (group, 'Estado CAN') in self.latest else 'A aguardar dados'
                fault = read(group, 'Falha')
                summary = f'{speed:.0f} rpm' if speed is not None else status
                label.setText(f'{speed:.0f} rpm  ·  {torque:.3f} N·m' if speed is not None and torque is not None else status)
                if fault:
                    summary += f' · Falha {fault}'
                usb_wheel_summary.append(f'{axis} · {summary}')
            else:
                label.setText('Sem telemetria' if speed is None or torque is None else f'{speed:.0f} rpm  ·  {torque:.3f} N·m')
        if self.window.read_only:
            self.window.saturation.setText('\n'.join(usb_wheel_summary))
        self.manual_send.setEnabled(self.manual_ready(now))
        self.manual_send.setToolTip('Requer telemetria Core3 recente, três motores válidos e modo WHEEL confirmado.')
        live = (self.window.last_rx is not None and now-self.window.last_rx <= 2
                if self.window.read_only else voltage is not None)
        def number(k,n,unit):
            val = read(k,n)
            return '—' if val is None else f'{val:.1f} {unit}'
        self.window.instrument_status.setText('  ·  '.join([
            'Ligado' if live else 'A aguardar telemetria',
            number('Bateria','Tensão recebida','V'),number('Wi-Fi e ligação','Telemetria recebida','Hz'),
            number('Wi-Fi e ligação','Latência PING','ms'),str(read('Controlo','Modo atual') or '—')]))
        q = [read('Atitude','Quaternion '+c) for c in 'wxyz']
        self.window.quat_label.setText('q [w, x, y, z]   '+(' / '.join(f'{v:.3f}' for v in q) if all(v is not None for v in q) else '—'))
        self.draw_plots()
