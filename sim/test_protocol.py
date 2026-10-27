#!/usr/bin/env python3
"""test_protocol.py - o protocolo em C (firmware) e a implementação Python (estação de terra) têm de concordar."""
import math, os, struct, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(os.path.dirname(HERE), "groundstation"))
import gs_protocol as gs
import cproto as cp
from sil import TlmT, VisionT, CMD_MODE, CMD_WHEEL, CMD_RATE, CMD_HEADING, CMD_AIM, CMD_GAIN, CMD_RECAL

passed = 0
def ok(name, cond):
    global passed
    assert cond, "FALHOU: " + name
    passed += 1
    print("  ok  ", name)

# 1. CRC (valor de verificação CRC-16/CCITT-FALSE)
ok("CRC16('123456789') == 0x29B1 (C e Python)", cp.crc16(b"123456789") == 0x29B1 == gs.crc16(b"123456789"))

# 2. tramas: C == Python, e cada lado lê o que o outro escreve
rng = np.random.default_rng(0)
same = True
for n in (0, 1, 7, 52, 205, 240):
    pl = bytes(rng.integers(0, 256, n, dtype=np.uint8))
    same &= cp.pack_frame(0x42, 9, pl) == gs.pack_frame(0x42, 9, pl)
ok("pack_frame idêntico em C e Python (0..240 B)", same)
frames = gs.pack_frame(0x01, 1, b"abc") + b"\x00\xA5\xA5\x12garbage" + cp.pack_frame(0x02, 2, b"xyz") + gs.pack_frame(0x03, 3)
got_py = list(gs.Parser().feed(frames)); got_c = list(cp.CParser().feed(frames))
ok("ressincroniza após lixo; Python == C", got_py == got_c and [g[0] for g in got_py] == [1, 2, 3])
bad = bytearray(gs.pack_frame(0x01, 1, b"abc")); bad[6] ^= 0xFF
p = gs.Parser(); cpar = cp.CParser()
ok("CRC errado é rejeitado (C e Python)", list(p.feed(bytes(bad))) == [] and list(cpar.feed(bytes(bad))) == [] and p.crc_errors == 1 and cpar.p.n_crc_err == 1)

# 3. TM_STATE
t = TlmT(); t.t_ms = 123456; t.mode = 5; t.trk = 1; t.flags = 0b1101
vals = dict(heading=math.radians(12.5), rate=math.radians(-3.0), bias=math.radians(0.25), err=math.radians(7.0),
            rate_ref=math.radians(20.0), wheel_rps=100 * 2 * math.pi / 60, wheel_cmd_rps=-250 * 2 * math.pi / 60,
            torque_nm=0.05, vbat_v=22.4, target_err=math.radians(-8.0), cam_age_s=0.07)
for k, v in vals.items(): setattr(t, k, v)
s = gs.decode_state(cp.pack_state(t))
ok("TM_STATE tem 52 B e converte unidades (°, °/s, rpm)",
   len(cp.pack_state(t)) == struct.calcsize(gs.STATE_FMT) == 52 and s.t_ms == 123456 and s.mode == 5 and s.flags == 13
   and abs(s.heading - 12.5) < 1e-4 and abs(s.rate + 3.0) < 1e-4 and abs(s.wheel_rpm - 100) < 1e-3
   and abs(s.wheel_cmd_rpm + 250) < 1e-3 and abs(s.target_err + 8.0) < 1e-4 and abs(s.cam_age - 0.07) < 1e-6)

# 4. TM_VISION
v = VisionT(); v.t_capture_ms = 777; v.found = 1; v.accepted = 1; v.n_cand = 3; v.x = 100.5; v.y = 60.25
v.err_x = math.radians(-9.5); v.err_y = math.radians(1.0); v.contrast = 150.0; v.area = 314
vv = gs.decode_vision(cp.pack_vision(v, 4321))
ok("TM_VISION tem 36 B e descodifica", len(cp.pack_vision(v, 1)) == struct.calcsize(gs.VISION_FMT) == 36 and vv.x == 100.5 and
   abs(vv.err_x + 9.5) < 1e-4 and vv.area == 314 and vv.proc_us == 4321 and vv.n_cand == 3)

# 5. imagem em chunks
img = rng.integers(0, 256, (240, 320), dtype=np.uint8)
asm = gs.ImageAssembler(); out = None
for c in range(cp.IMG_NCHUNKS):
    out = asm.feed(cp.pack_img_chunk(img, 5, c)) or out
ref = img.reshape(60, 4, 80, 4).mean(axis=(1, 3)).astype(np.uint8)
ok("TM_IMG: 24 chunks -> 80x60 = média 4x4 da imagem", out is not None and out[0] == 5 and np.abs(out[1].astype(int) - ref.astype(int)).max() <= 1)

# 6. telecomandos Python -> C -> comando do núcleo (unidades SI)
def tc(frame):
    (ty, sq, pl), = list(gs.Parser().feed(frame))
    return cp.tc_to_cmd(ty, pl)
r, c = tc(gs.tc_mode(1, 5)); ok("TC modo", r == 1 and c.type == CMD_MODE and c.a == 5)
r, c = tc(gs.tc_wheel(1, 3000)); ok("TC roda 3000 rpm -> 314.16 rad/s", r == 1 and c.type == CMD_WHEEL and abs(c.a - 314.159) < 0.01)
r, c = tc(gs.tc_rate(1, 30)); ok("TC taxa 30 °/s -> rad/s", abs(c.a - math.radians(30)) < 1e-6)
r, c = tc(gs.tc_heading(1, -45)); ok("TC rumo -45° -> rad", c.type == CMD_HEADING and abs(c.a + math.radians(45)) < 1e-6)
r, c = tc(gs.tc_aim(1, 10)); ok("TC pontaria 10° -> rad", c.type == CMD_AIM and abs(c.a - math.radians(10)) < 1e-6)
r, c = tc(gs.tc_gain(1, 5, 2500)); ok("TC ganho wheel_lim 2500 rpm -> rad/s", c.type == CMD_GAIN and c.id == 5 and abs(c.a - 261.8) < 0.1)
r, c = tc(gs.tc_gain(1, 0, 4.5)); ok("TC ganho k_theta sem conversão", c.id == 0 and abs(c.a - 4.5) < 1e-6)
r, c = tc(gs.tc_recal(1)); ok("TC recalibrar", r == 1 and c.type == CMD_RECAL)
ok("TC modo inválido -> erro", tc(gs.tc_mode(1, 0))[0] < 0 and cp.tc_to_cmd(gs.TC_SET_WHEEL, b"\0\0")[0] < 0)
ok("TC ping/desconhecido -> não é do núcleo", tc(gs.tc_ping(1))[0] == 0)

# 7. moteus (formato da documentação oficial mjbots)
doc_reply = bytes.fromhex("2404000a005000000170ff230d181400")   # exemplo da doc protocol/can
r, st = cp.moteus_parse(doc_reply)
ok("moteus: resposta de exemplo da doc (modo 10, 0.008 rev, 0.064 rev/s, -1.44 N·m, 12 V, 20 °C)",
   r == 0 and st.mode == 10 and abs(st.position_rev - 0.008) < 1e-6 and abs(st.velocity_rev_s - 0.064) < 1e-6
   and abs(st.torque_nm + 1.44) < 1e-6 and st.voltage_v == 12.0 and st.temp_c == 20.0 and st.fault == 0)
cmd = cp.moteus_velocity(50.0, 0.2, 127.0)
ok("moteus: comando de velocidade = 48 B, começa por modo=posição e 6 floats em 0x020",
   len(cmd) == 48 and cmd[:6] == bytes([0x01, 0x00, 0x0a, 0x0c, 0x06, 0x20]) and math.isnan(struct.unpack_from("<f", cmd, 6)[0])
   and struct.unpack_from("<f", cmd, 10)[0] == 50.0 and cmd[-1] == 0x50)
ok("moteus: stop = modo 0; id CAN 0x8001 (origem 0, destino 1, pede resposta)",
   cp.moteus_stop(False)[:3] == bytes([1, 0, 0]) and cp.lib().moteus_can_id(0, 1, 1) == 0x8001 and cp.lib().moteus_can_id(1, 0, 0) == 0x100)
print(f"\n{passed} verificações OK")
