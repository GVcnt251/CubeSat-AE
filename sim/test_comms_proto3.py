"""Run real firmware comms.c + core with host Wi-Fi/UDP/RTOS stubs.

No hardware or network required. Run: python sim/test_comms_proto3.py
Requires host GCC (PATH, CC, or MSYS2 UCRT64).
"""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

STUBS = r'''
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "cc_proto3.h"
#include "wire_le.h"

typedef int esp_event_base_t;
enum { WIFI_EVENT = 1, IP_EVENT, WIFI_EVENT_STA_START,
       WIFI_EVENT_STA_DISCONNECTED, IP_EVENT_STA_GOT_IP };
#define ESP_EVENT_ANY_ID -1
#define ESP_ERROR_CHECK(v) assert((v) == 0)
#define ESP_LOGI(tag, ...) do { (void)(tag); if (0) printf(__VA_ARGS__); } while (0)
#define ESP_LOGW ESP_LOGI
#define ESP_LOGE ESP_LOGI
#define IPSTR "%u"
#define IP2STR(ip) (*(ip))
#define CONFIG_CUBE_GS_PORT 5000
#define CONFIG_CUBE_WIFI_SSID "test"
#define CONFIG_CUBE_WIFI_PASS "test"
#define WIFI_MODE_STA 0
#define WIFI_IF_STA 0
#define WIFI_PS_NONE 0
typedef struct { struct { unsigned ip; } ip_info; } ip_event_got_ip_t;
typedef struct { int unused; } wifi_init_config_t;
#define WIFI_INIT_CONFIG_DEFAULT() {0}
typedef struct { struct { uint8_t ssid[32], password[64]; } sta; } wifi_config_t;
static int esp_wifi_connect(void) { return 0; }
static int esp_netif_init(void) { return 0; }
static int esp_event_loop_create_default(void) { return 0; }
static void esp_netif_create_default_wifi_sta(void) {}
static int esp_wifi_init(const wifi_init_config_t *cfg) { (void)cfg; return 0; }
static int esp_event_handler_register(esp_event_base_t base, int id,
    void (*cb)(void *, esp_event_base_t, int32_t, void *), void *arg) {
    (void)base; (void)id; (void)cb; (void)arg; return 0;
}
static int esp_wifi_set_mode(int mode) { (void)mode; return 0; }
static int esp_wifi_set_config(int iface, wifi_config_t *cfg) {
    (void)iface; (void)cfg; return 0;
}
static int esp_wifi_start(void) { return 0; }
static int esp_wifi_set_ps(int ps) { (void)ps; return 0; }

typedef struct { int held; } mutex_t;
typedef mutex_t *SemaphoreHandle_t;
static mutex_t tx_mutex, core_mutex;
static SemaphoreHandle_t xSemaphoreCreateMutex(void) { return &tx_mutex; }
static void xSemaphoreTake(SemaphoreHandle_t m, int timeout) {
    (void)timeout; assert(m && !m->held); m->held = 1;
}
static void xSemaphoreGive(SemaphoreHandle_t m) {
    assert(m && m->held); m->held = 0;
}
#define configASSERT assert
#define portMAX_DELAY -1
#define pdMS_TO_TICKS(v) (v)
cc3_t g_core;
SemaphoreHandle_t g_core_mtx = &core_mutex;
#define CORE_LOCK() xSemaphoreTake(g_core_mtx, portMAX_DELAY)
#define CORE_UNLOCK() xSemaphoreGive(g_core_mtx)

typedef unsigned socklen_t;
struct sockaddr { int unused; };
struct sockaddr_in { int sin_family; uint16_t sin_port; struct { uint32_t s_addr; } sin_addr; };
#define AF_INET 2
#define SOCK_DGRAM 2
#define IPPROTO_IP 0
#define INADDR_ANY 0
#define MSG_DONTWAIT 1
#define htons(v) (v)
#define htonl(v) (v)
static int socket(int af, int type, int proto) { (void)af; (void)type; (void)proto; return 1; }
static int bind(int s, const struct sockaddr *a, socklen_t len) { (void)s; (void)a; (void)len; return 0; }
static void close(int s) { (void)s; }
static uint8_t datagram[512];
static int datagram_len, sent_count;
static uint32_t peer = 123;
static cc_parser_t sent[32];
static int64_t clock_us = 100000;
static int64_t esp_timer_get_time(void) { return clock_us; }
static jmp_buf task_end;
static int stop_task;
static void vTaskDelay(int ticks) { (void)ticks; if (stop_task) longjmp(task_end, 1); }
static int recvfrom(int s, void *out, size_t capacity, int flags,
    struct sockaddr *from, socklen_t *len) {
    (void)s; (void)flags; (void)len;
    assert((size_t)datagram_len <= capacity);
    memcpy(out, datagram, (size_t)datagram_len);
    ((struct sockaddr_in *)from)->sin_addr.s_addr = peer;
    int n = datagram_len; datagram_len = 0; return n;
}
static int sendto(int s, const void *data, size_t n, int flags,
    const struct sockaddr *to, socklen_t len) {
    (void)s; (void)flags; (void)len;
    assert(((const struct sockaddr_in *)to)->sin_addr.s_addr == peer);
    assert(sent_count < 32);
    cc_parser_t *p = &sent[sent_count++]; cc_parser_init(p);
    int complete = 0;
    for (size_t i = 0; i < n; i++) complete += cc_parser_feed(p, ((const uint8_t *)data)[i]);
    assert(complete == 1 && p->n_crc_err == 0);
    assert(n == (size_t)p->len + CC_FRAME_OVERHEAD);
    return (int)n;
}
'''

TESTS = r'''
static void run_datagram(uint8_t type, const uint8_t *pl, uint8_t len, int damage) {
    sent_count = 0;
    datagram_len = (int)cc_frame_pack(datagram, type, 42, pl, len);
    if (damage == 1) datagram[datagram_len - 1] ^= 1;
    if (damage == 2) datagram_len--;
    if (damage == 3) datagram[datagram_len++] = 0;
    stop_task = 1;
    if (!setjmp(task_end)) comms_task(NULL);
    stop_task = 0;
    assert(!tx_mutex.held && !core_mutex.held);
}
static void expect_ack(uint8_t type, const uint8_t *pl, uint8_t len, uint8_t status) {
    run_datagram(type, pl, len, 0);
    assert(sent_count == 1 && sent[0].type == TM_ACK && sent[0].len == 3);
    assert(sent[0].payload[0] == type && sent[0].payload[1] == 42 && sent[0].payload[2] == status);
}
int main(void) {
    assert(cc3_init(&g_core, NULL)); comms_init();
    s_ip_ok = true; g_state_hz = 0;
    expect_ack(TC_PING, NULL, 0, ACK_OK);
    uint8_t p[14] = {CC3_WIRE_VERSION, CC_CMD_WHEEL};
    expect_ack(TC_PING, p, 1, ACK_BAD_LEN);
    expect_ack(TC_STREAM_CFG, p, 2, ACK_BAD_LEN);
    uint8_t cfg[] = {0, 7, 3};
    expect_ack(TC_STREAM_CFG, cfg, 3, ACK_OK);
    assert(g_state_hz == 0 && g_vision_hz == 7 && g_img_every == 3);
    cc_write_le_float(p + 2, 600); cc_write_le_float(p + 6, -1200); cc_write_le_float(p + 10, 1800);
    expect_ack(TC_SET_VECTOR3, p, 14, ACK_OK);
    for (int i = 0; i < 3; i++) assert(fabsf(g_core.wheel_sp[i] - (i == 1 ? -1200 : 600 * (i + 1)) * 0.1047197551f) < 0.001f);
    expect_ack(TC_SET_VECTOR3, p, 13, ACK_BAD_LEN);
    p[0] = 2; expect_ack(TC_SET_VECTOR3, p, 14, ACK_BAD_ARG); p[0] = 1;
    cc_write_le_float(p + 6, NAN); expect_ack(TC_SET_VECTOR3, p, 14, ACK_BAD_ARG);
    assert(fabsf(g_core.wheel_sp[1] + 1200 * 0.1047197551f) < 0.001f);
    uint8_t gain[] = {1, 1, CC_GAIN_RATE_KP, 0, 0, 0, 0};
    float old_x = g_core.cfg.axis[0].rate_kp, old_z = g_core.cfg.axis[2].rate_kp;
    cc_write_le_float(gain + 3, 0.004f); expect_ack(TC_SET_GAIN3, gain, 7, ACK_OK);
    assert(g_core.cfg.axis[1].rate_kp == 0.004f && g_core.cfg.axis[0].rate_kp == old_x && g_core.cfg.axis[2].rate_kp == old_z);
    expect_ack(TC_SET_GAIN3, gain, 6, ACK_BAD_LEN);
    gain[1] = 3; expect_ack(TC_SET_GAIN3, gain, 7, ACK_BAD_ARG);
    expect_ack(0xfe, NULL, 0, ACK_UNKNOWN);
    uint8_t legacy[4]; cc_write_le_float(legacy, 300);
    expect_ack(TC_SET_WHEEL, legacy, 4, ACK_OK);
    assert(fabsf(g_core.wheel_sp[2] - 300 * 0.1047197551f) < 0.001f);
    for (int damage = 1; damage <= 3; damage++) {
        s_have_client = false;
        run_datagram(TC_PING, NULL, 0, damage);
        assert(sent_count == 0 && !s_have_client);
    }
    g_state_hz = 20; g_core.tlm.imu_status = 2; g_core.tlm.motor_valid_mask = 5;
    for (int i = 0; i < 3; i++) { g_core.tlm.axis[i].t_ms = 100; g_core.tlm.axis[i].wheel_rps = (float)(i + 1); }
    run_datagram(TC_PING, NULL, 0, 0);
    assert(sent_count == 3 && sent[1].type == TM_STATE && sent[1].len == 52);
    assert(sent[2].type == TM_STATE3 && sent[2].len == 159);
    assert(sent[2].payload[0] == 1 && sent[2].payload[1] == 2 && sent[2].payload[2] == 5);
    assert(!memcmp(sent[1].payload, sent[2].payload + 3 + 104, 52));
    for (int i = 0; i < 3; i++) assert(fabsf(cc_read_le_float(sent[2].payload + 3 + 52*i + 28) - (i + 1) * 9.549296586f) < 0.001f);
    cc_vision_t vision = {0}; comms_send_vision(&vision, 100);
    assert(sent[3].type == TM_VISION);
    on_event(NULL, WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, NULL);
    assert(!s_ip_ok && !s_have_client);
    handle_tc(TC_PING, 42, NULL, 0); assert(sent_count == 4);
    puts("PASS: firmware UDP/CRC/ACK, vector3/gain3, legacy TC/TM, STATE3 XYZ/mask, stream config, invalid datagrams, mutexes and Wi-Fi reset");
    return 0;
}
'''


def main():
    compiler = os.environ.get('CC') or shutil.which('gcc')
    if not compiler and Path('C:/msys64/ucrt64/bin/gcc.exe').is_file():
        compiler = 'C:/msys64/ucrt64/bin/gcc.exe'
    if not compiler:
        raise SystemExit('Host GCC not found. Set CC to its executable.')
    compiler = shutil.which(compiler) or compiler
    env = os.environ.copy()
    env['PATH'] = str(Path(compiler).resolve().parent) + os.pathsep + env.get('PATH', '')
    source = (ROOT / 'firmware/main/comms.c').read_text(encoding='utf-8')
    source = re.sub(r'^#include "(?:comms|app|esp_event|esp_log|esp_netif|esp_timer|esp_wifi|sdkconfig|lwip/sockets)\.h"\s*$', '', source, flags=re.M)
    with tempfile.TemporaryDirectory(prefix='comms_proto3_') as directory:
        directory = Path(directory)
        cfile = directory / 'test.c'
        exe = directory / ('test.exe' if os.name == 'nt' else 'test')
        cfile.write_text(STUBS + '\n' + source + '\n' + TESTS, encoding='utf-8')
        command = [compiler, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                   '-Wno-unused-parameter', '-I' + str(ROOT / 'core/include'),
                   str(cfile), *(str(ROOT / 'core/src' / (name + '.c')) for name in
                   ('cube_core3', 'cc_proto', 'cc_proto3', 'target_detect', 'moteus_proto')),
                   '-lm', '-o', str(exe)]
        if os.name == 'nt':
            command.append('-static-libgcc')
        subprocess.run(command, env=env, check=True)
        subprocess.run([str(exe)], env=env, check=True)


if __name__ == '__main__':
    main()
