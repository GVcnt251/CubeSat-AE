#include "cube_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static cc_t c;
static cc_meas_t m;
static cc_out_t o;
static uint64_t t;
static cc_imu_status_t step(unsigned dt, int valid) {
    t += dt; return cc_step_with_imu(&c,t,&m,valid,&o);
}
static void command(unsigned type, float a) {
    cc_cmd_t cmd={type,0,a,0}; cc_command(&c,&cmd);
}
static void ready(void) {
    cc_config_t cfg; cc_config_default(&cfg); cfg.calib_s=0.1f;
    cc_init(&c,&cfg); t=0; m=(cc_meas_t){0}; m.accel[2]=9.81f;
    for(int i=0;i<25;i++) assert(step(5000,1)==CC_IMU_OK);
    assert(c.calibrated);
}
int main(void) {
    ready(); command(CC_CMD_RECAL,0);
    for(int i=0;i<15;i++) step(5000,1);
    assert(c.gcount==15);
    assert(step(5000,0)==CC_IMU_UNAVAILABLE && c.gcount==0);
    for(int i=0;i<20;i++) step(5000,1);
    assert(!c.calibrated);
    step(5000,1); step(5000,1); assert(c.calibrated);

    ready(); command(CC_CMD_MODE,CC_MODE_RATE); command(CC_CMD_RATE,0.2f);
    step(5000,1);
    float x=c.x[0], integral=c.rate_I; int hist=c.h_count;
    m.gyro[2]=50;
    assert(step(5000,0)==CC_IMU_UNAVAILABLE && !o.enable);
    assert(c.x[0]==x && c.rate_I==integral && c.h_count==hist);
    float mag[3]={44,0,0}; cc_on_mag(&c,t,mag); assert(c.x[0]==x);
    assert(step(45000,1)==CC_IMU_OK); /* exactly 50 ms since last valid */
    assert(c.x[0]==x && !o.enable && c.last_us==t);
    m.gyro[2]=0; assert(step(5000,1)==CC_IMU_OK && o.enable);

    ready(); command(CC_CMD_MODE,CC_MODE_HEADING);
    assert(step(50001,1)==CC_IMU_RECAL_REQUIRED && !o.enable);
    for(int mode=CC_MODE_RATE;mode<=CC_MODE_TRACK;mode++) {
        command(CC_CMD_MODE,(float)mode);
        assert(step(5000,1)==CC_IMU_RECAL_REQUIRED && !o.enable);
    }
    command(CC_CMD_MODE,CC_MODE_WHEEL); command(CC_CMD_WHEEL,100);
    assert(step(5000,0)==CC_IMU_RECAL_REQUIRED && o.enable && o.wheel_cmd_rps==100);
    assert(step(5000,1)==CC_IMU_RECAL_REQUIRED && o.enable);
    command(CC_CMD_RECAL,0);
    assert(c.imu_recal_required);
    for(int i=0;i<15;i++) assert(step(5000,1)==CC_IMU_RECAL_REQUIRED);
    assert(step(5000,0)==CC_IMU_RECAL_REQUIRED && c.gcount==0);
    for(int i=0;i<20;i++) assert(step(5000,1)==CC_IMU_RECAL_REQUIRED);
    assert(step(5000,1)==CC_IMU_RECAL_REQUIRED);
    assert(step(5000,1)==CC_IMU_OK && c.calibrated);
    command(CC_CMD_MODE,CC_MODE_RATE); assert(step(5000,1)==CC_IMU_OK && o.enable);

    ready(); command(CC_CMD_MODE,CC_MODE_WHEEL); command(CC_CMD_WHEEL,1000);
    x=c.x[0]; hist=c.h_count;
    assert(step(5000,0)==CC_IMU_UNAVAILABLE && o.enable);
    assert(o.wheel_cmd_rps==c.cfg.wheel_lim && c.x[0]==x && c.h_count==hist);
    assert(step(5000,1)==CC_IMU_OK && o.enable && c.x[0]==x);
    ready(); c.cfg.imu_timeout_us=10000;
    assert(step(10001,0)==CC_IMU_RECAL_REQUIRED);
    cc_init(&c,NULL); t=0;
    assert(step(5000,0)==CC_IMU_UNAVAILABLE && !o.enable);
    step(5000,1); assert(step(50001,0)==CC_IMU_RECAL_REQUIRED);
    assert(!c.calibrated && c.gcount==0);
    puts("IMU availability, calibration, timeout boundary, latch, recovery and WHEEL: PASS");
    return 0;
}
