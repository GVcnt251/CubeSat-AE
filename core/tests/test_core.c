#include "cc_proto.h"
#include "moteus_proto.h"
#include "wire_le.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static cc_t core;
static uint8_t img[CC_IMG_N];
static cc_det_ws_t ws;

static void protocol(void) {
    uint8_t buf[256], pl[240] = {0}; cc_cmd_t cmd;
    const uint8_t ping[] = {0xa5,0x5a,0,0x80,1,0x25,0xc7};
    assert(cc_crc16((const uint8_t *)"123456789",9)==0x29b1);
    assert(cc_frame_pack(buf,TC_PING,1,NULL,0)==sizeof(ping));
    assert(memcmp(buf,ping,sizeof(ping))==0);
    memset(buf,0xaa,sizeof(buf));
    assert(cc_frame_pack(buf,0,0,pl,241)==0 && buf[0]==0xaa);
    assert(cc_frame_pack(buf,0,0,NULL,1)==0);
    for (int n=0;n<=240;n++) {
        cc_parser_t p; cc_parser_init(&p);
        for (int i=0;i<n;i++) pl[i]=(uint8_t)(i*37+n);
        size_t size=cc_frame_pack(buf,0x42,255,pl,(uint8_t)n);
        int found=0;
        for (size_t i=0;i<size;i++) found+=cc_parser_feed(&p,buf[i]);
        assert(found==1 && p.len==n && memcmp(p.payload,pl,(size_t)n)==0);
        buf[size-1]^=1;
        for (size_t i=0;i<size;i++) assert(!cc_parser_feed(&p,buf[i]));
        assert(p.n_crc_err==1);
        for (size_t i=0;i<sizeof(ping);i++) found+=cc_parser_feed(&p,ping[i]);
        assert(found==2);
    }
    assert(cc_tc_to_cmd(TC_RECAL,pl,1,&cmd)==-ACK_BAD_LEN);
    assert(cc_tc_to_cmd(TC_PING,pl,1,&cmd)==-ACK_BAD_LEN);
    assert(cc_tc_to_cmd(TC_STREAM_CFG,pl,2,&cmd)==-ACK_BAD_LEN);
    float invalid[]={NAN,INFINITY,-INFINITY};
    for (unsigned i=0;i<3;i++) {
        cc_write_le_float(pl,invalid[i]);
        for (int tc=TC_SET_WHEEL;tc<=TC_SET_AIM;tc++)
            assert(cc_tc_to_cmd((uint8_t)tc,pl,4,&cmd)==-ACK_BAD_ARG);
    }
    pl[0]=CC_GAIN_R_CAM; cc_write_le_float(pl+1,0);
    assert(cc_tc_to_cmd(TC_SET_GAIN,pl,5,&cmd)==-ACK_BAD_ARG);
    pl[0]=CC_GAIN_USE_MAG; cc_write_le_float(pl+1,0.5f);
    assert(cc_tc_to_cmd(TC_SET_GAIN,pl,5,&cmd)==-ACK_BAD_ARG);
    pl[0]=CC_GAIN_RATE_KI; cc_write_le_float(pl+1,0);
    assert(cc_tc_to_cmd(TC_SET_GAIN,pl,5,&cmd)==1);
    memset(img,173,sizeof(img));
    for (int chunk=0;chunk<24;chunk++) {
        assert(cc_pack_tm_img_chunk(buf,img,17,(uint8_t)chunk)==205);
        assert(buf[0]==17 && buf[1]==chunk && buf[2]==24 && buf[3]==80 && buf[4]==60);
        for (int i=5;i<205;i++) assert(buf[i]==173);
    }
    assert(cc_pack_tm_img_chunk(buf,img,17,24)==0);
    puts("OK: TMTC, CRC, lengths, validation and image chunks");
}

static void control(void) {
    cc_init(&core,NULL);
    cc_meas_t m={0}; m.accel[2]=9.81f; cc_out_t out;
    for (uint64_t t=0;t<=1600000;t+=5000) cc_step(&core,t,&m,&out);
    assert(core.calibrated && core.mode==CC_MODE_IDLE);
    cc_cmd_t cmd={CC_CMD_GAIN,CC_GAIN_RATE_KI,0,0}; cc_command(&core,&cmd);
    cc_step(&core,1605000,&m,&out);
    assert(core.rate_I==0 && isfinite(out.wheel_cmd_rps));
    cmd.type=CC_CMD_MODE; cmd.a=CC_MODE_RATE; cc_command(&core,&cmd);
    cmd.type=CC_CMD_RATE; cmd.a=0.2f; cc_command(&core,&cmd);
    cc_step(&core,1610000,&m,&out);
    assert(out.enable && isfinite(out.wheel_cmd_rps) && out.wheel_cmd_rps<0);
    float x=core.x[0];
    m.gyro[2]=NAN; cc_step(&core,1615000,&m,&out);
    assert(!out.enable && core.x[0]==x);
    m.gyro[2]=0; cc_step(&core,1600000,&m,&out);
    assert(!out.enable && core.last_us==1610000);
    cmd.type=CC_CMD_MODE; cmd.a=1e30f; cc_command(&core,&cmd);
    assert(core.mode==CC_MODE_RATE);
    cmd.type=CC_CMD_HEADING; cmd.a=1e30f; cc_command(&core,&cmd);
    cmd.type=CC_CMD_MODE; cmd.a=CC_MODE_HEADING; cc_command(&core,&cmd);
    cc_step(&core,1620000,&m,&out);
    assert(out.enable && isfinite(out.wheel_cmd_rps));
    float mag[]={NAN,0,0}; cc_on_mag(&core,1620000,mag);
    assert(!core.mag_ok && isfinite(core.x[0]));
    cc_det_t d={0}; cc_vision_t v;
    d.found=1; d.x=159.5f; d.y=119.5f; d.area=100; d.contrast=180;
    assert(!cc_on_detection(&core,1620001,1620000,&d,&v));
    assert(!cc_on_detection(&core,1,1620000,&d,&v));
    assert(cc_on_detection(&core,1620000,1620000,&d,&v));
    d.x=NAN; assert(!cc_on_detection(&core,1620000,1620000,&d,&v));
    cmd.type=CC_CMD_RECAL; cc_command(&core,&cmd);
    assert(!core.target_valid && !core.h_count && !core.entering_track && !core.calibrated);
    assert(core.cfg.rate_ki==0 && core.mode==CC_MODE_CALIB);
    puts("OK: calibration, zero Ki, invalid samples, timestamps and recalibration");
}

static void detector(void) {
    cc_det_cfg_t cfg; cc_det_cfg_default(&cfg); cc_det_t d;
    memset(img,220,sizeof(img));
    cc_detect(&cfg,&ws,img,&d); assert(!d.found);
    for(int y=0;y<CC_IMG_H;y++) for(int x=0;x<CC_IMG_W;x++)
        if((x-160)*(x-160)+(y-120)*(y-120)<=100) img[y*CC_IMG_W+x]=10;
    cc_detect(&cfg,&ws,img,&d);
    assert(d.found && fabsf(d.x-160)<0.1f && fabsf(d.y-120)<0.1f);
    memset(img,0,sizeof(img)); cc_detect(&cfg,&ws,img,&d); assert(!d.found);
    cfg.dark_ratio=NAN; cc_detect(&cfg,&ws,img,&d); assert(!d.found);
    cc_det_cfg_default(&cfg); cfg.min_area=1;
    memset(img,220,sizeof(img));
    for(int y=10;y<230;y+=10) for(int x=10;x<310;x+=10) img[y*CC_IMG_W+x]=0;
    cc_detect(&cfg,&ws,img,&d); assert(d.n_cand==255);
    /* A large component exhausts the bounded DFS stack: reject the frame. */
    cc_det_cfg_default(&cfg); memset(img,220,sizeof(img));
    for(int y=20;y<200;y++) for(int x=20;x<300;x++) img[y*CC_IMG_W+x]=0;
    cc_detect(&cfg,&ws,img,&d); assert(!d.found);
    puts("OK: detector centroid, blank/dark frames and candidate saturation");
}

static void motor(void) {
    moteus_status_t s; uint8_t b[64];
    const uint8_t reply[]={0x24,4,0,10,0,0x50,0,0,1,0x70,0xff,0x23,0x0d,0x18,0x14,0};
    assert(!moteus_parse_reply(reply,sizeof(reply),&s));
    assert(s.have_mode && s.have_motion && s.have_aux && s.mode==10);
    assert(fabsf(s.velocity_rev_s-0.064f)<1e-6f && s.voltage_v==12);
    assert(moteus_parse_reply(reply,sizeof(reply)-1,&s)<0);
    assert(!s.have_mode && !s.have_motion && !s.have_aux);
    uint8_t partial[]={0x21,1,5};
    assert(!moteus_parse_reply(partial,sizeof(partial),&s) && !s.have_motion);
    uint8_t overflow[]={0x20,0x80,0x80,0x80,0x80,0x10,0};
    assert(moteus_parse_reply(overflow,sizeof(overflow),&s)<0);
    uint8_t badmode[]={0x2d,0,0,0,0,0}; cc_write_le_float(badmode+2,NAN);
    assert(moteus_parse_reply(badmode,sizeof(badmode),&s)<0);
    assert(moteus_pack_velocity(b,50,0.2f,127,1)==48);
    assert(isnan(cc_read_le_float(b+6)) && cc_read_le_float(b+10)==50 && b[47]==0x50);
    assert(moteus_can_id(0,1,1)==0x8001);
    uint32_t rng=1;
    for(int trial=0;trial<10000;trial++) {
        for(int j=0;j<64;j++) { rng=rng*1664525u+1013904223u; b[j]=(uint8_t)(rng>>24); }
        (void)moteus_parse_reply(b,(size_t)(trial%65),&s);
    }
    puts("OK: moteus reference, partial/truncated frames, varuint and malformed inputs");
}
int main(void) { protocol(); control(); detector(); motor(); puts("All core regression tests passed."); return 0; }
