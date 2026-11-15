"""Testa as funcoes C reais do driver com SPI MCP2518FD simulado (sem hardware).
Colocar em CubeSat-AE/sim/test_can_driver_mock.py e executar na pasta firmware:
  python ..\\sim\\test_can_driver_mock.py
Requer gcc do MSYS2 UCRT64 (ou no PATH).
"""
from pathlib import Path
from tempfile import TemporaryDirectory
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / 'firmware' / 'main' / 'can_mcp2518fd.c'

STUBS = r'''
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NO_MEM 0x105
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_NOT_SUPPORTED 0x106
static const char *esp_err_to_name(int e) { (void)e; return "error"; }
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define SPI2_HOST 2
#define SPI_DMA_CH_AUTO 0
#define PIN_SPI_MISO 8
#define PIN_SPI_MOSI 9
#define PIN_SPI_SCK 7
#define PIN_CAN_CS 3
#define PIN_CAN_INT 4
#define GPIO_MODE_INPUT 1
#define GPIO_PULLUP_ENABLE 1
#define pdTRUE 1
#define pdMS_TO_TICKS(v) (v)
#define portMAX_DELAY 999
#define pdFALSE 0
typedef struct { uint32_t id; uint8_t len; uint8_t data[64]; } can_frame_t;
typedef void * spi_device_handle_t;
typedef struct {
 int miso_io_num,mosi_io_num,sclk_io_num,quadwp_io_num,quadhd_io_num,max_transfer_sz;
} spi_bus_config_t;
typedef struct {int clock_speed_hz, mode, spics_io_num, queue_size;} spi_device_interface_config_t;
typedef struct {size_t length; const void *tx_buffer; void *rx_buffer;} spi_transaction_t;
typedef struct {uint64_t pin_bit_mask; int mode,pull_up_en;} gpio_config_t;
typedef void * SemaphoreHandle_t;
static int mock_lock;
static uint8_t mem[0x1000];
static uint32_t last_tx_id;
static uint8_t last_tx_len;
static uint8_t last_tx_data[64];
static unsigned tx_requests;
static unsigned rx_pops;
static bool rx_pending;
static bool rx_corrupt_data;
static uint32_t fail_read_addr;
static int tests, failures;
#define ASSERT(cond,desc) do { tests++; if (!(cond)) { failures++; printf("FALHOU: %s (linha %d)\n",desc,__LINE__); } else {printf("PASSOU: %s\n",desc);} } while(0)
static uint32_t get32(unsigned a) {return (uint32_t)mem[a] | ((uint32_t)mem[a+1]<<8) | ((uint32_t)mem[a+2]<<16) | ((uint32_t)mem[a+3]<<24);}
static void put32(unsigned a,uint32_t v) {for(int k=0;k<4;k++) mem[a+k]=(uint8_t)(v>>(k*8));}
static void mock_hw_reset(void) {
 memset(mem,0,sizeof(mem));
 put32(0x000,4u<<21); /* CiCON OPMOD configuration */
 put32(0x060,1); /* TX FIFO has space */
 put32(0x064,0); /* TX FIFO pointer offset 0 */
 put32(0x06C,0); /* RX initially empty */
 put32(0x070,128); /* RX FIFO offset */
 rx_pending=false;
}
static esp_err_t spi_bus_initialize(int host,const spi_bus_config_t *b,int dma) {(void)host;(void)b;(void)dma;return ESP_OK;}
static esp_err_t spi_bus_add_device(int host,const spi_device_interface_config_t *dev,spi_device_handle_t *out) {(void)host;(void)dev;*out=(void*)1;return ESP_OK;}
static esp_err_t spi_bus_remove_device(spi_device_handle_t dev) {(void)dev;return ESP_OK;}
static esp_err_t spi_device_polling_transmit(spi_device_handle_t dev,spi_transaction_t *t) {
 (void)dev;
 const uint8_t *tx=(const uint8_t*)t->tx_buffer;
 size_t n=t->length/8;
 if(n==2 && tx[0]==0 && tx[1]==0) {mock_hw_reset();return ESP_OK;}
 if(n<2) return ESP_FAIL;
 unsigned cmd=((unsigned)tx[0]<<8)|tx[1];
 unsigned a=cmd&0x0fff;
 bool rd=(cmd&0x3000)==0x3000;
 bool wr=(cmd&0x3000)==0x2000;
 if(a+n-2>sizeof(mem)) return ESP_FAIL;
 if(rd) {
   if(fail_read_addr && a==fail_read_addr) return ESP_FAIL;
   uint8_t *rx=(uint8_t*)t->rx_buffer;
   if(!rx)return ESP_FAIL;
   memset(rx,0,n);
   memcpy(rx+2,mem+a,n-2);
   if(rx_corrupt_data && a==0x488) return ESP_FAIL;
 } else if(wr) {
   memcpy(mem+a,tx+2,n-2);
   if(a==0x05d && n==3 && tx[2]==3) {
     tx_requests++;
     last_tx_id=get32(0x400)&0x7ff;
     uint8_t dlc=mem[0x404]&0x0f;
     const uint8_t lens[]={0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
     last_tx_len=lens[dlc];
     memcpy(last_tx_data,mem+0x408,last_tx_len);
   }
   if(a==0x069 && n==3 && tx[2]==1) {rx_pops++;put32(0x06c,0);rx_pending=false;}
   if(a==0x000 && n==6) { /* software emulates OPMOD mirroring REQOP */
     uint32_t v=get32(0);v=(v&~(7u<<21))|(((v>>24)&7u)<<21);put32(0,v);
   }
 } else return ESP_FAIL;
 return ESP_OK;
}
static esp_err_t gpio_config(const gpio_config_t *g){(void)g;return ESP_OK;}
static void vTaskDelay(int t){(void)t;}
static SemaphoreHandle_t xSemaphoreCreateMutex(void){return &mock_lock;}
static int xSemaphoreTake(SemaphoreHandle_t l,int ticks){(void)l;(void)ticks;return pdTRUE;}
static void xSemaphoreGive(SemaphoreHandle_t l){(void)l;}
static void vSemaphoreDelete(SemaphoreHandle_t l){(void)l;}
'''

TESTS = r'''
static void supply_rx(uint32_t id,const uint8_t *data,uint8_t n) {
  memset(mem+0x480,0,72);
  put32(0x480,id);
  int dlc=0;
  const uint8_t lengths[]={0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
  for(int i=0;i<16;i++) if(lengths[i]==n) dlc=i;
  mem[0x484]=(uint8_t)(0xc0 | dlc);
  memcpy(mem+0x488,data,n);
  put32(0x06c,1);
  rx_pending=true;
}
int main(void) {
  ASSERT(can_port_send(NULL)==ESP_ERR_INVALID_ARG,"TX: rejeita ponteiro NULL");
  can_frame_t bad={.id=0x800,.len=1};
  ASSERT(can_port_send(&bad)==ESP_ERR_INVALID_ARG,"TX: rejeita ID fora dos 11 bits");
  bad.id=1;bad.len=65;
  ASSERT(can_port_send(&bad)==ESP_ERR_INVALID_ARG,"TX: rejeita 65 bytes");
  ASSERT(can_port_init()==ESP_OK,"INIT: controlador simulado inicializa");
  for(int j=0;j<3;j++) {
    can_frame_t f={.id=(uint32_t)(0x21+j),.len=48};
    for(int i=0;i<48;i++) f.data[i]=(uint8_t)(i+j);
    int prev=tx_requests;
    esp_err_t e=can_port_send(&f);
    char s[110];
    snprintf(s,sizeof s,"TX: chamada envia ID distinto %d",j+1);
    ASSERT(e==ESP_OK && tx_requests==prev+1 && last_tx_id==f.id &&
           last_tx_len==48 && memcmp(last_tx_data,f.data,48)==0,s);
  }
  can_frame_t result={0};
  ASSERT(!can_port_recv(&result),"RX: sem mensagem devolve false");
  for(int j=0;j<3;j++) {
    uint8_t pl[12];for(int k=0;k<12;k++)pl[k]=(uint8_t)(j+10+k);
    supply_rx((uint32_t)(0x21+j),pl,12);
    char s[110];snprintf(s,sizeof s,"RX: devolve ID/comprimento/dados roda %d",j+1);
    ASSERT(can_port_recv(&result) && result.id==(uint32_t)(0x21+j) && result.len==12 &&
      memcmp(result.data,pl,12)==0,s);
    ASSERT(!can_port_recv(&result),"RX: consumiu trama e nao a repete");
  }
  uint8_t data[12]={1,2,3};supply_rx(0x33,data,12);
  fail_read_addr=0x488;
  ASSERT(!can_port_recv(&result),"RX: falha na leitura de payload nao devolve true");
  fail_read_addr=0;
  printf("\n%d verificacoes; %d falhas\n",tests,failures);
  return failures?1:0;
}
'''

def main():
    if not SOURCE.is_file():
        sys.exit(f'Nao encontrei o driver: {SOURCE}')
    gcc = shutil.which('gcc')
    if not gcc:
        p = Path(r'C:\msys64\ucrt64\bin\gcc.exe')
        if p.exists():
            gcc = str(p)
    if not gcc:
        sys.exit('GCC nao encontrado. Instala/ativa MSYS2 UCRT64 e verifica gcc --version.')
    src = SOURCE.read_text(encoding='utf-8')
    # Replace only the platform includes in a temporary copy. The actual driver code is preserved.
    import re
    src = re.sub(r'^\s*#include\s+[<"](?:can_port\.h|board\.h|esp_log\.h|driver/spi_master\.h|driver/gpio\.h|freertos/FreeRTOS\.h|freertos/task\.h|freertos/semphr\.h)[>"]\s*$', '', src, flags=re.M)
    with TemporaryDirectory(prefix='can_mock_') as d:
        d = Path(d)
        cfile = d/'test.c'
        exe = d/('test.exe' if sys.platform=='win32' else 'test')
        cfile.write_text(STUBS + '\n' + src + '\n' + TESTS,encoding='utf-8')
        print('A compilar as funcoes C reais com SPI simulado...')
        cmd=[gcc,'-std=c11','-O0','-Wall','-Wextra','-o',str(exe),str(cfile)]
        cp=subprocess.run(cmd,text=True,capture_output=True)
        if cp.returncode:
            print(cp.stdout,cp.stderr)
            sys.exit(cp.returncode)
        r=subprocess.run([str(exe)],text=True,capture_output=True)
        print(r.stdout,r.stderr,sep='')
        sys.exit(r.returncode)

if __name__=='__main__':
    main()
