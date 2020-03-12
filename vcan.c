#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/can.h>

#include <atom.h>
#include <atomsem.h>
#include <atomqueue.h>
#include <atomtimer.h>
#include <string.h>

#include "hw.h"


#define DEBUG 1

#define WHITE   0xffffff
#define RED     0xff0000
#define GREEN   0x00Ff00
#define BLUE    0x0000fF
#define YELLOW  0xffFf00
#define CYAN    0x00Ff99
#define MAGENTA 0xFf00ff

static uint8_t idle_stack[256];
static uint8_t master_thread_stack[512];
static ATOM_TCB master_thread_tcb;

static ATOM_QUEUE uart1_tx;
static uint8_t uart1_tx_storage[1024];

static ATOM_QUEUE uart1_rx;
static uint8_t uart1_rx_storage[64];

void _fault(int, int, const char*);
int u_write(int file, uint8_t *ptr, int len);
int s_write(int file, char *ptr, int len);
void xcout(unsigned char c);
void x2cout(uint16_t c);
void to_hex4(uint8_t c,unsigned char* b);
void to_hex8(uint8_t c,unsigned char* b);
void to_hex16(uint16_t c,unsigned char* b);

#define fault(code) _fault(code,__LINE__,__FUNCTION__)
void _fault(__unused int code, __unused int line, __unused const char* function){
    cm_mask_interrupts(true);
    while(1){
    }
};


static char set[]="0123456789ABCDEF";
void xcout(unsigned char c){
    char s[2];
    s[0]=set[(c>>4)&0x0f];
    s[1]=set[c&0x0f];
    s_write(1,s,2);
}

void to_hex4(uint8_t c,unsigned char* b){
    b[0]=set[c&0x0f];
}

void to_hex8(uint8_t c,unsigned char* b){
    b[0]=set[(c>>4)&0x0f];
    b[1]=set[c&0x0f];
}

void to_hex16(uint16_t c,unsigned char* b){
    b[0]=set[(c>>12)&0x0f];
    b[1]=set[(c>>8)&0x0f];
    b[2]=set[(c>>4)&0x0f];
    b[3]=set[c&0x0f];
}

void x2cout(uint16_t c){
    char s[4];
    s[0]=set[(c>>12)&0x0f];
    s[1]=set[(c>>8)&0x0f];
    s[2]=set[(c>>4)&0x0f];
    s[3]=set[c&0x0f];
    s_write(1,s,4);
}

void usart1_isr(void) {
    static uint8_t data = 'A';
    atomIntEnter();

    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {
        data = usart_recv(USART1);
        atomQueuePut(&uart1_rx, -1, (uint8_t*) &data);
    }

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
            ((USART_SR(USART1) & USART_SR_TXE) != 0)) {
        uint8_t status = atomQueueGet(&uart1_tx, 0, &data);
        if(status == ATOM_OK){
            usart_send_blocking(USART1, data);
        }else{
            USART_CR1(USART1) &= ~USART_CR1_TXEIE;
        }
    }
    atomIntExit(0);
}


static int8_t scan(uint8_t c) {
  if(c>='0' && c<='9') return c-'0';
  if(c>='a' && c<='f') return c-'a'+10;
  if(c>='A' && c<='F') return c-'A'+10;
  return -1;
}


static int handle_uart_cmd(uint8_t* cmd, uint8_t len){
  bool ext=true;
  if(cmd[0]=='V' && len==1) return s_write(1,"V1013\r",6);
  if(cmd[0]=='N' && len==1) return s_write(1,"N1234\r",6);
  if(cmd[0]=='S' && len==2) return -1;
  if(cmd[0]=='s' && len==5) return -1;
  if(cmd[0]=='O' && len==1) return s_write(1,"\r",1);
  if(cmd[0]=='C' && len==1) return s_write(1,"\r",1);
  if(cmd[0]=='T' || cmd[0]=='t'){
    uint32_t address=0;
    uint8_t dlen;
    ext=(cmd[0]=='T');
    uint8_t offset=4;
    uint8_t data[8];
    if(!ext){
      address|=(scan(cmd[1])<<8);
      address|=(scan(cmd[2])<<4);
      address|=(scan(cmd[3]));
    }else{
      address|=(scan(cmd[1])<<28);
      address|=(scan(cmd[2])<<24);
      address|=(scan(cmd[3])<<20);
      address|=(scan(cmd[4])<<16);
      address|=(scan(cmd[5])<<12);
      address|=(scan(cmd[6])<<8);
      address|=(scan(cmd[7])<<4);
      address|=(scan(cmd[8]));
      offset=9;
    }
    dlen=scan(cmd[offset++]);
    int i=0;
    for(;i<dlen;i++){
      data[i]=(scan(cmd[offset])<<4)|(scan(cmd[offset+1]));
      offset+=2;
    }
    if ((can_transmit(CAN1,
            address,     /* (EX/ST)ID: CAN ID */
            ext, /* IDE: CAN ID extended? */
            false, /* RTR: Request transmit? */
            dlen,     /* DLC: Data length */
            data)) == -1){
      return -1;
    }else{
      return s_write(1,"\r",1);
    }
  };
  if(cmd[0]=='r') return -1;
  if(cmd[0]=='R') return -1;

  if(cmd[0]=='F') return -1;
  if(cmd[0]=='M') return -1;
  if(cmd[0]=='m') return -1;
  if(cmd[0]=='Z' && len==1) return -1;
  return -1;
}

static void master_thread(uint32_t args __maybe_unused) {
  s_write(1,"MASTER\r\n",8);
  uint8_t recv[32];
  uint8_t rlen=0;
  while(1){
    uint8_t status = atomQueueGet(&uart1_rx, 0, (void*)&recv[rlen]);
    if(status == ATOM_OK){
      if(recv[rlen]=='\n' || recv[rlen]=='\r' || rlen==31){
        if(rlen){
          if(handle_uart_cmd(recv,rlen)==-1)
            s_write(1,"\7",1);
        }
        s_write(1,"\r\n",2);
        rlen=0;
      }else
        rlen++;
    }
    gpio_toggle(GPIOC, GPIO13);
    //atomTimerDelay();
  }
}

static void can_setup(void) {
  /* Enable peripheral clocks. */
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_CAN);
  AFIO_MAPR |= AFIO_MAPR_CAN1_REMAP_PORTB;
  //  AFIO_MAPR &= ~AFIO_MAPR_CAN1_REMAP_PORTB;

  /* Configure CAN pin: RX (input pull-up). */
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN_PB_RX);
  gpio_set(GPIOB, GPIO_CAN_PB_RX);

  /* Configure CAN pin: TX. */
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN_PB_TX);

  /* NVIC setup. */
  nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
  nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1);

  /* Reset CAN. */
  can_reset(CAN1);

  /* CAN cell init. */
  if (can_init(CAN1,
        false,           /* TTCM: Time triggered comm mode? */
        true,            /* ABOM: Automatic bus-off management? */
        false,           /* AWUM: Automatic wakeup mode? */
        false,           /* NART: No automatic retransmission? */
        false,           /* RFLM: Receive FIFO locked mode? */
        false,           /* TXFP: Transmit FIFO priority? */
        CAN_BTR_SJW_1TQ,
        CAN_BTR_TS1_4TQ,
        CAN_BTR_TS2_3TQ,
        6,/* BRP+1: Baud rate prescaler */
        false,
        false))
  {
    gpio_set(GPIOC, GPIO13);		/* LED0*/
    /* Die because we failed to initialize. */
    while (1)
      __asm__("nop");
  }

  /* CAN filter 0 init. */
  can_filter_id_mask_32bit_init(
      CAN1,
      0,     /* Filter ID */
      0,     /* CAN ID */
      0,     /* CAN ID mask */
      0,     /* FIFO assignment (here: FIFO0) */
      true); /* Enable the filter. */

  /* Enable CAN RX interrupt. */
  can_enable_irq(CAN1, CAN_IER_FMPIE0);
}

void usb_lp_can_rx0_isr(void) {
  atomIntEnter();
  uint32_t id, fmi;
  bool ext, rtr;
  uint8_t length, data[8];
  uint8_t sb[28];
  uint8_t *ptr;
  int i;

  can_receive(CAN1, 0, false, &id, &ext, &rtr, &fmi, &length, data);

  if(ext){
    to_hex16(0xffff&(id >> 16),sb+1);
    to_hex16(0xffff&(id),sb+5);
    if(rtr)
      sb[0]='R';
    else
      sb[0]='T';
    ptr=sb+9;
  }else{
    to_hex16(id,sb);
    if(rtr)
      sb[0]='r';
    else
      sb[0]='t';
    ptr=sb+4;
  }

  to_hex4(length,ptr);
  ptr++;

  for(i=0;i<length;i++,ptr+=2){
    to_hex8(data[i],ptr);
  }
  *ptr='\r';
  if(rtr)
    s_write(1,(char*)sb,(ext?9:4)+1);
  else
    s_write(1,(char*)sb,(ext?9:4)+2+(length<<1));

  can_fifo_release(CAN1, 0);
  atomIntExit(0);
}

int main(void) {
  //rcc_clock_setup_in_hsi_out_48mhz();
  rcc_clock_setup_in_hse_8mhz_out_24mhz();
  //rcc_clock_setup_in_hsi_out_64mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_USART1);

  AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

  usart_setup();
  can_setup();

  cm_mask_interrupts(true);
  systick_set_frequency(SYSTEM_TICKS_PER_SEC, 24000000);
  systick_interrupt_enable();
  systick_counter_enable();

  nvic_set_priority(NVIC_PENDSV_IRQ, 0xFF);
  nvic_set_priority(NVIC_SYSTICK_IRQ, 0xFE);

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 );

  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      GPIO12 | GPIO13 | GPIO14 );

  //Buttons
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, 
      GPIO3|GPIO4);
  gpio_set(GPIOB, GPIO3|GPIO4);


  //LED
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
  gpio_clear(GPIOC, GPIO13);

  /* extra gpo/NSS B3...B6
     gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
     GPIO3|GPIO3|GPIO5|GPIO6);
     gpio_set(GPIOB, GPIO3|GPIO4|GPIO5|GPIO6);

     inputs (GPI) A0...A3 
     gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, 
     GPIO0|GPIO1|GPIO2|GPIO3);
     gpio_clear(GPIOC, GPIO0|GPIO1|GPIO2|GPIO3);
     */

#if DEBUG
  usart_send_blocking(USART1, '\r');
  usart_send_blocking(USART1, '\n');
  usart_send_blocking(USART1, 'p');
  usart_send_blocking(USART1, 'r');
  usart_send_blocking(USART1, 'e');
  usart_send_blocking(USART1, 'v');
  usart_send_blocking(USART1, 'e');
  usart_send_blocking(USART1, 'd');
  usart_send_blocking(USART1, '\r');
  usart_send_blocking(USART1, '\n');
#endif

  gpio_set(GPIOA, GPIO15);
  if(atomOSInit(idle_stack, sizeof(idle_stack), FALSE) != ATOM_OK) 
    fault(1);
  if (atomQueueCreate (&uart1_rx, uart1_rx_storage, sizeof(uint8_t), 
        sizeof(uart1_rx_storage)) != ATOM_OK) 
    fault(2);
  if (atomQueueCreate (&uart1_tx, uart1_tx_storage, sizeof(uint8_t), 
        sizeof(uart1_tx_storage)) != ATOM_OK) 
    fault(3);

  atomThreadCreate(&master_thread_tcb, 10, master_thread, 0,
      master_thread_stack, sizeof(master_thread_stack), TRUE);

  atomOSStart();
  while (1){
  }
}

int s_write(int file, char *ptr, int len) {
    return u_write(file, (uint8_t *)ptr,len);
};

#if DEBUG
int u_write(int file, uint8_t *ptr, int len) {
    int i;
    for (i = 0; i < len; i++){
        switch(file){
            case 1: //DEBUG
                atomQueuePut(&uart1_tx,-1, (uint8_t*) &ptr[i]);
                break;
        }
    }
    switch(file){
        case 1:
            USART_CR1(USART1) |= USART_CR1_TXEIE;
            break;
    }
    return i;
}
#else
int u_write(int file, uint8_t *ptr, int len) {
  (void)file;
  (void)ptr;
  (void)len;
  return 0;
}
#endif

