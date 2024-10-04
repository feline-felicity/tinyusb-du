#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "bsp/board_api.h"
#include "board.h"



ISR(USB0_BUSEVENT_vect) {
  tud_int_handler(0);
}

ISR(USB0_TRNCOMPL_vect) {
  tud_int_handler(0);
}


#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

ISR(TCB0_INT_vect) {
  system_ticks++;
  TCB0.INTFLAGS = TCB_CAPT_bm;
}


uint32_t board_millis(void) {
  uint32_t ret;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ret = system_ticks;
  }
  return ret;
}
#endif

static int usart0_putc(char c, FILE*) {
    loop_until_bit_is_set(USART0_STATUS, USART_DREIF_bp);
    USART0.TXDATAL = c;
    return 0;
}

void board_init(void) {
  cli();

#if CFG_TUSB_OS == OPT_OS_NONE

  TCB0.CCMP = 24000-1;
  TCB0.INTCTRL = TCB_CAPT_bm;
  TCB0.CTRLA |= TCB_ENABLE_bm;

#endif

  // Main clock: 24 MHz autotuned to SOF
  _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, CLKCTRL_FRQSEL_24M_gc | CLKCTRL_AUTOTUNE_SOF_gc);

  // Enable regulator
  SYSCFG.VUSBCTRL |= SYSCFG_USBVREG_bm;

  #ifdef LED_PIN
  
  LED_PORT.DIRSET = (1 << LED_PIN);

  #endif

  #ifdef BUTTON_PIN

  BUTTON_PORT.DIRCLR = (1 << BUTTON_PIN);
  BUTTON_PORT.PINCONFIG = PORT_PULLUPEN_bm;
  BUTTON_PORT.PINCTRLUPD = (1 << BUTTON_PIN);

  #endif

  // USART0: 115200 bps, 8N1
  USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
  USART0.BAUD = (64 * F_CPU) / (16 * 115200);
  VPORTA.DIR |= PIN0_bm;
  USART0.CTRLB = USART_TXEN_bm /* | USART_RXEN_bm */ | USART_RXMODE_NORMAL_gc;

  // stdout bypassed to USART0
  fdevopen(usart0_putc, NULL);

  sei();

  board_led_write(true);
}

void board_led_write(bool state) {
  if ((!state && !LED_STATE_ON) || (state && LED_STATE_ON)) {
    LED_PORT.OUTSET = (1 << LED_PIN);
  } else {
    LED_PORT.OUTCLR = (1 << LED_PIN);
  }
}

uint32_t board_button_read(void) {
  return BUTTON_STATE_ACTIVE == !!(BUTTON_PORT.IN & (1 << BUTTON_PIN));
}

int board_uart_read(uint8_t *buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const *buf, int len) {
  const uint8_t *bufc = (const uint8_t *) buf;
  for (int i = 0; i < len; i++) {
    loop_until_bit_is_set(USART0_STATUS, USART_DREIF_bp);
    USART0.TXDATAL = bufc[i];
  }
  return len;
}
