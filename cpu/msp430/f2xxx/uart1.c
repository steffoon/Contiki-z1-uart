/*
 * Copyright (c) 2010, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/*
 * Machine dependent MSP430X UART1 code.
 */
#include "contiki.h"
#include <stdlib.h>
#include "sys/energest.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"
#include "lib/ringbuf.h"
#include "isr_compat.h"

static int (*uart1_input_handler)(unsigned char c);

static volatile uint8_t transmitting;

#ifdef UART1_CONF_TX_WITH_INTERRUPT
#define TX_WITH_INTERRUPT UART1_CONF_TX_WITH_INTERRUPT
#else /* UART1_CONF_TX_WITH_INTERRUPT */
#define TX_WITH_INTERRUPT 1
#endif /* UART1_CONF_TX_WITH_INTERRUPT */

#if TX_WITH_INTERRUPT
#define TXBUFSIZE 64

static struct ringbuf txbuf;
static uint8_t txbuf_data[TXBUFSIZE];
#endif /* TX_WITH_INTERRUPT */

/*---------------------------------------------------------------------------*/
uint8_t
uart1_active(void)
{
  return (UCA1STAT & UCBUSY) | transmitting;
}
/*---------------------------------------------------------------------------*/
void
uart1_set_input(int (*input)(unsigned char c))
{
  uart1_input_handler = input;
}
/*---------------------------------------------------------------------------*/
void
uart1_writeb(unsigned char c)
{
  /* watchdog_periodic(); */
#if TX_WITH_INTERRUPT

  /* Put the outgoing byte on the transmission buffer. If the buffer
     is full, we just keep on trying to put the byte into the buffer
     until it is possible to put it there. */
  while(ringbuf_put(&txbuf, c) == 0);

  /* If there is no transmission going, we need to start it by putting
     the first byte into the UART. */
  if(transmitting == 0) {
    transmitting = 1;
    UCA1TXBUF = ringbuf_get(&txbuf);
  }

#else /* TX_WITH_INTERRUPT */

  /* Loop until the transmission buffer is available. */
  while(!(UC1IFG & UCA1TXIFG));

  /* Transmit the data. */
  UCA1TXBUF = c;
#endif /* TX_WITH_INTERRUPT */
}
/*---------------------------------------------------------------------------*/
#if ! NETSTACK_CONF_WITH_IPV4 /* If NETSTACK_CONF_WITH_IPV4 is defined, putchar() is defined by the SLIP driver */
#endif /* ! NETSTACK_CONF_WITH_IPV4 */
/*---------------------------------------------------------------------------*/
/**
 * Initalize the RS232 port.
 *
 */
void
uart1_init(unsigned long ubr)
{
  /* RS232 */
  UCA1CTL1 |= UCSWRST;            /* Hold peripheral in reset state */
  UCA1CTL1 |= UCSSEL_2;           /* CLK = SMCLK */

  UCA1BR0 = 0x45; /* 8MHz/115200 = 69 = 0x45 */
  UCA1BR1 = 0x00;
  UCA1MCTL = UCBRS_3;             /* Modulation UCBRSx = 3 */

  P3DIR &= ~0x80;                 /* P3.7 = USCI_A1 RXD as input */
  P3DIR |= 0x40;                  /* P3.6 = USCI_A1 TXD as output */
  P3SEL |= 0xC0;                  /* P3.6,7 = USCI_A1 TXD/RXD */
  /*UCA0CTL1 &= ~UCSWRST;*/       /* Initialize USCI state machine */

  transmitting = 0;

  /* XXX Clear pending interrupts before enable */
  UC1IFG &= ~UCA1RXIFG;
  UC1IFG &= ~UCA1TXIFG;
  UCA1CTL1 &= ~UCSWRST;                   /* Initialize USCI state machine **before** enabling interrupts */
  UC1IE |= UCA1RXIE;                        /* Enable UCA0 RX interrupt */
  /* Enable USCI_A1 TX interrupts (if TX_WITH_INTERRUPT enabled) */
#if TX_WITH_INTERRUPT
  ringbuf_init(&txbuf, txbuf_data, sizeof(txbuf_data));
  UC1IE |= UCA1TXIE;                        /* Enable UCA1 TX interrupt */
#endif /* TX_WITH_INTERRUPT */
 
}
/*---------------------------------------------------------------------------*/
ISR(USCIAB1RX, uart1_rx_interrupt)
{
  uint8_t c;
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  /* Check status register for receive errors. */
  if(UCA1STAT & UCRXERR) {
    c = UCA1RXBUF;   /* Clear error flags by forcing a dummy read. */
  } else {
    c = UCA1RXBUF;
    if(uart1_input_handler != NULL) {
      if(uart1_input_handler(c)) {
	LPM4_EXIT;
      }
    }
  }
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
#if TX_WITH_INTERRUPT
ISR(USCIAB1TX, uart1_tx_interrupt)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  if(UC1IFG & UCA1TXIFG) {
    if(ringbuf_elements(&txbuf) == 0) {
      transmitting = 0;
    } else {
      UCA1TXBUF = ringbuf_get(&txbuf);
    }
  }
  /* In a stand-alone app won't work without this. Is the UG misleading? */
  UC1IFG &= ~UCA0TXIFG;

  
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
#endif /* TX_WITH_INTERRUPT */
/*---------------------------------------------------------------------------*/
