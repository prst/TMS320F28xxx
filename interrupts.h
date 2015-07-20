/* ************************************************************************** */
/* ************************************************************************** */
#ifndef __INTERRUPTS_H__
#define __INTERRUPTS_H__

/* ************************************************************************** */


interrupt void  adc_isr(void);
interrupt void  sciaTxFifoIsr (void);
interrupt void  sciaRxFifoIsr (void);
__interrupt void  cpu_timer0_isr(void);

interrupt void  epwm1_timer_isr (void);
interrupt void  epwm2_timer_isr (void);
interrupt void  epwm3_timer_isr (void);
//interrupt void  timer0_isr      (void);



/* ************************************************************************** */

#endif //__INTERRUPTS_H__
/* ************************************************************************** */
