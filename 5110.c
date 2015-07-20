/* ************************************************************************** */
//#include"f2802x_headers/include/F2802x_Device.h"
#include "include/F2802x_Device.h"     // DSP2802x Headerfile Include File

#include "types.h"
#include "init.h"

#include "5110.h"

#include "./include/DSP28x_Project.h"
#include "./include/F2802x_Examples.h"
#include "./include/F2802x_GlobalPrototypes.h"

/*
#include "./include/cpu.h"
#include "./include/clk.h"
#include "./include/flash.h"
#include "./include/gpio.h"
#include "./include/pll.h"
#include "./include/pwm.h"
#include "./include/wdog.h"
#include "./include/pie.h"
//#include "./include/pie_init.h"
#include "./include/timer.h"
#include "./include/adc.h"
#include "./include/sci.h"
*/

//#include "./F28027_SCI.h"
//#include "TM1638.h"

#include "n5110.h"
#include "5110.h"

#include "interrupts.h"

/* ************************************************************************** */

extern CPU_Handle   myCpu;
extern PLL_Handle   myPll;
extern CLK_Handle   myClk;
extern FLASH_Handle myFlash;
extern ADC_Handle   myAdc;
extern SCI_Handle   mySci;
extern GPIO_Handle  myGpio;
extern PIE_Handle   myPie;
extern PWM_Handle   myPwm1, myPwm2, myPwm3;
extern TIMER_Handle myTimer;
extern WDOG_Handle  myWDog;


/* ************************************************************************** */
// j6   | pin   |
// -----+-------+---------------------------------------------------------------
// RST  | GPIO0 |   �����
// CE   | GPIO1 |   ���������
// DC   | GPIO2 |   ������� / ������
// SDIN | GPIO3 |   ���������������� ���� ������
// SCLK | GPIO4 |   ���������������� ���� ������
/* ************************************************************************** */


/* ************************************************************************** */
/*
 * ���                   : init_5110
 * ��������              : ���������� ������������� ����� � SPI ��, ����������� LCD
 * ��������(�)           : ���
 * ������������ �������� : ���
 */
/* ************************************************************************** */
void init_5110() {
    rst_l;           // Reset (2) 0 OXFD = 11111101
    DELAY(10);       // �������� 5us
    rst_h;           // Reset (2) 1

    ce_l;            // ��� �������� (3) 0, �������� 0xFB = 11111011
    DELAY(0);        // �������� 5us
    ce_h;            // ��� �����

    // ���������� ������� �������
    // ........................................................................
    // �������� ����������� ����� ������ (LCD Extended Commands)
    // �������� ������
    write_com(0x21);

    // ��������� ������������� (LCD Vop)
    // 0xC8 = 11001000
    write_com(0xc8);

    // ��������� �������������� ������������ (Temp coefficent)
    // 0x06 = 00000110
    write_com(0x06);

    // ��������� ������� (LCD bias mode 1:48)
    // 0x13 = 00010011
    write_com(0x13);

    // �������� ����������� ����� ������ � �������������� ���������
    // (LCD Standard Commands,Horizontal addressing mode)
    // �������� ����� ������.
    write_com(0x20);

    // ���������� ����� (LCD in normal mode)
    // ������� ����� 0x0c = 00001100
    write_com(0x0c);
    // ........................................................................

    lcd_clear();

    ce_l;            // ��� �������� 0xFB = 11111011
}
/* ************************************************************************** */


/* ************************************************************************** */
void write_byte (char DATA) {
    char i,j;

    for(i=0;i<8;i++) {
        j=DATA&0x80;    // ���������� �������
        if(j>0) sdin_h; // ���� ������� ����� 1, �� SDIN (4) �������� �������
        else  sdin_l;   // ���� ������� ����� 0, �� SDIN (4) ������ 0xEF=11101111
        sclk_l;         // SCLK (5) ������, 0xDF = 11011111
        //DELAY(3);       // �������� 30 ������, �������� 35us
        DELAY(1);       // �������� 30 ������, �������� 35us
        sclk_h;         // SCLK (5) �������� �������
        DATA<<=1;       // �������� ������
    }
    sclk_l;
    sdin_l;             // ������� SDIN (4), SCLK (5) ������ 0XCF=11001111
}
/* ************************************************************************** */


/* ************************************************************************** */
void write_com(char com) {
    ce_l;              // �������� ��� (2)=0 ������� (3)=0 �����, ������ 0 0xC3=11000011
	dc_l;
    //DELAY(2);          // �������� 25us
    DELAY(1);          // �������� 25us
    write_byte(com);   // �������� ������� ������
    ce_h;              // ��������� ��� (2) = 1;
}
/* ************************************************************************** */


/* ************************************************************************** */
void write_data(char dat) {
    ce_l;              // �������� ��� (2)=0 ������ (3)=1 ������ ���� 0 0XCB=11001011
	dc_h;
    //DELAY(2);          // �������� 25us
    DELAY(1);          // �������� 25us
    write_byte(dat);   // �������� ������� ������
    ce_h;              // ��������� ��� (2) = 0;
}
/* ************************************************************************** */


/* ************************************************************************** */
void set_row(char row) {
    write_com(0x80+row); // ���������������� ������ ����� �������
}
/* ************************************************************************** */


/* ************************************************************************** */
void set_col(char col) {
    write_com(0x40+col); // ���������������� ������ ����� ������
}
/* ************************************************************************** */


/* ************************************************************************** */
/*
 * ���                   : lcd_clear
 * ��������              : ������� �������. ����� ���������� ��������� LcdUpdate
 * ��������(�)           : ���
 * ������������ �������� : ���
 */
/* ************************************************************************** */
void lcd_clear (void) {
    int i,j;

    for (i=0; i<6; i++) {
    set_col(i);            // �����������
    set_row(0);            // ���������� ������ �������

    for ( j=0; j<84; j++ ) {
        write_data (0x00); // �������� ������ 83 �� ������ ������� ������� ������� �������� 0
    }
  }
}
/* ************************************************************************** */





#if 0
/* ************************************************************************** */
/*
 * ���                   :  LcdContrast
 * ��������              :  ������������� ������������� �������
 * ��������(�)           :  �������� -> �������� �� 0x00 � 0x7F
 * ������������ �������� :  ���
 */
/* ************************************************************************** */
void LcdContrast ( byte contrast ) {
    LcdSend( 0x21, LCD_CMD );              // ����������� ����� ������
    LcdSend( 0x80 | contrast, LCD_CMD );   // ��������� ������ �������������
    LcdSend( 0x20, LCD_CMD );              // ����������� ����� ������, �������������� ���������
}
/* ************************************************************************** */


/* ************************************************************************** */
/*
 * ���                   :  LcdSend
 * ��������              :  ���������� ������ � ���������� �������
 * ��������(�)           :  data -> ������ ��� ��������
 *                          cd   -> ������� ��� ������ (������ enum � n5110.h)
 * ������������ �������� :  ���
 */
/* ************************************************************************** */
void LcdSend ( byte data, LcdCmdData cd ) {
    // �������� ���������� ������� (������ ������� ��������)
    LCD_PORT &= ~( _BV( LCD_CE_PIN ) );

    if ( cd == LCD_DATA ) {
        LCD_PORT |= _BV( LCD_DC_PIN );
    } else {
        LCD_PORT &= ~( _BV( LCD_DC_PIN ) );
    }

    // �������� ������ � ���������� �������
    SPDR = data;

    // ���� ��������� ��������
    while ( (SPSR & 0x80) != 0x80 );

    // ��������� ���������� �������
    LCD_PORT |= _BV( LCD_CE_PIN );
}
/* ************************************************************************** */

#endif //0
