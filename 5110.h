/* ************************************************************************** */
#ifndef __5110_H_
#define __5110_H_
/* ************************************************************************** */
#include  "init.h"

/* ************************************************************************** */
#define FALSE              0
#define TRUE               1

// Разрешение дисплея в пикселях
#define LCD_X_RES                  84    // разрешение по горизонтали
#define LCD_Y_RES                  48    // разрешение по вертикали

// Настройки для рисования группы прямоугольников функцией LcdBars ( byte data[], byte numbBars, byte width, byte multiplier )
#define EMPTY_SPACE_BARS           2     // расстояние между прямоугольниками
#define BAR_X                      30    // координата x
#define BAR_Y                      47    // координата y

// Размер кэша ( 84 * 48 ) / 8 = 504 байта
#define LCD_CACHE_SIZE     ( ( LCD_X_RES * LCD_Y_RES ) / 8 )

// Для возвращаемых значений
#define OK                         0   // Безошибочная отрисовка
#define OUT_OF_BORDER              1   // Выход за границы дисплея
#define OK_WITH_WRAP               2   // Переход на начало (ситуация автоинкремента указателя курсора при выводе длинного текста)
/* ************************************************************************** */


/* ************************************************************************** */
/*
// Перечисления
typedef enum {
    LCD_CMD  = 0,     // Команда
    LCD_DATA = 1      // Данные

} LcdCmdData;

typedef enum {
    PIXEL_OFF =  0,   // Погасить пиксели дисплея
    PIXEL_ON  =  1,   // Включить пиксели дисплея
    PIXEL_XOR =  2    // Инвертировать пиксели

} LcdPixelMode;

typedef enum {
    FONT_1X = 1,      // Обычный размер шрифта 5x7
    FONT_2X = 2       // Увеличенный размер шрифта
} LcdFontSize;
*/
/* ************************************************************************** */


/* ************************************************************************** */
//#define gpio_mux  0x0000
//#define gpio_dir  0x003f

/* ************************************************************************** */
#define rst_h     GpioDataRegs.GPASET.bit.GPIO0=1
#define rst_l     GpioDataRegs.GPACLEAR.bit.GPIO0=1
#define ce_h      GpioDataRegs.GPASET.bit.GPIO1=1
#define ce_l      GpioDataRegs.GPACLEAR.bit.GPIO1=1
#define dc_h      GpioDataRegs.GPASET.bit.GPIO2=1
#define dc_l      GpioDataRegs.GPACLEAR.bit.GPIO2=1
#define sdin_h    GpioDataRegs.GPASET.bit.GPIO3=1
#define sdin_l    GpioDataRegs.GPACLEAR.bit.GPIO3=1
#define sclk_h    GpioDataRegs.GPASET.bit.GPIO4=1
#define sclk_l    GpioDataRegs.GPACLEAR.bit.GPIO4=1
/* ************************************************************************** */


/* ************************************************************************** */
//extern void init_sys();
extern void DELAY (char);
extern void InitGpio_Conf_HW ();
extern void init_5110 ();
extern void set_row (char);
extern void set_col (char);
extern void write_com (char);
extern void write_data (char);
extern void InitSysCtrl (void);
extern void InitPieCtrl (void);
extern void InitPieVectTable (void);

extern void lcd_clear ();

#if 0
extern void LcdContrast ( byte );
extern void LcdSend ( byte data, LcdCmdData cd );
#endif //0
/* ************************************************************************** */

#endif //__5110_H_
