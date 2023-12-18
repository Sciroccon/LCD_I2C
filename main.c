#include "stm32f4xx.h"
#include "delay.h"
#include "lcd_driver.h"
#include "usart.h"




int main(void){
    I2C_Conf();
    LCD_Init();
    delay_ms(20);
    LCD_Cursor(0,0);
    LCD_Write_Data(LCD_ADDR, 'H'); // Character Generation ROM Pattern for 'H'
    return 0;
}
