#include "stm32f4xx.h"
#include "delay.h"
#include "lcd_driver.h"
#include "usart.h"





int main(void){


    I2C_Conf();
    LCD_Init();
    LCD_Send_String("Kako ste mi dns u Tz");
return 0;
}