Repozitorij za interfejsovanje 1602A LCD-a sa STM32F407 plocicom koristeći serijski I2C interfejs modul.LCD radi u 4-bitnom modu,jer serijski I2C(PCF8574T) modul moze da manipulise sa 8 pinova odjednom.
Pinovi P4 do P7 su koristeni za D4 do D7 pinove na LCD displeju i sluze kao data pinovi, dok ostala 4 pina kontrolisu RS,RW,E i B pinove LCD-a.

Uputstvo za spajanje:


![image](https://github.com/Sciroccon/LCD_I2C/assets/104562710/4fcd932e-2ce6-4331-9ed4-29a61382af50)
