#include "stm32f4xx.h"
#include "lcd_driver.h"
#include "delay.h"
#include "usart.h"

#define I2C_DELAY 1000

uint8_t clc; 	// varijabla za citanje SR1 i SR2 reg u svrhu ciscenja ADDR zastavice


void I2C_Conf(void){
RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;  // Uključuje I2C1 periferiju
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Uključuje GPIOB periferiju (port na kojem je I2C1 povezan)

// I2C software reset
I2C1->CR1 |= I2C_CR1_SWRST;
I2C1->CR1 &= ~(I2C_CR1_SWRST);


GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; // Alternativna funkcija
GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;     // Otvoreni kolektor/otvoreni izlaz
GPIOB->OSPEEDR |= ( (2UL<<(6*2)) | (2UL<<(7*2)) );
// GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7; // Visoka brzina
GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0; // Pull-up
GPIOB->AFR[0] |= (4 << (6 * 4)) | (4 << (7 * 4)); // AF4 za I2C1 na pinovima PB6 i PB7

// Konfiguracija registara za I2C
I2C1->OAR1 |= 0x4000;
I2C1->CR1 &= ~I2C_CR1_PE; 	// Isključuje I2C prije nego što se konfigurise
I2C1->CR2 = 0x0010; 		// 16MHz takt, moze ici do max 42Mhz
I2C1->CCR = 0x50;			// T=1/F=1/100khz=0.01ms  Duty cycle Ton=0.01ms/2=5000ns jer hocemo da je duty cycle 0.5
							// Periferal clock radi na 16MHz, te ide da je perio Tpclk=1/16Mhz=62.5ns, te ide CCR=5000ns/62.5=80 ili moze ici CCR=16Mhz/2*100khz
I2C1->TRISE = 0x11; 		// Računamo prema formuli za 100 kHz brzinu (Periferal_clock(16Mhz)/1Mhz) + 1)= 16+1= 17


I2C1->CR1 |= I2C_CR1_PE; 	// Ponovno uključi I2C
initUSART2(USART2_BAUDRATE_921600);
}


void I2C_Start(){

	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){} 		// Počni START sekvencu

}

void I2C_Write(uint8_t data){
	while(!(I2C1->SR1 & I2C_SR1_TXE)); 		// Provjerava da li je data registar prazan
	I2C1->DR = data;
	while(!(I2C1->SR1 & I2C_SR1_BTF)){}		// Provjerava da li je bajt prenesen

}

void I2C_Send_Addr(uint8_t Addr){
    I2C1->DR = Addr;  						// Adresa koja se salje
    while(!(I2C1->SR1 & I2C_SR1_ADDR)){}	// Provjerava da li je poslata adresa
    clc = (I2C1->SR1 | I2C1->SR2);			// Da bi se ocistio ADDR bit, moraju se procitati SR1 I SR2 registri uzastopno
}
void I2C_Stop(){
    I2C1->CR1 |= I2C_CR1_STOP;				// Zaustavlja komunikaciju
	while(I2C1->CR1 & I2C_CR1_STOP);
}



// RS(Register select),0 - instrukcijski registar, 1 - data registar , RW(Read/Write), 0 - pisanje, 1 - citanje, 
// E(Enable), treba pulsirati pin kako bi se procitala i obradila komanda za LCD,
// B(Backlight), pozadinsko osvjetljenje 1 - upaljeno, 0 ugaseno
void LCD_Write_Cmd8bit(uint8_t DevAddr,uint8_t data){ // bajagi jer LCD starta u 8bitnom modu treba slat samo upper dio 8bitne komande
uint8_t data_h,d1,d2;
data_h=(data & 0xF0);
d1=data_h | 0x04; 		// Blacklight_enable,	E enable  D7 D6 D5 D4 B E RW RS 
d2=data_h | 0x00; 		// Isto samo bez E enable

I2C_Start();
I2C_Send_Addr(DevAddr);	// Prvo se salje adresa slave uredjaja kao prvi bajt prijenosa
I2C_Write(0x00);		// RS,RW I E u nepoznatom stanju na pocetku postavljaju se na vrijednosti za upisivanje cmd u instrukcijski reg


// F-ija za slanje podataka LCDa u 8bitnom modu,
// posalju se gornja 4 bita, sa E 1, i ponovo se salje taj isti gornji dio samo sa E pinom na 0
I2C_Write(d1);
TIM4_ms_Delay(2);
I2C_Write(d2);

TIM4_ms_Delay(2);
I2C_Stop();
printUSART2("Init komande 8 bit");
}

void LCD_Write_Cmd4bit(uint8_t DevAddr,uint8_t data)
{
	uint8_t data_h,data_l,d1,d2,d3,d4;
	data_h=(data & 0xF0);
	data_l=((data<<4) & 0xF0);

	d1=data_h | 0x04; // 1100
	d2=data_h | 0X00;	

	d3=data_l | 0X04;
	d4=data_l | 0X00;

	I2C_Start();
	I2C_Send_Addr(DevAddr); 		
	I2C_Write(0x00);


	I2C_Write(d1);
	TIM4_ms_Delay(2);
	I2C_Write(d2);

	TIM4_ms_Delay(2);

	I2C_Write(d3);
	TIM4_ms_Delay(2);
	I2C_Write(d4);

	TIM4_ms_Delay(2);
	I2C_Stop();
	printUSART2("Proslo CMD slanje");
}

void LCD_Write_Data(uint8_t DevAddr,uint8_t data)
{
	uint8_t data_h,data_l,d1,d2,d3,d4;
	data_h=data & 0xF0;
	data_l=(data<<4) & 0xF0;

	d1=data_h | 0X05;
	d2=data_h | 0X01;	

	d3=data_l | 0X05;
	d4=data_l | 0X01;

	I2C_Start();
	I2C_Send_Addr(DevAddr); 			// Prvo se salje adresa slave uredjaja
	I2C_Write(0x01);


	I2C_Write(d1);
	TIM4_ms_Delay(2);
	I2C_Write(d2);

	TIM4_ms_Delay(2);

	I2C_Write(d3);
	TIM4_ms_Delay(2);
	I2C_Write(d4);

	TIM4_ms_Delay(2);
	I2C_Stop();
	printUSART2("Proslo DATA slanje");
}


void LCD_Init(){
    // 1. Inicijalizacijske funkcije za LCD --- pocinje raditi u 8 bitnom modu, te se moraju slati 8bitne komande
	TIM4_ms_Delay(150);
    LCD_Write_Cmd8bit(LCD_ADDR,0x33);
	TIM4_ms_Delay(10);
	LCD_Write_Cmd8bit(LCD_ADDR,0x32);

	//
	TIM4_ms_Delay(5);
	LCD_Write_Cmd4bit(LCD_ADDR,0x0C);
	TIM4_ms_Delay(5);
	LCD_Write_Cmd4bit(LCD_ADDR,0x01);
	TIM4_ms_Delay(5);
	LCD_Write_Cmd4bit(LCD_ADDR,0x06);

}


void LCD_Cursor(int r, int c){
    if (r==1){
        c |= 0xC0;
        LCD_Write_Cmd4bit(LCD_ADDR,c);
    }
    else{
        c |= 0x80;
        LCD_Write_Cmd4bit(LCD_ADDR,c);
    }
}

