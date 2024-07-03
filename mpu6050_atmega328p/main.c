
#define F_CPU 16000000UL								/* Define CPU clock Frequency e.g. here its 8MHz */
#include <avr/io.h>										/* Include AVR std. library file */
#include <avr/interrupt.h>
#include <util/delay.h>									/* Include delay header file */
#include <inttypes.h>									/* Include integer type header file */
#include <stdlib.h>										/* Include standard library file */
#include <stdio.h>										/* Include standard library file */
#include <stdbool.h>
#include "MPU6050_res_define.h"							/* Include MPU6050 register define file */
#include "I2C_Master_H_file.h"							/* Include I2C Master header file */
#include "USART_RS232_H_file.h"							/* Include USART header file */
#define FIXED_TIME_STEP 0.01

float Acc_x,Acc_y,Acc_z,Temperature,Gyro_x,Gyro_y,Gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
unsigned long lastTime;
float Input, Output, Setpoint;
float errSum, lastErr;

float Kp = 6.0, Ki = 0.6, Kd = 25;
float trget, err, last_error, integral, derivative;

float speedR = 125;
float speedL = 125;

volatile unsigned long timer1_millis;

float angle = 0;
const float alpha = 0.95;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) +
	out_min;
}

void MPU6050_Init()								/* Gyro initialization function */
{
	_delay_ms(150);										/* Power up time >100ms */
	I2C_Start_Wait(0xD0);								/* Start with device write address */
	I2C_Write(SMPLRT_DIV);								/* Write to sample rate register */
	I2C_Write(0x07);									/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1);								/* Write to power management register */
	I2C_Write(0x01);									/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG);									/* Write to Configuration register */
	I2C_Write(0x00);									/* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG);								/* Write to Gyro configuration register */
	I2C_Write(0x18);									/* Full scale range +/- 2000 degree/C */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE);								/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();
}

void MPU_Start_Loc()
{
	I2C_Start_Wait(0xD0);								/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */ 
	I2C_Repeated_Start(0xD1);							/* I2C start with device read address */
}

void Read_RawValue()
{
	MPU_Start_Loc();									/* Read Gyro values */
	Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
	I2C_Stop();
}


void Compute(){
	double timeChange = FIXED_TIME_STEP;
	
	double errinp = Setpoint - Gyro_x;
	errSum += (errinp * timeChange);
	double dErr = (errinp - lastErr) / timeChange;
	
	Output = Kp * errinp + Ki * errSum + Kd * dErr;
	
	lastErr = errinp;
}

void kalibrasi(int n){
	
	for (int i = 0; i < n; i++){
		char buffer[20], intbuffer[10];
		Read_RawValue();
		
		    //Add the gyro x offset to the gyro_x_cal variable
		    gyro_x_cal += Gyro_x;
		    //Add the gyro y offset to the gyro_y_cal variable
		    gyro_y_cal += Gyro_y;
		    //Add the gyro z offset to the gyro_z_cal variable
		    gyro_z_cal += Gyro_z;
			_delay_us(3);
			
			itoa(n, intbuffer, 10);
			sprintf(buffer,"kalibrasi- %s/s\r\n", intbuffer);
			USART_SendString(buffer);
	}
	gyro_x_cal /= n;
	gyro_y_cal /= n;
	gyro_z_cal /= n;
}

void PWM_init(){
	DDRB |= (1<<PB1)|(1<<PB2)|(1<<PB3); //set pin menjadi OUTPUT
	DDRD |= (1<<PD3)|(1<<PD5)|(1<<PD6);
	
	TCCR2A = (1<<COM2A1)|(1<<COM2B1)|(1<<WGM20)|(1<<WGM21);  // Fast PWM mode
	//mode non-inverting ()
	TCCR2B = (1<<CS22);  // Prescaler 64
	OCR2A = 0;  //  duty cycle
	OCR2B = 0;  //  duty cycle
	
}

int constrain(int val, int min, int max) {
	if (val < min) {
		return min;
		} else if (val > max) {
		return max;
		} else {
		return val;
	}
}

void control_motor(float PIDspeedL, float PIDspeedR) {
	// Kontrol motor kanan
	if (PIDspeedR >= 0) {
		OCR2A = PIDspeedR;  // Set duty cycle
		PORTD |= (1<<PD6);  // Motor kanan maju
		PORTD &= ~(1<<PD5);
		} else {
		OCR2A = -PIDspeedR;  // Set duty cycle
		PORTD &= ~(1<<PD6);  // Motor kanan mundur
		PORTD |= (1<<PD5);
	}

	// Kontrol motor kiri
	if (PIDspeedL >= 0) {
		OCR2B = PIDspeedL;  // Set duty cycle
		PORTB |= (1<<PB1);  // Motor kiri maju
		PORTB &= ~(1<<PB2);
		} else {
		OCR2B = -PIDspeedL;  // Set duty cycle
		PORTB &= ~(1<<PB1);  // Motor kiri mundur
		PORTB |= (1<<PB2);
	}
}


int main()
{
	char buffer[20], buffer2[20], float_[10];
	PWM_init();
	I2C_Init();											/* Initialize I2C */
	MPU6050_Init();										/* Initialize MPU6050 */
	USART_Init(9600);									/* Initialize USART with 9600 baud rate */
	kalibrasi(100);									//kalibrasi dengan 1000 data
	
	while(1)
	{
		Read_RawValue(); //membaca sensor mpu6050
		
		Gyro_x -= gyro_x_cal;
		Gyro_y -= gyro_y_cal;
		Gyro_z -= gyro_z_cal;
		
		Gyro_x = Gyro_x * 0.611;
		
		angle = alpha * (angle + Gyro_x * 0.01) + (1 - alpha) * (atan2(-Acc_y, Acc_z) * 180.0 / M_PI);
		
		Setpoint = -angle;
		
		Compute();
		
		Output = map(Output, -30000, 30000, -255, 255);
		if(Output > 255){
			Output = 255;
		}
		else if(Output < -255){
			Output = -255;
		}
		
		int PIDspeedL = speedL - Output;               // Adjust speeds
		int PIDspeedR = speedR + Output;
		
		PIDspeedL = constrain(PIDspeedL, 0, 125);
		PIDspeedR = constrain(PIDspeedR, 0, 125);
		
		control_motor(PIDspeedL, PIDspeedR);
		
		dtostrf( Gyro_x, 3, 2, float_ );
		sprintf(buffer," Gx = %s%c/s\t",float_,0xF8);
		USART_SendString(buffer);
		
		dtostrf( Output, 3, 2, float_ );
		sprintf(buffer," pid = %s%c/s\t",float_,0xF8);
		USART_SendString(buffer);
		
		dtostrf( PIDspeedL, 3, 2, float_ );
		sprintf(buffer," PIDspeedL = %s%c/s\t",float_,0xF8);
		USART_SendString(buffer);
		
		dtostrf( PIDspeedR, 3, 2, float_ );
		sprintf(buffer2," PIDspeedR = %s%c/s\r\n",float_,0xF8);
		USART_SendString(buffer2);
	}
}
