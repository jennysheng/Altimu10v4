/*
 * GccApplication1.c
 * Created: 2015-03-21 13:56:19
 * Author: Jenny
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>

#define F_CPU 16000000UL
#define BaudRate 9600
#define MYUBRR   (F_CPU / 16 / BaudRate ) - 1

#define Accelerometer_addr    0x1E//0b0011000
#define Magnetrometer_addr 0x1E
#define Gyrometer_addr 0x6A
#define barometer_addr 0x5C


unsigned char serialCheckTxReady(void) {
	return( UCSR0A & _BV(UDRE0) ) ;  // nonzero if transmit register is ready to receive new data.
}

void serialWrite(unsigned char DataOut) {
	while (serialCheckTxReady() == 0)  // while NOT ready to transmit
	{;;}
	UDR0 = DataOut;
}

void Error_TWI(void){
	
		PORTD^=(1<<PIND3);
		_delay_ms(200);
	    PORTD &= ~(1<<PIND3);
		serialWrite(0x57);//Wrong
	
}
void OK_TWI(void){
	
	    PORTD^=(1<<PIND2);
		_delay_ms(200);	   
	     PORTD &= ~(1<<PIND2);
		 serialWrite(0x52);//Right
}



void CheckStatusRegister(uint8_t status)
{
	if ((TWSR & 0xF8) !=status) //INSERT ERROR CODE HERE
	Error_TWI();
	else
	OK_TWI();	
}

void writeACK(uint8_t data)
{
	TWDR=data;
	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while (!(TWCR & (1 <<TWINT)));
}
void writeData(uint8_t data)
{
	TWDR= data;
	TWCR=(1<<TWINT)|(1<<TWEN);
	while (!(TWCR & (1 <<TWINT)));
}

uint8_t readACK()
{
	
	 //TWI Interrupt Flag, TWI Enable Bit, TWI Enable Acknowledge Bit
	 TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	 //Wait for TWINT Flag set.
	 while (!(TWCR & (1<<TWINT)));
	 return TWDR;
}

uint8_t readData()
{
	//TWI Interrupt Flag, TWI Enable Bit, TWI Enable Acknowledge Bit
	TWCR = (1<<TWINT)|(1<<TWEN);
	//Wait for TWINT Flag set.
	while (!(TWCR & (1<<TWINT)));
	return TWDR;
}


void start()
{
	TWCR = (1 << TWEN) | (1 << TWINT) | (1<< TWSTA);
	while (!(TWCR & (1 <<TWINT)));
}

void stop()
{
	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	
	//Wait for STOP to finish
	while(TWCR & (1<<TWSTO));
}


void initInterrupt(){
 /************************************************************************/
 /* Initiate Interrupt                                                  */
 /************************************************************************/

  OCR1A=15624;
 // DDRC = (1<<PC4)||(1<<PC5);		// Set PC4 and PC5 as input (Using for interrupt)
 // PORTC = (1<<PD4)||(1<<PC5);		// Enable PD4&5 pull-up resistor
  sei();		//Enable Global Interrupt
  
  /************************************************************************/
  /* Initiate TIMER1                                    */
  /************************************************************************/
  TCCR1A = 0b10000000;//Clears our comparison value at match
  TCCR1B=0b00001011;
  //TCCR1B = 0b00001101; //Use a prescaler of clock/1024 and enable clear timer on compare (ctc)//Start timer without prescaller TCCR1B|=(1<<CS10);
  TCNT1H = 0; //TCNT1H+L is the start value for our timer
  TCNT1L = 0; //
  TIMSK1 = 0b00000010; //Enable interrupt for compare match mot "A"
}
void initUSART(){
  /************************************************************************/
  /* Initiate USART                                                       */
  /************************************************************************/
  /*Set baud rate */
  UBRR0H = (unsigned char)(MYUBRR>>8);
  UBRR0L = (unsigned char) MYUBRR;
  /* Enable receiver and transmitter   */
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  /* Frame format: 8 data, No parity, 1stop bit */
  UCSR0C = (3<<UCSZ00);
  

  
}
void TWI_init(){
	//Initialize I2C
		PRR &= ~(1 << PRTWI);	//Turn off power reduction so i2c is not disabled
		//TWCR &= ~(1 << TWIE);	//Turn off the Interrupt Enable
		//TWCR |=(1<<TWIE);
		TWBR = 12;	//Set the bit rate register at 12 for 16 MHz CPU Clock and SCL I2C Clock at 400KHz
		//TWSR &= ~(1 << TWPS1) | (1 << TWPS0); //No Prescaler to mess up the status register
		TWSR=0;
	
}

 

void TWIwrite(uint8_t slaveAddr, uint8_t reg, uint8_t data){
	//Start
	start();
	//CheckStatusRegister(0x08);
//	serialWrite(0x41);//A	
	writeData(slaveAddr<<1);
	//CheckStatusRegister(0x18);// check the status register for the slave SLA+W transmitted ACK received)
//	serialWrite(0x42);//B
	//Write data (Reg.Addr)	
	writeData(reg);
	//CheckStatusRegister(0x28);// check the status register for the slave register transmitted ACK received)
//	serialWrite(0x43);//C	
	writeData(data);
	//CheckStatusRegister(0x28);// check the status register for the slave data transmitted ACK received)
//	serialWrite(0x44);//D
	//Stop
	stop();
	
}


uint8_t TWIread(uint8_t slaveAddr, uint16_t reg){
	uint8_t data;
	//Start
	start();
	//CheckStatusRegister(0x08);	
	//serialWrite(0x47);//G		
   //Send SLA + W 0x1E accelerometer
    writeData((slaveAddr<<1));
	//CheckStatusRegister(0x18);// check the status register for the slave SLA+W transmitted ACK received)
	//serialWrite(0x48);//H
	writeData(reg);
	//CheckStatusRegister(0x50);// check the status register for the slave SLA+W transmitted ACK received)
//	serialWrite(0x49);//I
	//Repeat start
	start();
	//CheckStatusRegister(0x10);
	//serialWrite(0x4A);//J
	 //send slave address and r
    writeData((slaveAddr<<1)|1);
	//CheckStatusRegister(0x40);// check the status register for the slave SLA+R transmitted ACK received)
	//serialWrite(0x4B);//K
	data=readData();	
	//CheckStatusRegister(0x58);// check the status register for the data  received and Master NACK sent)
	//serialWrite(0x4C);//L
	//Stop
	stop();
	return data;
}

void TWIreadMultiple(uint8_t slaveAddr, uint16_t reg,uint8_t *array, uint8_t size){
	//Start
	start();
	//CheckStatusRegister(0x08);
	//serialWrite(0x47);//G
	//Send SLA + W 0x1E accelerometer
	writeData((slaveAddr<<1));
	//CheckStatusRegister(0x18);// check the status register for the slave SLA+W transmitted ACK received)
	//serialWrite(0x48);//H
	writeData(reg);
	//CheckStatusRegister(0x50);// check the status register for the slave SLA+W transmitted ACK received)
	//serialWrite(0x49);//I
	//Repeat start
	start();
	//CheckStatusRegister(0x10);
	//serialWrite(0x4A);//J
	//send slave address and r
	writeData((slaveAddr<<1)|1);
	//CheckStatusRegister(0x40);// check the status register for the slave SLA+R transmitted ACK received)
	//serialWrite(0x4B);//K
		int i=0;
		while(i<size) {
			if ((i+1)!=size)
			array[i]=readACK();
			else array[i]=readData(); // read without ACK on last byte
			i++;
		}
	//CheckStatusRegister(0x58);// check the status register for the data  received and Master NACK sent)
	//serialWrite(0x4C);//L
	//Stop
	stop();
	
}
/******************************************************************//**
 * Sends a single character via UART (eg. to a Hyperterminal).
 *********************************************************************/
void uart_putc(unsigned char c)
{
    while (!(UCSR0A & (1<<UDRE0)))  // wait until sending is possible
    {
    }                             
    UDR0 = c;                       // sending signs
}
 
/******************************************************************//**
 * Sends a complete string via UART (eg. to a Hyperterminal).
 *********************************************************************/
void uart_puts (char *s)
{
    while (*s)
    {                               // sending char until "/0"
        uart_putc(*s);
        s++;
    }
}
/******************************************************************//**
 * Sends an Integer via UART (eg. to a Hyperterminal).
 *********************************************************************/
void uart_puti16 (uint16_t value)
{
	char _buffer[6];
	itoa( value, _buffer, 10 );	//conversion from integer to char
	uart_puts(_buffer);
}
/************************************************************************/
/* Interrupt method, reads from the I2C-bus and then sends it via
   USART										                        */
/************************************************************************/



char s[7];
ISR(TIMER1_COMPA_vect){
/************************************************************************/
/* Get the value from  accelerometer/magnetrometer send through usart.			        */
/************************************************************************/
   uart_puts("Acc: ");
	uint8_t jenny[5];
	uint16_t accelerometer[2];
	TWIreadMultiple(Accelerometer_addr,0xA8,jenny,5);//MSB=1 read from OUT_X_L 0x28;  0b0010 1000

 	accelerometer[0] = (int16_t)(jenny[0]|jenny[1] << 8 );
	accelerometer[1] = (int16_t)(jenny[2]|jenny[3] << 8 );
	accelerometer[2] = (int16_t)(jenny[4]|jenny[5] << 8 );

	uart_puts("X:");
	uart_puts( " " );
	itoa(accelerometer[0], s, 10 );
	uart_puts( s );
	uart_puts( "\t\t" );
	
	uart_puts("Y:");
	uart_puts( " " );
	itoa(accelerometer[1], s, 10 );
	uart_puts( s );
	uart_puts( "\t\t" );
	
	uart_puts("Z:");
	uart_puts( " " );
	itoa(accelerometer[2], s, 10 );
	uart_puts( s );
	uart_puts( "\t\t" );	
	uart_puts( "\n\r" ); //new line
	_delay_ms(100000);
/************************************************************************/
/* Get the value from  accelerometer/magnetrometer send through usart.			        */
/************************************************************************/
	    uart_puts("Mag: ");
		uint8_t jenni[5];
		uint16_t magnetrometer[2];
		TWIreadMultiple(Magnetrometer_addr,0x08|(1<<7),jenni,5);//MSB=1 read from OUT_X_L 0x28;  0b0010 1000

		magnetrometer[0] = (int16_t)(jenni[0]|jenni[1] << 8 );
		magnetrometer[1] = (int16_t)(jenni[2]|jenni[3] << 8 );
		magnetrometer[2] = (int16_t)(jenni[4]|jenni[5] << 8 );

	
	uart_puts("X:");
	uart_puts( " " );
	itoa(magnetrometer[0], s, 10 );
	uart_puts( s );
	uart_puts( "\t\t" );
	
	uart_puts("Y:");
	uart_puts( " " );
	itoa(magnetrometer[1], s, 10 );
	uart_puts( s );
	uart_puts( "\t\t" );
	
	uart_puts("Z:");
	uart_puts( " " );
	itoa(magnetrometer[2], s, 10 );
	uart_puts( s );
	uart_puts( "\t\t" );
	uart_puts( "\n\r" ); //new line
	_delay_ms(100000);

/************************************************************************/
/* Get the value from  gyrometer send through usart.			        */
/************************************************************************/
	 uart_puts("Gyro: ");
	uint8_t array[5];
	uint16_t gyrometer[2];
	TWIreadMultiple(Gyrometer_addr,0xA8,array,5);//MSB=1 read from OUT_X_L 0x28;  0b0010 1000

 	gyrometer[0] = (int16_t)(array[0]|array[1] << 8 );
	gyrometer[1] = (int16_t)(array[2]|array[3] << 8 );
	gyrometer[2] = (int16_t)(array[4]|array[5] << 8 );
uart_puts("X:");
uart_puts( " " );
itoa(gyrometer[0], s, 10 );
uart_puts( s );
uart_puts( "\t\t" );
uart_puts("Y:");
uart_puts( " " );
itoa(gyrometer[1], s, 10 );
uart_puts( s );
uart_puts( "\t\t" );
uart_puts("Z:");
uart_puts( " " );
itoa(gyrometer[2], s, 10 );
uart_puts( s );
uart_puts( "\t\t" );	
uart_puts( "\n\r" ); //new line	
	_delay_ms(100000);
/************************************************************************/
/* Get the value from  barometer send through usart.			        */
/************************************************************************/
   uart_puts("Baro: ");
	uint8_t jen[5];
	uint32_t barometer[1];
	TWIreadMultiple(barometer_addr,0x28|(1<<7),jen,2);//MSB=1 read from OUT_X_L

	barometer[0] = (int32_t)(jen[0]|jen[1] << 8|jen[2]<<16 );
	
	 uart_puts(" ");
	
	itoa(barometer[0], s, 10 );
	uart_puts( s );
	uart_puts( "\t\t" );
	uart_puts( "\n\r" ); //new line
	_delay_ms(10000);


}
uint8_t main(void)
{
	initUSART();
	TWI_init();	
	/************************************************************************/
	/* Configurate gyrometer		        */
	/************************************************************************/
	//TWIwrite(Accelerometer_addr,0x23,0x08);	//0b0000 1000
	 // DR = 01 (200 Hz ODR); BW = 10 (50 Hz bandwidth); PD = 1 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
	// writeReg(CTRL_REG1, 0x6F);
	TWIwrite(Gyrometer_addr,0x20, 0x6F); // 0b0100 0111
// 	 // writeReg(CTRL_REG4, 0x00);
 	TWIwrite(Gyrometer_addr,0x23, 0x00); // 0b0100 0111
// 	 // writeReg(LOW_ODR, 0x00);
 	TWIwrite(Gyrometer_addr,0x39, 0x00); // 0b0100 0111
	 /************************************************************************/
	 /* Configurate accelerometer/magnetrometer 			                 */
	 /************************************************************************/
	  // Accelerometer
	  // 0x00 = 0b00000000
	  // AFS = 0 (+/- 2 g full scale)
	 // writeReg(CTRL2, 0x00);
       TWIwrite(Accelerometer_addr,0x21,0x00);
	  // 0x57 = 0b01010111
	  // AODR = 0101 (50 Hz ODR); AZEN = AYEN = AXEN = 1 (all axes enabled)
	 // writeReg(CTRL1, 0x57);
	  TWIwrite(Accelerometer_addr,0x20,0x57);

	  // Magnetometer
	  // 0x64 = 0b01100100
	  // M_RES = 11 (high resolution mode); M_ODR = 001 (6.25 Hz ODR)
	  //writeReg(CTRL5, 0x64);
	  TWIwrite(Magnetrometer_addr,0x24,0x64);

	  // 0x20 = 0b00100000
	  // MFS = 01 (+/- 4 gauss full scale)
	 // writeReg(CTRL6, 0x20);
	 TWIwrite(Magnetrometer_addr,0x25,0x20);

	  // 0x00 = 0b00000000
	  // MLP = 0 (low power mode off); MD = 00 (continuous-conversion mode)
	  //writeReg(CTRL7, 0x00);
	  TWIwrite(Magnetrometer_addr,0x26,0x00);
	 
	 
	 /************************************************************************/
	 /* Configurate barometer			                                     */
	 /************************************************************************/
	 
	 // 0xB0 = 0b10110000
	 // PD = 1 (active mode);  ODR = 011 (12.5 Hz pressure & temperature output data rate)
	 //writeReg(CTRL_REG1, 0xB0);
	 TWIwrite(barometer_addr,0x20,0xB0);
	
	initInterrupt();

	while(1){

	}
	return 0;
}