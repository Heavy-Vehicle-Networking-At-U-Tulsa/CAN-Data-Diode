#include <util/atomic.h>


/*
Set MCP25625 PRescaler
https://github.com/SpenceKonde/ATTinyCore/blob/master/avr/extras/ATtiny_x61.md
 */

#define GREEN 10
#define RED 14
#define CS 6 //The chip select pin for SPI
#define SCK 7
#define DO 9 
#define DI 8
#define SILENT 11

#define RESET       0b11000000
#define READ_STATUS 0b10100000
#define BIT_MODIFY  0b00000101
#define READ        0b00000011
uint8_t status_byte;

uint8_t SPI_transfer(uint8_t data){
  uint8_t data_in = 0;
  //Use bitwise ORs and ANDs to set and clear bits directly on the port.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for (int8_t i = 0;i<8;i++){
      //Check to see what the first bit is in the data
      if ((data & 0x80) == 0x80) PORTB |= 0b00000001; //Set the bit on PB0
      else PORTB &= 0b11111110; //clear the bit on PB0
      PORTB |= 0b00000100; //Clock out the data on PB2
      data <<= 1;
      data_in <<=1;
      data_in |= (PORTB & 0x02) >> 1; //Bitwise OR with PB1 to build the input
      PORTB &= 0b11111011; //Set the ClockPin Low (PB2)     
    }
  }    
  return data_in;    
}

uint8_t tx_buff[4];

void setup() {
  // set the digital pin as output:
  pinMode(CS,OUTPUT);
  pinMode(SCK,OUTPUT);
  pinMode(DI,INPUT_PULLUP);
  pinMode(DO,OUTPUT);
  pinMode(GREEN,OUTPUT);
  pinMode(RED,OUTPUT);
  pinMode(SILENT,OUTPUT);
  
  
  digitalWrite(GREEN,HIGH) ;
  digitalWrite(RED,HIGH); 
  digitalWrite(CS,HIGH);
  digitalWrite(SCK,LOW);
  digitalWrite(SILENT,LOW);
  delay(1);
  

  //Reset the CAN Controller
  digitalWrite(CS,LOW);
  SPI_transfer(RESET); //Reset
  digitalWrite(CS,HIGH);
  
  delay(1);
  

  digitalWrite(CS,LOW);
  SPI_transfer(BIT_MODIFY); // instruction to modify bits
  SPI_transfer(0x0F); //Address Byte
  SPI_transfer(0xFF); //mask byte
  SPI_transfer(0b00000100); //data byte to enable 1 clock out
  digitalWrite(CS,HIGH);

  delay(1);  
}

void loop()
{
  //check status
  digitalWrite(CS,LOW);
  SPI_transfer(READ_STATUS); // instruction to READ STATUS
  status_byte = SPI_transfer(0xFF); //Address Byte
  digitalWrite(CS,HIGH);
  
  digitalWrite(CS,LOW);
  SPI_transfer(READ); // instruction to READ 
  SPI_transfer(0x0F); //Address
  for (int i = 0;i<8;i++){
    status_byte = SPI_transfer(0x55); //Address Byte
  }
  digitalWrite(CS,HIGH);
  
  digitalWrite(RED,HIGH);
  delay(5);
  digitalWrite(RED,LOW);
  delay(5);
  
}

