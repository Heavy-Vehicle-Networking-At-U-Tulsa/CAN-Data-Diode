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

#define RESET       0b11000000
#define READ_STATUS 0b10100000
#define BIT_MODIFY  0b00000101
#define READ        0b00000011
uint8_t status_byte;

uint8_t SPI_transfer(uint8_t data){
  uint8_t data_in = 0;
  int8_t i = 7;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    while(i >= 0){
      //mask the bit of interest, shift it to the first position and send it.
      digitalWrite(DO,((1<<i) & data) >> i) ;
      digitalWrite(SCK,HIGH); //Clock out the data
      data_in |= digitalRead(DI) << i; //Bitwise OR to build the input
      digitalWrite(SCK,LOW); 
      i--;
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
  
  
  digitalWrite(GREEN,HIGH) ;
  digitalWrite(RED,HIGH); 
  digitalWrite(CS,HIGH);
  digitalWrite(SCK,LOW);
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
  
  digitalWrite(GREEN,HIGH);
  delay(5);
  digitalWrite(GREEN,LOW);
  delay(5);
  
}

