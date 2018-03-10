/*
Set MCP25625 PRescaler
https://github.com/SpenceKonde/ATTinyCore/blob/master/avr/extras/ATtiny_x61.md
 */
#include <SPI.h>

#define CS 6 //The chip select pin for SPI

uint8_t tx_buff[4];

void setup() {
  // set the digital pin as output:
  pinMode(CS,OUTPUT);
  digitalWrite(CS,HIGH);
  delay(1);
  
  SPI.begin();
  delay(1);

  //Reset the CAN Controller
  digitalWrite(CS,LOW);
  uint8_t recieved = SPI.transfer(0b11000000); //Reset
  digitalWrite(CS,HIGH);
  
  delay(1);
  
  tx_buff[0] = 0x05; // instruction
  tx_buff[1] = 0x0F; //Address Byte
  tx_buff[3] = 0xFF; //mask byte
  tx_buff[3] = 0b00000100; //data byte

  digitalWrite(CS,LOW);
  uint8_t recieved = SPI.transfer(tx_buff,4);
  digitalWrite(CS,HIGH);
  
}

void loop()
{
  delay(1);
}

