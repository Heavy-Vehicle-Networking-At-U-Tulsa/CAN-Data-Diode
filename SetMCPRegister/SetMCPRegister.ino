

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

uint8_t SPI_transfer(uint8_t data){
  uint8_t data_in = 0;
  int8_t i = 7;
  while(i >= 0){
    //mask the bit of interest, shift it to the first position and send it.
    digitalWrite(DO,((1<<i) & data) >> i) ;
    digitalWrite(GREEN,((1<<i) & data) >> i) ;
    digitalWrite(SCK,HIGH); //Clock out the data
    bool in = digitalRead(DI);
    data_in |= in << i; //Bitwise OR to build the input
    digitalWrite(RED,in); 
    digitalWrite(SCK,LOW); 
    i--;
  }
  digitalWrite(GREEN,LOW) ;
  digitalWrite(RED,LOW); 
      
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
  SPI_transfer(0b11000000); //Reset
  digitalWrite(CS,HIGH);
  
  delay(1);
  
  tx_buff[0] = 0x05; // instruction
  tx_buff[1] = 0x0F; //Address Byte
  tx_buff[2] = 0xFF; //mask byte
  tx_buff[3] = 0b00000100; //data byte

  digitalWrite(CS,LOW);
  SPI_transfer(0x05);
  SPI_transfer(0x0F);
  SPI_transfer(0xFF);
  SPI_transfer(0b00000100);
  digitalWrite(CS,HIGH);
  
}

void loop()
{
  delay(1);
}

