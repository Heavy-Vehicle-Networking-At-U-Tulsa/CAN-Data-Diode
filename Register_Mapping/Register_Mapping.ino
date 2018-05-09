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
uint8_t counter = 0;

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

uint8_t

void flash(uint8_t ledPin){
  //flash once
  digitalWrite(ledPin, LOW);
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);
}

void errorFlash(uint8_t ledPin){
  //flash thrice
  digitalWrite(ledPin, LOW);
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);
  delay(200);
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);
  delay(200);
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);
}

void setRegisters (uint8_t address, byte value, int n){
  uint8_t i = 4;
  while(i>=1){
    flash(GREEN);
    digitalWrite(CS, LOW);
    SPI_transfer(0x02); //Write
    SPI_transfer(address); // to address
    SPI_transfer(0xFF); // mask
    SPI_transfer(value); // message to write
    delay(1);
    SPI_transfer(0x03); //read
    SPI_transfer(address); // from address
    byte incoming = shiftIn(DI, SCK, LSBFIRST);
    digitalWrite(CS, HIGH);
    if(incoming != value){
      i--;
    }
    else if(incoming == value){
      break;
    }
    else if(i = 1){
      errorFlash(RED);
    }
  }
}

char registers[128];
byte readRegister(uint8_t address){
  digitalWrite(CS, LOW); 
  SPI_transfer(0x03);
  SPI_transfer(address);
  byte incoming = shiftIn(DI, SCK, LSBFIRST);
  digitalWrite(CS, HIGH);
  return incoming;
}

void setRegisters(uint8_t address){
  digitalWrite(CS, LOW);
  SPI_transfer(0x02);
  SPI_transfer(address);
  SPI_transfer(0xFF);
  SPI_transfer(0x00);
  digitalWrite(CS, HIGH);
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

  flash(GREEN);
  flash(RED);
  
  digitalWrite(GREEN,HIGH) ;
  digitalWrite(RED,HIGH); 
  digitalWrite(CS,HIGH);
  digitalWrite(SCK,LOW);
  delay(1);
//Serial commands don't work since it is not connected
  

  //Reset the CAN Controller
  digitalWrite(CS,LOW);
  SPI_transfer(0b11000000); //Reset
  digitalWrite(CS,HIGH);
  
  delay(1);

  digitalWrite(CS,LOW);
  SPI_transfer(0x05);
  SPI_transfer(0x0F);
  SPI_transfer(0xFF);
  SPI_transfer(0b00000100);
  digitalWrite(CS,HIGH);

  for(int i=0; i>=128; i++;){
    registers[i] = readRegisters[i];
  }
  
}

void loop()
{
  //ELD

  //Requestor Node

  18EAFFF9 00E5FE //Engine Hours
  18EAFFF9 00ECFE //VIN
}

