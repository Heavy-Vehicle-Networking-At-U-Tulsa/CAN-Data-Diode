/*
Set MCP25625 PRescaler
https://github.com/SpenceKonde/ATTinyCore/blob/master/avr/extras/ATtiny_x61.md
 */

//These are the pins used
#define GREEN 10
#define RED 14
#define CS 6 //The chip select pin for SPI
#define SCK 7
#define DO 9 
#define DI 8

//registers
#define MCP_RXF0SIDH    0x00
#define MCP_RXF0SIDL    0x01
#define MCP_RXF0EID8    0x02
#define MCP_RXF0EID0    0x03
#define MCP_RXF1SIDH    0x04
#define MCP_RXF1SIDL    0x05
#define MCP_RXF1EID8    0x06
#define MCP_RXF1EID0    0x07
#define MCP_RXF2SIDH    0x08
#define MCP_RXF2SIDL    0x09
#define MCP_RXF2EID8    0x0A
#define MCP_RXF2EID0    0x0B
#define MCP_CANSTAT     0x0E
#define MCP_CANCTRL     0x0F
#define MCP_RXF3SIDH    0x10
#define MCP_RXF3SIDL    0x11
#define MCP_RXF3EID8    0x12
#define MCP_RXF3EID0    0x13
#define MCP_RXF4SIDH    0x14
#define MCP_RXF4SIDL    0x15
#define MCP_RXF4EID8    0x16
#define MCP_RXF4EID0    0x17
#define MCP_RXF5SIDH    0x18
#define MCP_RXF5SIDL    0x19
#define MCP_RXF5EID8    0x1A
#define MCP_RXF5EID0    0x1B
#define MCP_TEC         0x1C
#define MCP_REC         0x1D
#define MCP_RXM0SIDH    0x20
#define MCP_RXM0SIDL    0x21
#define MCP_RXM0EID8    0x22
#define MCP_RXM0EID0    0x23
#define MCP_RXM1SIDH    0x24
#define MCP_RXM1SIDL    0x25
#define MCP_RXM1EID8    0x26
#define MCP_RXM1EID0    0x27
#define MCP_CNF3        0x28
#define MCP_CNF2        0x29
#define MCP_CNF1        0x2A
#define MCP_CANINTE     0x2B
#define MCP_CANINTF     0x2C
#define MCP_EFLG        0x2D
#define MCP_TXB0CTRL    0x30
#define MCP_TXB1CTRL    0x40
#define MCP_TXB2CTRL    0x50
#define MCP_RXB0CTRL    0x60
#define MCP_RXB0SIDH    0x61
#define MCP_RXB1CTRL    0x70
#define MCP_RXB1SIDH    0x71
//End of register mapping

//Command instructions
#define MCP_WRITE           0x02
#define MCP_READ            0x03
#define MCP_BITMOD          0x05
#define MCP_LOAD_TX0        0x40
#define MCP_LOAD_TX1        0x42
#define MCP_LOAD_TX2        0x44
#define MCP_RTS_TX0         0x81
#define MCP_RTS_TX1         0x82
#define MCP_RTS_TX2         0x84
#define MCP_RTS_ALL         0x87
#define MCP_READ_RX0        0x90
#define MCP_READ_RX1        0x94
#define MCP_READ_STATUS     0xA0
#define MCP_RX_STATUS       0xB0
#define MCP_RESET           0xC0
//End of commands

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

