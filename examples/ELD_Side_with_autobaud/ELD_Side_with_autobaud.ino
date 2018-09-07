#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <util/atomic.h>
#include <EEPROM.h>


#define GREEN 10
#define RED 14
#define CS 6 //The chip select pin for SPI
#define SCK 7
#define DO 9 
#define DI 8
#define SILENT 11

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];  

byte tick = 0;

#define RESET       0b11000000
#define READ_STATUS 0b10100000
#define BIT_MODIFY  0b00000101
#define READ        0b00000011

MCP_CAN CAN0(CS);

byte data[8] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0F};

uint8_t can_Val = EEPROM.read(0x00);

uint8_t canSpeed[5] = {12, 13, 10, 14, 15};

//***************************************************************************

uint8_t SPI_transfer(uint8_t data){
  uint8_t data_in = 0;
  DDRB = 0b11111101;
  //Use bitwise ORs and ANDs to set and clear bits directly on the port.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for (int8_t i = 0;i<8;i++){
      
      //Check to see what the first bit is in the data
      if ((data & 0x80) == 0x80) PORTB |= 0b00000001; //Set the bit on PB0
      else PORTB &= 0b11111110; //clear the bit on PB0
      PORTB |= 0b00000100; //Clock out the data on PB2
      data <<= 1;//tick++;
      data_in <<=1;
      data_in |= (PINB & 0x02) >> 1;        //(PORTB & 0x02) << 1; //Bitwise OR with PB1 to build the input
      
      //if(i<7) 
      PORTB &= 0b11111011;       //Set the ClockPin Low (PB2)     
      //tick++;
    }
    tick++;
  }    
  return data_in;    
}
bool canTest = 1;

uint32_t currentMillis = elapsedMillis();



uint8_t readREC(){
  uint8_t ret;
  digitalWrite(CS, LOW);
  SPI_transfer(0x03); // READ instruction
  SPI_transfer(0x1D);
  ret = SPI_transfer(0x00);
  digitalWrite(CS, HIGH);

  return ret;
}

void mcp2515_setRegister(uint8_t address, uint8_t value){
    digitalWrite(CS, LOW);
    SPI_transfer(0x02); //Write command
    SPI_transfer(address);
    SPI_transfer(value);
    digitalWrite(CS, HIGH);
}

uint8_t config_rate(uint8_t canSpeed){
  
  uint8_t sets, cfg1, cfg2, cfg3;
  sets = 1;
  switch (canSpeed) 
        {
            case (0x10):                                             // 125Kbps
            cfg1 = 0x43;
            cfg2 = 0xE5;
            cfg3 = 0x83;
            break;

            case (0x12):                                             // 250Kbps
            cfg1 = 0x41;
            cfg2 = 0xE5;
            cfg3 = 0x83;
            break;

            case (0x13):                                             // 500Kbps
            cfg1 = 0x40;
            cfg2 = 0xE5;
            cfg3 = 0x83;
            break;
      
            case (0x14):                                             // 600Kbps
            cfg1 = 0x40;
            cfg2 = 0xE1;
            cfg3 = 0x83;
            break;
        
            case (0x15):                                            //   1Mbps
            cfg1 = 0x00;
            cfg2 = 0xCA;
            cfg3 = 0x81;
            break;  

            default:
            sets = 0;
            return 1;
            break;
        } 

    
   if(sets){
    digitalWrite(CS,LOW);
    SPI_transfer(BIT_MODIFY); // instruction to modify bits
    SPI_transfer(0x0F); //Address Byte
    SPI_transfer(0xE0); //mask byte
    SPI_transfer(0b10000000); //data byte to enable 1 clock out and to enter configuration mode
    digitalWrite(CS,HIGH);
    delay(100);
    
    mcp2515_setRegister(0x2A, cfg1);
    mcp2515_setRegister(0x29, cfg2);
    mcp2515_setRegister(0x28, cfg3);

    digitalWrite(CS,LOW);
    SPI_transfer(BIT_MODIFY); // instruction to modify bits
    SPI_transfer(0x0F); //Address Byte
    SPI_transfer(0xE0); //mask byte
    SPI_transfer(0b00000000); //data byte to enable 1 clock out and to enter normal mode again
    digitalWrite(CS,HIGH);
    delay(100);

    return 0;
   }
   return 1;
}

uint8_t autobaud(){
  
  uint32_t previousMillis100 = currentMillis;
  int j = 0;
  int lastREC = readREC();
  
  while(true){
    //
    while((currentMillis - previousMillis100)<= 100){
      
    if(CAN0.checkReceive()==3){ //Checking to see if canBus frames have come onto the bus
      return canSpeed[j];
    }
    if(lastREC <= readREC()){
      lastREC = readREC();
      break;
    }
  }
    previousMillis100 = currentMillis;
    j++;
    if(j>=5){
      j=0;
    }
  config_rate(canSpeed[j]);
  }
}

void flash(uint8_t led){
  digitalWrite(led, HIGH);
  delay(10);
  digitalWrite(led, LOW);
}
void error(uint8_t led, uint8_t error){
  for(int i=0; i<=error; ++i){
    flash(led);
    delay(20);
    }
  }

void sendFail(uint8_t led){
  delay(25);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led, LOW);
}

void sos(uint8_t led){
  digitalWrite(led, LOW);
  digitalWrite(led, HIGH);
  delay(25);
  digitalWrite(led,LOW);
  delay(50);
  digitalWrite(led, HIGH);
  delay(25);
  digitalWrite(led,LOW);
  delay(50);
  digitalWrite(led, HIGH);
  delay(25);
  digitalWrite(led,LOW);
  delay(50);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led, LOW);
  delay(50);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led, LOW);
  delay(50);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led, LOW);
  delay(50);
  digitalWrite(led, HIGH);
  delay(25);
  digitalWrite(led,LOW);
  delay(50);
  digitalWrite(led, HIGH);
  delay(25);
  digitalWrite(led,LOW);
  delay(50);
  digitalWrite(led, HIGH);
  delay(25);
  digitalWrite(led,LOW);
  
}

void setup() {
  // set the digital pin as output:
  delay(5);
  
  pinMode(CS,OUTPUT);
  pinMode(SCK,OUTPUT);
  pinMode(DI,INPUT_PULLUP);
  pinMode(DO,OUTPUT);
  pinMode(GREEN,OUTPUT);
  pinMode(RED,OUTPUT);
  pinMode(SILENT, OUTPUT);
  pinMode(4, INPUT);
  
  digitalWrite(GREEN,HIGH) ;
  digitalWrite(RED,HIGH); 
  digitalWrite(CS,HIGH);
  digitalWrite(SCK,LOW);
  delay(1000);

  //Reset the CAN Controller
  digitalWrite(CS,LOW);
  SPI_transfer(RESET); //Reset
  digitalWrite(CS,HIGH);
  
  delay(10);

 // determining the right clkout divisor

  digitalWrite(CS,LOW);
  SPI_transfer(BIT_MODIFY); // instruction to modify bits
  SPI_transfer(0x0F); //Address Byte
  SPI_transfer(0xFF); //mask byte
  SPI_transfer(0b10000111); //data byte to enable 1 clock out and to enter configuration mode
  digitalWrite(CS,HIGH);

  delay(100);  

  digitalWrite(CS,LOW);
  SPI_transfer(BIT_MODIFY); // instruction to modify bits
  SPI_transfer(0x0F); //Address Byte
  SPI_transfer(0xFF); //mask byte
  SPI_transfer(0b10000110); //data byte to enable 1 clock out and to enter configuration mode
  digitalWrite(CS,HIGH);

  delay(100);

  digitalWrite(CS,LOW);
  SPI_transfer(BIT_MODIFY); // instruction to modify bits
  SPI_transfer(0x0F); //Address Byte
  SPI_transfer(0xFF); //mask byte
  SPI_transfer(0b10000101); //data byte to enable 1 clock out and to enter configuration mode
  digitalWrite(CS,HIGH);

  delay(100);

  digitalWrite(CS,LOW);
  SPI_transfer(BIT_MODIFY); // instruction to modify bits
  SPI_transfer(0x0F); //Address Byte
  SPI_transfer(0xFF); //mask byte
  SPI_transfer(0b10000100); //data byte to enable 1 clock out and to enter configuration mode
  digitalWrite(CS,HIGH);

  delay(100);

 flash(GREEN);
 flash(RED);

 digitalWrite(RED, HIGH);
// delay(100);
 digitalWrite(RED, LOW);
 

if(CAN0.begin(MCP_ANY, can_Val, MCP_16MHZ)==CAN_OK) //can_Val is the EEPROM saved value. 
{
  uint8_t autobaudCan_Val = autobaud();
  if(autobaudCan_Val != can_Val){
    EEPROM.write(0x00, autobaudCan_Val);
  }
 }  

delay(100); 
}

void loop()
{ 
    /*CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if(rxId == 0x18FEF100){
       flash(RED);
    }*/
    //flash(RED);

     digitalWrite(CS,LOW);
     SPI_transfer(0x03); // instruction to modify bits
     SPI_transfer(0x0F); //Address Byte
     SPI_transfer(0x00);
     digitalWrite(CS,HIGH);
    
    CAN0.sendMsgBuf(0x111, 0, 8, data);
    digitalWrite(RED, HIGH);
    delay(1000);
    digitalWrite(RED, LOW);
  }

