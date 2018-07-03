#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <util/atomic.h>


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
  pinMode(5, INPUT);
  
  digitalWrite(GREEN,HIGH) ;
  digitalWrite(RED,HIGH); 
  digitalWrite(CS,HIGH);
  digitalWrite(SCK,LOW);
  delay(100);

  //Reset the CAN Controller
  digitalWrite(CS,LOW);
  SPI_transfer(RESET); //Reset
  digitalWrite(CS,HIGH);
  
  delay(10);
  

  digitalWrite(CS,LOW);
  SPI_transfer(BIT_MODIFY); // instruction to modify bits
  SPI_transfer(0x0F); //Address Byte
  SPI_transfer(0xFF); //mask byte
  SPI_transfer(0b10000100); //data byte to enable 1 clock out and to enter configuration mode
  digitalWrite(CS,HIGH);

  delay(10);  

 flash(GREEN);
 flash(RED);

 digitalWrite(RED, HIGH);
// delay(100);
 digitalWrite(RED, LOW);
 
while(canTest){
if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ)==CAN_OK)
{
  digitalWrite(GREEN, HIGH);
  canTest = 0;
 }  

delay(100); 
}

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
    
    CAN0.sendMsgBuf(0x000, 0, 8, data);
    digitalWrite(RED, HIGH);
    delay(1);
    digitalWrite(RED, LOW);
  }

