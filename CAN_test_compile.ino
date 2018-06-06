#include <mcp_can.h>
#include <mcp_can_dfs.h>

#define GREEN 10
#define RED 14
#define CS 6 //The chip select pin for SPI
#define SCK 7
#define DO 9 
#define DI 8
#define SILENT 11

MCP_CAN CAN0(6);



void flash(uint8_t led){
  digitalWrite(led, HIGH);
  delay(50);
  digitalWrite(led, LOW);
  delay(50);
  digitalWrite(led, HIGH);
  delay(50);
  digitalWrite(led, LOW);
  delay(50);
  digitalWrite(led, HIGH);
  delay(50);
  digitalWrite(led, LOW); 
}

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
  delay(100);
 flash(RED);
 flash(GREEN);

if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ)== CAN_OK){
 flash(GREEN);
 }
 else{
  flash(RED);
 }

  CAN0.setMode(MCP_NORMAL);  
}
void loop()
{
  byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
  byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);
  if(sndStat == CAN_OK){
    flash(GREEN);
  } else{
    flash(RED);
  }
}
