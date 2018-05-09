//These are the pins used
#define GREEN 10
#define RED 14
#define CS 6 //The chip select pin for SPI
#define SCK 7
#define DO 9 
#define DI 8

//CAN Speeds
#define MCP_16MHz_1000kBPS_CFG1 (0x00)
#define MCP_16MHz_1000kBPS_CFG2 (0xD0)
#define MCP_16MHz_1000kBPS_CFG3 (0x82)

#define MCP_16MHz_666kBPS_CFG1 (0x00)
#define MCP_16MHz_666kBPS_CFG2 (0xA8)
#define MCP_16MHz_666kBPS_CFG3 (0x03)

#define MCP_16MHz_500kBPS_CFG1 (0x00)
#define MCP_16MHz_500kBPS_CFG2 (0xF0)
#define MCP_16MHz_500kBPS_CFG3 (0x86)

#define MCP_16MHz_250kBPS_CFG1 (0x41)
#define MCP_16MHz_250kBPS_CFG2 (0xF1)
#define MCP_16MHz_250kBPS_CFG3 (0x85)

#define MCP_16MHz_200kBPS_CFG1 (0x01)
#define MCP_16MHz_200kBPS_CFG2 (0xFA)
#define MCP_16MHz_200kBPS_CFG3 (0x87)

#define MCP_16MHz_125kBPS_CFG1 (0x03)
#define MCP_16MHz_125kBPS_CFG2 (0xF0)
#define MCP_16MHz_125kBPS_CFG3 (0x86)

#define MCP_16MHz_100kBPS_CFG1 (0x03)
#define MCP_16MHz_100kBPS_CFG2 (0xFA)
#define MCP_16MHz_100kBPS_CFG3 (0x87)

#define MCP_16MHz_80kBPS_CFG1 (0x03)
#define MCP_16MHz_80kBPS_CFG2 (0xFF)
#define MCP_16MHz_80kBPS_CFG3 (0x87)

#define MCP_16MHz_50kBPS_CFG1 (0x07)
#define MCP_16MHz_50kBPS_CFG2 (0xFA)
#define MCP_16MHz_50kBPS_CFG3 (0x87)

#define MCP_16MHz_40kBPS_CFG1 (0x07)
#define MCP_16MHz_40kBPS_CFG2 (0xFF)
#define MCP_16MHz_40kBPS_CFG3 (0x87)

#define MCP_16MHz_31k25BPS_CFG1 (0x0F)
#define MCP_16MHz_31k25BPS_CFG2 (0xF1)
#define MCP_16MHz_31k25BPS_CFG3 (0x85)

#define MCP_16MHz_20kBPS_CFG1 (0x0F)
#define MCP_16MHz_20kBPS_CFG2 (0xFF)
#define MCP_16MHz_20kBPS_CFG3 (0x87)

#define MCP_16MHz_10kBPS_CFG1 (0x1F)
#define MCP_16MHz_10kBPS_CFG2 (0xFF)
#define MCP_16MHz_10kBPS_CFG3 (0x87)

#define MCP_16MHz_5kBPS_CFG1 (0x3F)
#define MCP_16MHz_5kBPS_CFG2 (0xFF)
#define MCP_16MHz_5kBPS_CFG3 (0x87)
//end speeds

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

//CANCTRL Modes
#define MODE_NORMAL     0x00
#define MODE_SLEEP      0x20
#define MODE_LOOPBACK   0x40
#define MODE_LISTENONLY 0x60
#define MODE_CONFIG     0x80
#define MODE_POWERUP    0xE0
#define MODE_MASK       0xE0
#define ABORT_TX        0x10
#define MODE_ONESHOT    0x08
#define CLKOUT_ENABLE   0x04
#define CLKOUT_DISABLE  0x00
#define CLKOUT_PS1      0x00
#define CLKOUT_PS2      0x01
#define CLKOUT_PS4      0x02
#define CLKOUT_PS8      0x03
//End of controller modes


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

void mcp25625_reset(){                                        //reset the CAN controller
   digitalWrite(CS, LOW); 
   SPI_transfer(0b11000000);                                  //reset command
   digitalWrite(CS, HIGH);
   delay(10);
}

void mcp25625_setCANCTRL_Mode(uint8_t newmode){
  uint8_t i;
  mcp25625_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);
  i = mcp2515_readRegister(MCP_CANCTRL);
  //need to determine a checking system that does not rely on Serial Com. 
}

void mcp25625_configRate(uint8_t canSpeed){
  uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canSpeed) 
    {
        case (CAN_5KBPS):
        cfg1 = MCP_16MHz_5kBPS_CFG1;
        cfg2 = MCP_16MHz_5kBPS_CFG2;
        cfg3 = MCP_16MHz_5kBPS_CFG3;
        break;

        case (CAN_10KBPS):
        cfg1 = MCP_16MHz_10kBPS_CFG1;
        cfg2 = MCP_16MHz_10kBPS_CFG2;
        cfg3 = MCP_16MHz_10kBPS_CFG3;
        break;

        case (CAN_20KBPS):
        cfg1 = MCP_16MHz_20kBPS_CFG1;
        cfg2 = MCP_16MHz_20kBPS_CFG2;
        cfg3 = MCP_16MHz_20kBPS_CFG3;
        break;
        
        case (CAN_31K25BPS):
        cfg1 = MCP_16MHz_31k25BPS_CFG1;
        cfg2 = MCP_16MHz_31k25BPS_CFG2;
        cfg3 = MCP_16MHz_31k25BPS_CFG3;
        break;

        case (CAN_40KBPS):
        cfg1 = MCP_16MHz_40kBPS_CFG1;
        cfg2 = MCP_16MHz_40kBPS_CFG2;
        cfg3 = MCP_16MHz_40kBPS_CFG3;
        break;

        case (CAN_50KBPS):
        cfg1 = MCP_16MHz_50kBPS_CFG1;
        cfg2 = MCP_16MHz_50kBPS_CFG2;
        cfg3 = MCP_16MHz_50kBPS_CFG3;
        break;

        case (CAN_80KBPS):
        cfg1 = MCP_16MHz_80kBPS_CFG1;
        cfg2 = MCP_16MHz_80kBPS_CFG2;
        cfg3 = MCP_16MHz_80kBPS_CFG3;
        break;

        case (CAN_100KBPS):                                             /* 100KBPS                  */
        cfg1 = MCP_16MHz_100kBPS_CFG1;
        cfg2 = MCP_16MHz_100kBPS_CFG2;
        cfg3 = MCP_16MHz_100kBPS_CFG3;
        break;

        case (CAN_125KBPS):
        cfg1 = MCP_16MHz_125kBPS_CFG1;
        cfg2 = MCP_16MHz_125kBPS_CFG2;
        cfg3 = MCP_16MHz_125kBPS_CFG3;
        break;

        case (CAN_200KBPS):
        cfg1 = MCP_16MHz_200kBPS_CFG1;
        cfg2 = MCP_16MHz_200kBPS_CFG2;
        cfg3 = MCP_16MHz_200kBPS_CFG3;
        break;

        case (CAN_250KBPS):
        cfg1 = MCP_16MHz_250kBPS_CFG1;
        cfg2 = MCP_16MHz_250kBPS_CFG2;
        cfg3 = MCP_16MHz_250kBPS_CFG3;
        break;

        case (CAN_500KBPS):
        cfg1 = MCP_16MHz_500kBPS_CFG1;
        cfg2 = MCP_16MHz_500kBPS_CFG2;
        cfg3 = MCP_16MHz_500kBPS_CFG3;
        break;
        
        case (CAN_666KBPS):
        cfg1 = MCP_16MHz_666kBPS_CFG1;
        cfg2 = MCP_16MHz_666kBPS_CFG2;
        cfg3 = MCP_16MHz_666kBPS_CFG3;
        break;
        
        case (CAN_1000KBPS):
        cfg1 = MCP_16MHz_1000kBPS_CFG1;
        cfg2 = MCP_16MHz_1000kBPS_CFG2;
        cfg3 = MCP_16MHz_1000kBPS_CFG3;
        break;  

        default:
        set = 0;
        break;
    }

    if (set) {
        mcp2515_setRegister(MCP_CNF1, cfg1);
        mcp2515_setRegister(MCP_CNF2, cfg2);
        mcp2515_setRegister(MCP_CNF3, cfg3);
}

void mcp25625_initCANBuffers(){
  //initialize the CAN buffers
}

void mcp25625_setRegisters(args){
  //copy this over. 
}

void mcp25625_modifyRegister(args){
  //copy this over too. 
}
void mcp25625_init(uint8_t canSpeed){
    mcp25625_reset();
    mcp25625_setCANCTRL_Mode(MODE_CONFIG);
    mcp25625_configRate(canSpeed))
    mcp25625_initCANBuffers();
    mcp25625_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF)
    mcp25625_modifyRegister(MCP_RXB0CTRL, MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK, MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK );
    mcp25625_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK, MCP_RXB_RX_STDEXT);
    mcp25625_setCANCTRL_Mode(MODE_NORMAL);                                                                
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

  

  //Reset the CAN Controller
  mcp25625_reset()
  

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

