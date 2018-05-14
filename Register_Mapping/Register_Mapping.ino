#include <util/atomic.h>
//These are the pins used
#define GREEN 10
#define RED 14
#define CS 6 //The chip select pin for SPI
#define SCK 7
#define DO 9 
#define DI 8
#define SILENT 11

/*
 *   Begin mt
 */
#define TIMEOUTVALUE    50
#define MCP_SIDH        0
#define MCP_SIDL        1
#define MCP_EID8        2
#define MCP_EID0        3

#define MCP_TXB_EXIDE_M     0x08                                        /* In TXBnSIDL                  */
#define MCP_DLC_MASK        0x0F                                        /* 4 LSBits                     */
#define MCP_RTR_MASK        0x40                                        /* (1<<6) Bit 6                 */

#define MCP_RXB_RX_ANY      0x60
#define MCP_RXB_RX_EXT      0x40
#define MCP_RXB_RX_STD      0x20
#define MCP_RXB_RX_STDEXT   0x00
#define MCP_RXB_RX_MASK     0x60
#define MCP_RXB_BUKT_MASK   (1<<2)

/*
** Bits in the TXBnCTRL registers.
*/
#define MCP_TXB_TXBUFE_M    0x80
#define MCP_TXB_ABTF_M      0x40
#define MCP_TXB_MLOA_M      0x20
#define MCP_TXB_TXERR_M     0x10
#define MCP_TXB_TXREQ_M     0x08
#define MCP_TXB_TXIE_M      0x04
#define MCP_TXB_TXP10_M     0x03

#define MCP_TXB_RTR_M       0x40                                        /* In TXBnDLC                   */
#define MCP_RXB_IDE_M       0x08                                        /* In RXBnSIDL                  */
#define MCP_RXB_RTR_M       0x40                                        /* In RXBnDLC                   */

#define MCP_STAT_RXIF_MASK   (0x03)
#define MCP_STAT_RX0IF (1<<0)
#define MCP_STAT_RX1IF (1<<1)

#define MCP_EFLG_RX1OVR (1<<7)
#define MCP_EFLG_RX0OVR (1<<6)
#define MCP_EFLG_TXBO   (1<<5)
#define MCP_EFLG_TXEP   (1<<4)
#define MCP_EFLG_RXEP   (1<<3)
#define MCP_EFLG_TXWAR  (1<<2)
#define MCP_EFLG_RXWAR  (1<<1)
#define MCP_EFLG_EWARN  (1<<0)
#define MCP_EFLG_ERRORMASK  (0xF8)                                      /* 5 MS-Bits                    */


/*
 *   Define MCP2515 register addresses
 */

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
#define MCP_TEC            0x1C
#define MCP_REC            0x1D
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
#define MCP_CANINTE        0x2B
#define MCP_CANINTF        0x2C
#define MCP_EFLG        0x2D
#define MCP_TXB0CTRL    0x30
#define MCP_TXB1CTRL    0x40
#define MCP_TXB2CTRL    0x50
#define MCP_RXB0CTRL    0x60
#define MCP_RXB0SIDH    0x61
#define MCP_RXB1CTRL    0x70
#define MCP_RXB1SIDH    0x71


#define MCP_TX_INT          0x1C                                    // Enable all transmit interrup ts
#define MCP_TX01_INT        0x0C                                    // Enable TXB0 and TXB1 interru pts
#define MCP_RX_INT          0x03                                    // Enable receive interrupts
#define MCP_NO_INT          0x00                                    // Disable all interrupts

#define MCP_TX01_MASK       0x14
#define MCP_TX_MASK        0x54

/*
 *   Define SPI Instruction Set
 */

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


/*
 *   CANCTRL Register Values
 */

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


/*
 *   CNF1 Register Values
 */

#define SJW1            0x00
#define SJW2            0x40
#define SJW3            0x80
#define SJW4            0xC0


/*
 *   CNF2 Register Values
 */

#define BTLMODE         0x80
#define SAMPLE_1X       0x00
#define SAMPLE_3X       0x40


/*
 *   CNF3 Register Values
 */

#define SOF_ENABLE      0x80
#define SOF_DISABLE     0x00
#define WAKFIL_ENABLE   0x40
#define WAKFIL_DISABLE  0x00


/*
 *   CANINTF Register Bits
 */

#define MCP_RX0IF       0x01
#define MCP_RX1IF       0x02
#define MCP_TX0IF       0x04
#define MCP_TX1IF       0x08
#define MCP_TX2IF       0x10
#define MCP_ERRIF       0x20
#define MCP_WAKIF       0x40
#define MCP_MERRF       0x80

/*
 *  speed 16M
 */
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

#define MCPDEBUG        (0)
#define MCPDEBUG_TXBUF  (0)
#define MCP_N_TXBUFFERS (3)

#define MCP_RXBUF_0 (MCP_RXB0SIDH)
#define MCP_RXBUF_1 (MCP_RXB1SIDH)

//#define CS 10
#define MCP2515_SELECT()   digitalWrite(CS, LOW)
#define MCP2515_UNSELECT() digitalWrite(CS, HIGH)

#define MCP25625_OK         (0)
#define MCP25625_FAIL       (1)
#define MCP_ALLTXBUSY      (2)

#define CANDEBUG   1

#define CANUSELOOP 0

#define CANSENDTIMEOUT (200)                                            /* milliseconds                 */

/*
 *   initial value of gCANAutoProcess
 */
#define CANAUTOPROCESS (1)
#define CANAUTOON  (1)
#define CANAUTOOFF (0)

#define CAN_STDID (0)
#define CAN_EXTID (1)

#define CANDEFAULTIDENT    (0x55CC)
#define CANDEFAULTIDENTEXT (CAN_EXTID)

#define CAN_5KBPS    1
#define CAN_10KBPS   2
#define CAN_20KBPS   3
#define CAN_31K25BPS 4
#define CAN_40KBPS   5
#define CAN_50KBPS   6
#define CAN_80KBPS   7
#define CAN_100KBPS  8
#define CAN_125KBPS  9
#define CAN_200KBPS  10
#define CAN_250KBPS  11
#define CAN_500KBPS  12
#define CAN_1000KBPS 13
#define CAN_666KBPS  14

#define CAN_OK         (0)
#define CAN_FAILINIT   (1)
#define CAN_FAILTX     (2)
#define CAN_MSGAVAIL   (3)
#define CAN_NOMSG      (4)
#define CAN_CTRLERROR  (5)
#define CAN_GETTXBFTIMEOUT (6)
#define CAN_SENDMSGTIMEOUT (7)
#define CAN_FAIL       (0xff)

#define CAN_MAX_CHAR_IN_MESSAGE (8)

//End of controller modes

uint8_t m_nExtFlg;
uint32_t m_nID;
uint8_t m_nDta[8];
uint8_t m_nRtr;
uint8_t m_nDlc; 
uint8_t m_nfilhit;

uint8_t counter = 0;

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

void mcp25625_reset(){                                        //reset the CAN controller
   digitalWrite(CS, LOW); 
   SPI_transfer(0b11000000);                                  //reset command
   digitalWrite(CS, HIGH);
   delay(10);
}

void mcp25625_modifyRegister(uint8_t address, uint8_t mask, uint8_t data){
    digitalWrite(CS, LOW);
    SPI_transfer(MCP_BITMOD);
    SPI_transfer(address);
    SPI_transfer(mask);
    SPI_transfer(data);
    digitalWrite(CS, HIGH);
}

void mcp25625_setCANCTRL_Mode(uint8_t newmode){
  uint8_t i;
  mcp25625_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode); 
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
        mcp25625_setRegister(MCP_CNF1, cfg1);
        mcp25625_setRegister(MCP_CNF2, cfg2);
        mcp25625_setRegister(MCP_CNF3, cfg3);
}
}

void mcp25625_setRegister(uint8_t address, uint8_t values){
    digitalWrite(CS, LOW);
    SPI_transfer(MCP_WRITE);
    SPI_transfer(address);
    SPI_transfer(values);
    digitalWrite(CS, HIGH);
}

void mcp25625_setRegisterS(uint8_t address, uint8_t values[], uint8_t n){
    
    uint8_t i;
    digitalWrite(CS, LOW);
    SPI_transfer(MCP_WRITE);
    SPI_transfer(address);
    for (i=0; i<n; i++) 
    {
        SPI_transfer(values[i]);
    }
    digitalWrite(CS, HIGH);
}

void mcp25625_write_id( uint8_t mcp_addr,uint8_t ext, uint32_t id ){
    uint16_t canid;
    uint8_t tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) 
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 3 );
        tbufdata[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    mcp25625_setRegisterS( mcp_addr, tbufdata, 4 );
}


void mcp25625_initCANBuffers(){
  uint8_t i, a1, a2, a3;
    
    uint8_t std = 0;               
    uint8_t ext = 1;
    uint32_t ulMask = 0x00, ulFilt = 0x00;

    mcp25625_write_id(MCP_RXM0SIDH, ext, ulMask);      /*Set both masks to 0           */
    mcp25625_write_id(MCP_RXM1SIDH, ext, ulMask);      /*Mask register ignores ext bit */
    
                                                       /* Set all filters to 0         */
    mcp25625_write_id(MCP_RXF0SIDH, ext, ulFilt);      /* RXB0: extended               */
    mcp25625_write_id(MCP_RXF1SIDH, std, ulFilt);      /* RXB1: standard               */
    mcp25625_write_id(MCP_RXF2SIDH, ext, ulFilt);      /* RXB2: extended               */
    mcp25625_write_id(MCP_RXF3SIDH, std, ulFilt);      /* RXB3: standard               */
    mcp25625_write_id(MCP_RXF4SIDH, ext, ulFilt);
    mcp25625_write_id(MCP_RXF5SIDH, std, ulFilt);
    //makes sure that nothing is preset in the registers to be sent out. 
    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++) {                         /* in-buffer loop               */
        mcp25625_setRegister(a1, 0);
        mcp25625_setRegister(a2, 0);
        mcp25625_setRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
    mcp25625_setRegister(MCP_RXB0CTRL, 0);
    mcp25625_setRegister(MCP_RXB1CTRL, 0);
}


uint8_t mcp25625_readStatus(void){
    uint8_t i;
    digitalWrite(CS, LOW);
    SPI_transfer(MCP_READ_STATUS);
    i = shiftIn(DI, SCK, LSBFIRST);
    digitalWrite(CS, HIGH);
  
    return i;
}


void mcp25625_read_id(uint8_t mcp_addr, uint8_t *ext, uint32_t *id){
    uint8_t tbufdata[4];
    
    ext = 0;
    id = 0; 

    mcp25625_readRegisterS(mcp_addr, tbufdata, 4);
    *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M ) 
    {
                                                                        /* extended id                  */
        *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id<<8) + tbufdata[MCP_EID8];
        *id = (*id<<8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

void mcp25625_write_canMsg(uint8_t buffer_sidh_addr){
    uint8_t mcp_addr;
    mcp_addr = buffer_sidh_addr;
    mcp25625_setRegisterS(mcp_addr+5, m_nDta, m_nDlc );                  /* write data bytes             */
    if ( m_nRtr == 1){
        m_nDlc |= MCP_RTR_MASK;  
    }
    mcp25625_setRegister((mcp_addr+4), m_nDlc );                        /* write the RTR and DLC        */
    mcp25625_write_id(mcp_addr, m_nExtFlg, m_nID );
}

void mcp25625_read_canMsg(uint8_t buffer_sidh_addr){
    uint8_t mcp_addr, ctrl;
    mcp_addr = buffer_sidh_addr;
    mcp25625_read_id( mcp_addr, &m_nExtFlg, &m_nID );
    ctrl = mcp25625_readRegister( mcp_addr-1 );
    m_nDlc = mcp25625_readRegister( mcp_addr+4 );
    if ((ctrl & 0x08)) {
        m_nRtr = 1;
    }
    else {
        m_nRtr = 0;
    }

    m_nDlc &= MCP_DLC_MASK;
    mcp25625_readRegisterS( mcp_addr+5, &(m_nDta[0]), m_nDlc );
}
void mcp25625_start_transmit(uint8_t mcp_addr){
    mcp25625_modifyRegister( mcp_addr-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );
}
uint8_t mcp25625_getNextFreeTXBuf(uint8_t *txbuf_n)                 /* get Next free txbuf          */
{
    uint8_t res, i, ctrlval;
    uint8_t ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

    res = MCP_ALLTXBUSY;
    txbuf_n = 0x00;

                                                                        /* check all 3 TX-Buffers       */
    for (i=0; i<MCP_N_TXBUFFERS; i++) {
        ctrlval = mcp25625_readRegister( ctrlregs[i] );
        if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 ) {
            *txbuf_n = ctrlregs[i]+1;                                   /* return SIDH-address of Buffe */
                                                                        /* r                            */
            res = MCP25625_OK;
            return res;                                                 /* ! function exit              */
        }
    }
    return res;
}

void mcp25625_begin(uint8_t speedset){
  mcp25625_init(speedset);
}

void init_Mask(uint8_t num, uint8_t ext, uint32_t ulData)
{
    uint8_t res = MCP25625_OK;
    mcp25625_setCANCTRL_Mode(MODE_CONFIG);    
    if (num == 0){
        mcp25625_write_id(MCP_RXM0SIDH, ext, ulData);
    }
    else if(num == 1){
        mcp25625_write_id(MCP_RXM1SIDH, ext, ulData);
    }
    mcp25625_setCANCTRL_Mode(MODE_NORMAL);
   
}

void init_Filt(uint8_t num, uint8_t ext, uint32_t ulData)
{
    uint8_t res = MCP25625_OK;
    mcp25625_setCANCTRL_Mode(MODE_CONFIG);
    
    switch( num )
    {
        case 0:
        mcp25625_write_id(MCP_RXF0SIDH, ext, ulData);
        break;

        case 1:
        mcp25625_write_id(MCP_RXF1SIDH, ext, ulData);
        break;

        case 2:
        mcp25625_write_id(MCP_RXF2SIDH, ext, ulData);
        break;

        case 3:
        mcp25625_write_id(MCP_RXF3SIDH, ext, ulData);
        break;

        case 4:
        mcp25625_write_id(MCP_RXF4SIDH, ext, ulData);
        break;

        case 5:
        mcp25625_write_id(MCP_RXF5SIDH, ext, ulData);
        break;

        default:
        res = MCP25625_FAIL;
    }
    
    mcp25625_setCANCTRL_Mode(MODE_NORMAL);
}

void setMsg(uint32_t id, uint8_t ext, uint8_t len, uint8_t *pData){
  int i = 0;
  m_nExtFlg = ext;
  m_nID     = id;
  m_nDlc    = len;
  for(i = 0; i<len; i++){
    m_nDta[i] = *(pData+i);
  }
}

void clearMsg(){
    m_nID       = 0;
    m_nDlc      = 0;
    m_nExtFlg   = 0;
    m_nRtr      = 0;
    m_nfilhit   = 0;
    for(int i = 0; i<m_nDlc; i++ )
      m_nDta[i] = 0x00;
}

uint8_t sendMsg(){
    uint8_t res, res1, txbuf_n;
    uint16_t uiTimeOut = 0;

    do {
        res = mcp25625_getNextFreeTXBuf(&txbuf_n);                       /* info = addr.                 */
        uiTimeOut++;
    } 
    while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

    if(uiTimeOut == TIMEOUTVALUE) 
    {   
        return CAN_GETTXBFTIMEOUT;                                      /* get tx buff time out         */
    }
    uiTimeOut = 0;
    mcp25625_write_canMsg( txbuf_n);
    mcp25625_start_transmit( txbuf_n );
    do
    {
        uiTimeOut++;        
        res1= mcp25625_readRegister(txbuf_n);                        /* read send buff ctrl reg  */
        res1 = res1 & 0x08;                                   
    }while(res1 && (uiTimeOut < TIMEOUTVALUE));   
    if(uiTimeOut == TIMEOUTVALUE)                                       /* send msg timeout             */  
    {
        return CAN_SENDMSGTIMEOUT;
    }
    return CAN_OK;

}

uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    setMsg(id, ext, len, buf);
    sendMsg();
}

uint8_t readMsg(){
    uint8_t stat, res;
    stat = mcp25625_readStatus();
    if ( stat & MCP_STAT_RX0IF ){                                        /* Msg in Buffer 0              */

        mcp25625_read_canMsg( MCP_RXBUF_0);
        mcp25625_modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if ( stat & MCP_STAT_RX1IF )                                   /* Msg in Buffer 1              */
    {
        mcp25625_read_canMsg( MCP_RXBUF_1);
        mcp25625_modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else 
    {
        res = CAN_NOMSG;
    }
    return res;
}
uint8_t readMsgBuf(uint8_t *len, uint8_t buf[]){
    
    readMsg();
    *len = m_nDlc;
    for(int i = 0; i<m_nDlc; i++)
    {
      buf[i] = m_nDta[i];
    }
}
uint8_t checkReceive()
{
    uint8_t res;
    res = mcp25625_readStatus();                                         /* RXnIF in Bit 1 and 0         */
    if ( res & MCP_STAT_RXIF_MASK ) 
    {
        return CAN_MSGAVAIL;
    }
    else 
    {
        return CAN_NOMSG;
    }
}

uint8_t checkError(){
  uint8_t eflg = mcp25625_readRegister(MCP_EFLG);

    if ( eflg & MCP_EFLG_ERRORMASK ) 
    {
        return CAN_CTRLERROR;
    }
    else 
    {
        return CAN_OK;
    }
}
uint32_t getCanId(){
    return m_nID;
}



void mcp25625_init(uint8_t canSpeed){
    mcp25625_reset();
    mcp25625_setCANCTRL_Mode(MODE_CONFIG);
    mcp25625_configRate(canSpeed);
    mcp25625_initCANBuffers();
    mcp25625_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);
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

void mcp25625_readRegisterS(uint8_t address, uint8_t values[], uint8_t n)
{
  uint8_t i;
  digitalWrite(CS, LOW);
  SPI_transfer(MCP_READ);
  SPI_transfer(address);
  // mcp2515 has auto-increment of address-pointer
  byte incoming = shiftIn(DI, SCK, LSBFIRST);
  digitalWrite(CS, HIGH);
}

byte mcp25625_readRegister(uint8_t address){
   byte incoming;
    digitalWrite(CS, LOW);
    SPI_transfer(MCP_READ);
    SPI_transfer(address);
    incoming = shiftIn(DI, SCK, LSBFIRST);
    digitalWrite(CS, HIGH);

  return incoming;
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

void setRegister(uint8_t address){
  digitalWrite(CS, LOW);
  SPI_transfer(0x02);
  SPI_transfer(address);
  SPI_transfer(0x00);
  digitalWrite(CS, HIGH);
}

byte vin[3] = {0x00, 0xE5, 0xFE};
byte hours[3] = {0x00, 0xE5, 0xFE};
uint32_t reqID = 0x18EAFFF9;


void setup() {

  mcp25625_reset();
  delay(10);
  mcp25625_begin(CAN_250KBPS);
  mcp25625_setCANCTRL_Mode(MODE_NORMAL);
  delay(1);
  
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
  mcp25625_reset();
  

  digitalWrite(CS,LOW);
  SPI_transfer(0x05);
  SPI_transfer(0x0F);
  SPI_transfer(0xFF);
  SPI_transfer(0b00000100);
  digitalWrite(CS,HIGH);

  for(int i=0; i>=128; i++){
    registers[i] = readRegister[i];
  }
  
}

void loop(){
  flash(GREEN);
  flash(GREEN);
  //uncomment whichever you are using
  
  /*//ELD
  if(96<=readRegister(0x1C)){
    digitalWrite(SILENT, LOW);
    delay(500);
    digitalWrite(SILENT, HIGH);
  }
  */
  
  //Requestor Node
  
  sendMsgBuf(reqID, 1, 3, vin);
  flash(GREEN);
  delay(150000);
  sendMsgBuf(reqID, 1, 3, hours);
  flash(GREEN);
  delay(150000);

}

