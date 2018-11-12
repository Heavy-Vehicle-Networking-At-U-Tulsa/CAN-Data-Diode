#include <CANDataDiode.h>
#include <CANDataDiode_dfs.h>
#include <util/atomic.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <EEPROM.h>
/*************************************************/

/* PIN Defines */ 
#define GREEN 10 // Pin for the Green LED
#define RED 14 // Red LED pin
#define CS 6 // The chip select pin for SPI
#define SCK 7 // Clock Source 
#define DO 9  // Data OUT
#define DI 8 // Data IN
#define SILENT 11 // Silent trigger pin for the transceiver
//Note: reference https://github.com/SpenceKonde/ATTinyCore#attiny-261461861 for configuring other pins

/* EEPROM Memory Map */
/* Note: there are 512 bytes of EEPROM on the attiny861 */
#define CAN_BAUDRATE  0x00 // Previously stored CAN_Bus Baudrate
#define CAN_MSGCOUNT  0x01 // Amount of messages to be sent
#define CAN_ID1LSB    0x02 // LSB for request 1
#define CAN_ID1MSB    0x03 // MSB for request 1
#define CAN_ID2LSB    0x04 // LSB for request 2
#define CAN_ID2MSB    0x05 // MSB for request 2
#define CAN_ID3LSB    0x06 // LSB for request 3
#define CAN_ID3MSB    0x07 // MSB for request 3
//0x08-0x20 are reserved for additional request message space. 
#define MAX_REC       0x21 // When to trigger the silent pin on the transceiver.
#define EFLG          0x22 // Use EFLG Register rather than the REC register for silent detection
#define WDT_TIME      0x23 // Configuration of the WDT time interval
#define WDT_CONF      0x24 // Does the user want to set up the device to reset in the event of a code hang up. 
#define CAN_ID1_int   0x25 // Rate at which CANID1 messages are sent (may be implemented as a method choice)
#define CAN_ID2_int   0x26 // Rate at which CANID2 messages are sent (may be implemented as a method choice)
#define CAN_ID3_int   0x27 // Rate at which CANID3 messages are sent (may be implemented as a method choice)
//0x28 - 0x33 are reserved for additional CAN intervals 
/* Note: there are 512 bytes of EEPROM on the attiny861 */

/* SPI Commands */
#define RESET       0b11000000
#define READ_STATUS 0b10100000
#define BIT_MODIFY  0b00000101
#define WRITE       0b00000010
#define READ        0b00000011

/* Check Receive Possible returns */
#define CAN_MSGAVAIL       3
#define CAN_NOMSG          4

/* SPI PIN low level defines */
#define PIN_PB0     0b00000001
#define PIN_PB2     0b00000100

/* MCP CANCTRL VALUES */
#define CONFIGURE_MODE_CLKOUT_1 0b10000100
#define CONFIGURE_MODE_CLKOUT_2 0b10000101
#define CONFIGURE_MODE_CLKOUT_4 0b10000110
#define CONFIGURE_MODE_CLKOUT_8 0b10000111

#define NORMAL_MODE_CLKOUT_1 0b00000100
#define NORMAL_MODE_CLKOUT_2 0b00000101
#define NORMAL_MODE_CLKOUT_4 0b00000110
#define NORMAL_MODE_CLKOUT_8 0b00000111

/* CAN SPEED VALUES */
#define CAN_250KBS  0x12
#define CAN_500KBS  0x13
#define CAN_125KBS  0x10
#define CAN_666KBS  0x14
#define CAN_1000KBS 0x15

/*CAN Baudrate Configuration values*/ 
#define CAN_125KBPS_cfg1 0x43
#define CAN_125KBPS_cfg2 0xE5
#define CAN_125KBPS_cfg3 0x83

#define CAN_250KBPS_cfg1 0x41
#define CAN_250KBPS_cfg2 0xE5
#define CAN_250KBPS_cfg3 0x83

#define CAN_500KBPS_cfg1 0x40
#define CAN_500KBPS_cfg2 0xE5
#define CAN_500KBPS_cfg3 0x83

#define CAN_666KBPS_cfg1 0x40
#define CAN_666KBPS_cfg2 0xE1
#define CAN_666KBPS_cfg3 0x83

#define CAN_1000KBPS_cfg1 0x00
#define CAN_1000KBPS_cfg2 0xCA
#define CAN_1000KBPS_cfg3 0x81

/* MCP25265 Registers */
#define REC         0x1D
#define CANCTRL     0x0F
#define CFG1_Reg    0x2A
#define CFG2_Reg    0x29
#define CFG3_Reg    0x28

/* Watchdog Timer Config */
#define WDT_8sec        0x21
#define WDT_4sec        0x20
#define WDT_2sec        0x07
#define WDT_1sec        0x06

/* WDT CONF Options */
#define WDT_INT         0x01
#define WDT_RESET       0x02
#define WDT_INT_RESET   0x03

/* ERROR FLAG MODE */
#define EFLAG_MODE_OFF     0x00

/* Register Masks */
#define first_bit     0b00000001
#define second_bit    0b00000010
#define third_bit     0b00000100
#define fourth_bit    0b00001000
#define fifth_bit     0b00010000
#define sixth_bit     0b00100000
#define seventh_bit   0b01000000
#define eighth_bit    0b10000000

MCP_CAN CAN0(CS); // passing the Chip Select to the MCP_CAN library

/* Watchdog Timer flag */
volatile int f_wdt=1;

/* Setting up CAN value */
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
uint8_t autobaudCan_Val =0;

long unsigned int tick = 0; //one clock signal.

/* These are the possible baudrate configurations */
uint8_t canSpeed[5] = {CAN_250KBS, CAN_500KBS, CAN_125KBS, CAN_666KBS, CAN_1000KBS};

/* Pulling EEPROM CONFIG SETTINGS */
uint8_t can_Val =         EEPROM.read(CAN_BAUDRATE); //Need to establish a check to ensure that the value here is actually a usable value. 
uint8_t EFLG_MODE =       EEPROM.read(EFLG);
uint8_t REC_TRIGGER =     EEPROM.read(MAX_REC);
uint8_t WDT_WAIT_TIME =   EEPROM.read(WDT_TIME);
uint8_t WDT_SETUP_CONF =  EEPROM.read(WDT_CONF);

/***************************************************
 *  Name:        ISR(WDT_vect)
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 ***************************************************/
ISR(WDT_vect){
  if(f_wdt == 0){
    f_wdt=1;
  }
}

/***************************************************
 *  Name:        enterSleep
 *  Description: Enters the arduino into sleep mode.
 ***************************************************/
void enterSleep(void){
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
  /* Re-enable the peripherals. */
  power_all_enable();
}

/******************************************************
 * Name:        SPI_transfer
 * Description: Bit-Banging the SPI-Bus
 *****************************************************/
uint8_t SPI_transfer(uint8_t data){
  uint8_t data_in = 0;
  DDRB = 0b11111101; // Setting up inputs and outputs for DDRB.
  
  //Use bitwise ORs and ANDs to set and clear bits directly on the port.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for (int8_t i = 0;i<8;i++){
      
      //Check to see what the first bit is in the data
      if ((data & 0x80) == 0x80) PORTB |= PIN_PB0; //Set the bit on PB0
      else PORTB &= ~PIN_PB0; //clear the bit on PB0
      
      PORTB |= PIN_PB2; //Clock out the data on PB2
      data <<= 1; // Setting up the next data bit. 
      data_in <<=1;
      data_in |= (PINB & 0x02) >> 1;     //Bitwise OR with PINB to build the input
      PORTB &= ~PIN_PB2;       //Set the ClockPin Low (PB2)     
    }
    tick++;
  }    
  return data_in;    
}

/******************************************************
 * Name:        readREC
 * Description: reads the current value in the Receive Error Count Register
 *****************************************************/
uint8_t readREC(){
  uint8_t ret;
    digitalWrite(CS, LOW);
    SPI_transfer(READ);
    SPI_transfer(REC);
    ret = SPI_transfer(0x00); //sending 00s allows the return data to be built
    digitalWrite(CS, HIGH);
  return ret;
}

/******************************************************
 * Name:        mcp2515_setRegister
 * Description: Directly writing values to MCP registers. 
 *****************************************************/
void mcp2515_setRegister(uint8_t address, uint8_t value){
    digitalWrite(CS, LOW);
    SPI_transfer(WRITE); //Write command
    SPI_transfer(address);
    SPI_transfer(value);
    digitalWrite(CS, HIGH);
}

/******************************************************
 * Name:        config_Rate
 * Description: Changing the BaudRate configuration 
 *              registers
 *****************************************************/
uint8_t config_Rate(uint8_t canSpeed){
  /* NTOE: To add more values to these, a bitrate calculator must be used. */
  
  uint8_t sets, cfg1, cfg2, cfg3;
  sets = 1;
  switch (canSpeed) 
        {
            case (CAN_125KBS):                                             // 125Kbps
            cfg1 = CAN_125KBPS_cfg1;
            cfg2 = CAN_125KBPS_cfg2;
            cfg3 = CAN_125KBPS_cfg3;
            break;

            case (CAN_250KBS):                                             // 250Kbps
            cfg1 = CAN_250KBPS_cfg1;
            cfg2 = CAN_250KBPS_cfg2;
            cfg3 = CAN_250KBPS_cfg3;
            break;

            case (CAN_500KBS):                                             // 500Kbps
            cfg1 = CAN_500KBPS_cfg1;
            cfg2 = CAN_500KBPS_cfg2;
            cfg3 = CAN_500KBPS_cfg3;
            break;
      
            case (CAN_666KBS):                                             // 666Kbps
            cfg1 = CAN_666KBPS_cfg1;
            cfg2 = CAN_666KBPS_cfg2;
            cfg3 = CAN_666KBPS_cfg3;
            break;
        
            case (CAN_1000KBS):                                            //   1Mbps
            cfg1 = CAN_1000KBPS_cfg1;
            cfg2 = CAN_1000KBPS_cfg2;
            cfg3 = CAN_1000KBPS_cfg3;
            break;  

            default:
            sets = 0;
            return 1;
            break;
        } 

   if(sets){
    digitalWrite(CS,LOW);
    SPI_transfer(BIT_MODIFY); // instruction to modify bits
    SPI_transfer(CANCTRL); //Address Byte
    SPI_transfer(0xFF); //mask byte
    SPI_transfer(CONFIGURE_MODE_CLKOUT_1); //CONFIGURE MODE with clockout divisor of 1
    digitalWrite(CS,HIGH);
    delay(100);
    
    mcp2515_setRegister(CFG1_Reg, cfg1);
    mcp2515_setRegister(CFG2_Reg, cfg2);
    mcp2515_setRegister(CFG3_Reg, cfg3);

    digitalWrite(CS,LOW);
    SPI_transfer(BIT_MODIFY); // instruction to modify bits
    SPI_transfer(CANCTRL); //Address Byte
    SPI_transfer(0xFF); //mask byte
    SPI_transfer(NORMAL_MODE_CLKOUT_1); //NORMAL MODE with Clockout divisor of 1
    digitalWrite(CS,HIGH);
    delay(100);

    return 0; 
   }
   return 1;
}

/******************************************************
 * Name:        autobaud
 * Description: Changes the CAN Bus baudrate to 
 *              determine whether or not the baudrate 
 *              is properly set.
 *****************************************************/
uint8_t autobaud(){
  
  uint32_t previousMillis100 = millis();
  int current_baud = 0;
  int lastREC = readREC();
  
  while(true){
    //
    while((millis() - previousMillis100)<= 100){
      
    if(CAN0.checkReceive() == CAN_MSGAVAIL){ //Checking to see if canBus frames have come onto the bus
      return canSpeed[current_baud];
      }
    if(lastREC <= readREC()){
      lastREC = readREC();
      break;
      }
    }
    current_baud++;
    if(current_baud>=5){
      current_baud=0;
    }
  config_Rate(canSpeed[current_baud]);
  previousMillis100 = millis(); // moved this to aftetr the config_Rate to prevent immediate escape of the while loop. 
  }
}

/******************************************************
 * Name:        flash
 * Description: flashes the specified LED one time. 
 *****************************************************/
void flash(uint8_t led){
  digitalWrite(led, HIGH);
  delay(10);
  digitalWrite(led, LOW);
}

/******************************************************
 * Name:        error
 * Description: flashes the LED in accordance with a 
 *              numeric error message. (Similar to 
 *              OBDI diagnostics lol)
 *****************************************************/
void error(uint8_t led, uint8_t error){
  for(int i=0; i<=error; ++i){
    flash(led);
    delay(20);
    }
  }

/******************************************************
 * Name:        sendFail
 * Description: one long pause on LED HIGH
 *****************************************************/
void sendFail(uint8_t led){
  delay(25);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led, LOW);
}

/******************************************************
 * Name:        sos
 * Description: flashes LED in SOS morse code. Used for 
 *              unknown territory. 
 *****************************************************/
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

/******************************************************
 * Name:        setupWatchDog
 * Description: Takes input of WDT_SETUP_CONF and
 *              WDT_WAIT_TIME to configure the WDT.
 *****************************************************/
void setupWatchDog(){
  if(WDT_SETUP_CONF == WDT_INT){
    if(WDT_WAIT_TIME == WDT_8sec){
      /* Clear the reset flag. */
      MCUSR &= ~(1<<WDRF);
      WDTCR |= (1<<WDCE) | (1<<WDE);
      /* set new watchdog timeout prescaler value */
      WDTCR |= WDT_8sec; 
      /* Enable the WD interrupt (note no reset). */
      WDTCR |= _BV(WDIE);
    }
    else if(WDT_WAIT_TIME == WDT_4sec){
      /* Clear the reset flag. */
      MCUSR &= ~(1<<WDRF);
      WDTCR |= (1<<WDCE) | (1<<WDE);
      /* set new watchdog timeout prescaler value */
      WDTCR |= WDT_4sec; 
      /* Enable the WD interrupt (note no reset). */
      WDTCR |= _BV(WDIE);
    }
    else if(WDT_WAIT_TIME == WDT_2sec){
      /* Clear the reset flag. */
      MCUSR &= ~(1<<WDRF);
      WDTCR |= (1<<WDCE) | (1<<WDE);
      /* set new watchdog timeout prescaler value */
      WDTCR |= WDT_2sec; 
      /* Enable the WD interrupt (note no reset). */
      WDTCR |= _BV(WDIE);
    }
    else if(WDT_WAIT_TIME == WDT_1sec){
      /* Clear the reset flag. */
      MCUSR &= ~(1<<WDRF);
      WDTCR |= (1<<WDCE) | (1<<WDE);
      /* set new watchdog timeout prescaler value */
      WDTCR |= WDT_1sec; 
      /* Enable the WD interrupt (note no reset). */
      WDTCR |= _BV(WDIE);
    }
  }
  else if(WDT_SETUP_CONF == WDT_RESET){
    if(WDT_WAIT_TIME == WDT_8sec){
      /* Clear the reset flag. */
      MCUSR &= ~(1<<WDRF);
      WDTCR |= (1<<WDCE) | (1<<WDE);
      /* set new watchdog timeout prescaler value */
      WDTCR |= WDT_8sec; 
      /* Enable the WD reset. */
      WDTCR |= _BV(WDE);
    }
    else if(WDT_WAIT_TIME == WDT_4sec){
      /* Clear the reset flag. */
      MCUSR &= ~(1<<WDRF);
      WDTCR |= (1<<WDCE) | (1<<WDE);
      /* set new watchdog timeout prescaler value */
      WDTCR |= WDT_4sec; 
      /* Enable the WD reset. */
      WDTCR |= _BV(WDE);
    }
    else if(WDT_WAIT_TIME == WDT_2sec){
      /* Clear the reset flag. */
      MCUSR &= ~(1<<WDRF);
      WDTCR |= (1<<WDCE) | (1<<WDE);
      /* set new watchdog timeout prescaler value */
      WDTCR |= WDT_2sec; 
      /* Enable the WD reset. */
      WDTCR |= _BV(WDE);
    }
    else if(WDT_WAIT_TIME == WDT_1sec){
      /* Clear the reset flag. */
      MCUSR &= ~(1<<WDRF);
      WDTCR |= (1<<WDCE) | (1<<WDE);
      /* set new watchdog timeout prescaler value */
      WDTCR |= WDT_1sec; 
      /* Enable the WD reset. */
      WDTCR |= _BV(WDE);
    }
  }
  else if(WDT_SETUP_CONF == WDT_INT_RESET){
    if(WDT_WAIT_TIME == WDT_8sec){
      /* Clear the reset flag. */
      MCUSR &= ~(1<<WDRF);
      WDTCR |= (1<<WDCE) | (1<<WDE);
      /* set new watchdog timeout prescaler value */
      WDTCR |= WDT_8sec; 
      /* Enable the WD reset. */
      WDTCR |= _BV(WDE) | _BV(WDIE);
    }
    else if(WDT_WAIT_TIME == WDT_4sec){
      /* Clear the reset flag. */
      MCUSR &= ~(1<<WDRF);
      WDTCR |= (1<<WDCE) | (1<<WDE);
      /* set new watchdog timeout prescaler value */
      WDTCR |= WDT_4sec; 
      /* Enable the WD reset. */
      WDTCR |= _BV(WDE) | _BV(WDIE);
    }
    else if(WDT_WAIT_TIME == WDT_2sec){
      /* Clear the reset flag. */
      MCUSR &= ~(1<<WDRF);
      WDTCR |= (1<<WDCE) | (1<<WDE);
      /* set new watchdog timeout prescaler value */
      WDTCR |= WDT_2sec; 
      /* Enable the WD reset. */
      WDTCR |= _BV(WDE) | _BV(WDIE);
    }
    else if(WDT_WAIT_TIME == WDT_1sec){
      /* Clear the reset flag. */
      MCUSR &= ~(1<<WDRF);
      WDTCR |= (1<<WDCE) | (1<<WDE);
      /* set new watchdog timeout prescaler value */
      WDTCR |= WDT_1sec; 
      /* Enable the WD reset. */
      WDTCR |= _BV(WDE) | _BV(WDIE);
    }
  }
}

void setupMCP(){
  //Reset the CAN Controller
  digitalWrite(CS,LOW);
  SPI_transfer(RESET); //Reset
  digitalWrite(CS,HIGH);
  
  delay(10); //allow time to reset

  // The following sequence is the only way we could get the clkout divisor to stay at 1

  digitalWrite(CS,LOW);
  SPI_transfer(BIT_MODIFY); // instruction to modify bits
  SPI_transfer(CANCTRL); //Address Byte
  SPI_transfer(0xFF); //mask byte
  SPI_transfer(CONFIGURE_MODE_CLKOUT_8); //CONFIGURE MODE with clockout divisor of 8
  digitalWrite(CS,HIGH);

  delay(100);  

  digitalWrite(CS,LOW);
  SPI_transfer(BIT_MODIFY); // instruction to modify bits
  SPI_transfer(CANCTRL); //Address Byte
  SPI_transfer(0xFF); //mask byte
  SPI_transfer(CONFIGURE_MODE_CLKOUT_4); //CONFIGURE MODE with clockout divisor of 4
  digitalWrite(CS,HIGH);

  delay(100);

  digitalWrite(CS,LOW);
  SPI_transfer(BIT_MODIFY); // instruction to modify bits
  SPI_transfer(CANCTRL); //Address Byte
  SPI_transfer(0xFF); //mask byte
  SPI_transfer(CONFIGURE_MODE_CLKOUT_2); //CONFIGURE MODE with clockout divisor of 2
  digitalWrite(CS,HIGH);

  delay(100);

  digitalWrite(CS,LOW);
  SPI_transfer(BIT_MODIFY); // instruction to modify bits
  SPI_transfer(CANCTRL); //Address Byte
  SPI_transfer(0xFF); //mask byte
  SPI_transfer(CONFIGURE_MODE_CLKOUT_1); //CONFIGURE MODE with clockout divisor of 1
  digitalWrite(CS,HIGH);

  delay(100);
}

void setup() {
  
  /* pin mode sets */ 
  pinMode(CS,OUTPUT);
  pinMode(SCK,OUTPUT);
  pinMode(DI,INPUT_PULLUP);
  pinMode(DO,OUTPUT);
  pinMode(GREEN,OUTPUT);
  pinMode(RED,OUTPUT);
  pinMode(SILENT, OUTPUT);

  /*setting initial states*/
  digitalWrite(GREEN,HIGH) ;
  digitalWrite(RED,HIGH); 
  digitalWrite(CS,HIGH);
  digitalWrite(SCK,LOW);

  /*Can_val check to make sure bus initializes*/
  //Gotta make sure the saved value is an acceptable value for the initialization
  if(can_Val != CAN_250KBS && can_Val != CAN_500KBS && can_Val != CAN_125KBS && can_Val != CAN_666KBS && can_Val != CAN_1000KBS){
    can_Val == CAN_250KBS;
  }

  setupWatchDog();

  /*Can_val check to make sure bus initializes*/
  //Gotta make sure the saved value is an acceptable value for the initialization
  if(can_Val != CAN_250KBS && can_Val != CAN_500KBS && can_Val != CAN_125KBS && can_Val != CAN_666KBS && can_Val != CAN_1000KBS){
    can_Val = CAN_250KBS;
  }
  
  setupMCP();

  flash(GREEN);
  flash(RED);
  
  /*Initialize CAN and then check to make sure you are receiving CAN frames. If not, the autobaud routine 
    determines the proper baudrate value and then writes it to EEPROM */
  if(CAN0.begin(MCP_ANY, can_Val, MCP_16MHZ)==CAN_OK) //can_Val is the EEPROM saved baudrate 
  {
    autobaudCan_Val = autobaud();
    if(autobaudCan_Val != can_Val){
      EEPROM.write(CAN_BAUDRATE, autobaudCan_Val);
    }
  }  

delay(100); 
}

void loop()
{ 
  // This should be the Low Power Setting allowing the device to sleep for 8 seconds. 
  // This can be changed in the setup by referencing pg. 48 of ATTINY861 Datasheet. 
  
  if(f_wdt == 1){ //this makes sure that the device is alive, and stops sleep until called to sleep again
    //Error monitoring on ELD side of the BUS
    if(EFLG_MODE != EFLAG_MODE_OFF){
      if(CAN0.getError()&& first_bit){// Check the first bit of the register
        digitalWrite(SILENT, HIGH);
        while(CAN0.getError()&& first_bit){ // wait until the first bit is not 1
          digitalWrite(SILENT, HIGH);
          }
        delay(50); // Additional time to reduce
        digitalWrite(SILENT, LOW);
      }
    }
    if(EFLG_MODE == EFLAG_MODE_OFF){
      if(readREC() >= REC_TRIGGER){
        digitalWrite(SILENT, HIGH);
        while(readREC() >= REC_TRIGGER){
        }
        delay(50);
        digitalWrite(SILENT, LOW);
      }
    }
    //Clear the watchdog flag
    f_wdt = 0;

    //Re-enter sleep mode "Low-Power-Down"
    //enterSleep(); // this will make the device go into a low power mode. 

    /*
     * We need to add protections to make sure that in the event that this device is plugged in for 
     * 50 days the code does not get stuck. The watchdog timer can also be used to reset the device
     * with checks to make sure that the code is still functioning. In the event that the code locks
     * up. 
     */

    /*
     * We also need to discuss how the REC register will reduce if the silent pin is triggered since
     * register decrements for successful receives. Or if the methodology needs to be changed. 
     */
  }
}


