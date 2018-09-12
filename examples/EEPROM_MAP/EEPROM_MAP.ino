#include <EEPROM.h>
/****************************************************/

/* PIN Defines */ 
#define GREEN 10 // Pin for the Green LED
#define RED 14 // Red LED pin

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
//0x08-0x1F are reserved for additional request message space. 

#define MAX_REC       0x21 // When to trigger the silent pin on the transceiver.
#define EFLG          0x22 // Use EFLG Register rather than the REC register for silent detection
#define WDT_TIME      0x23 // Configuration of the WDT time interval
#define WDT_CONF      0x24 // Does the user want to set up the device to reset in the event of a code hang up. 
#define CAN_ID1_int   0x25 // Rate at which CANID1 messages are sent (may be implemented as a method choice)
#define CAN_ID2_int   0x26 // Rate at which CANID2 messages are sent (may be implemented as a method choice)
#define CAN_ID3_int   0x27 // Rate at which CANID3 messages are sent (may be implemented as a method choice)
//0x28 - 0x33 are reserved for additional CAN intervals      

/* CAN SPEED VALUES */
#define CAN_250KBS  0x12   // Set a baudrate of 250kbs
#define CAN_500KBS  0x13   // Set a baudrate of 500kbs
#define CAN_125KBS  0x10   // Set a baudrate of 125kbs
#define CAN_666KBS  0x14   // Set a baudrate of 666kbs
#define CAN_1000KBS 0x15   // Set a baudrate of 1000kbs

/* CAN MESSAGE Values */
#define VIN_REQ_LSB     0xEC
#define VIN_REQ_MSB     0xFE
#define HOURS_REQ_LSB   0xE5
#define HOURS_REQ_MSB   0xFE
#define COMP_REQ_LSB    0xEB
#define COMP_REQ_MSB    0xFE
#define VEH_HOURS_LSB   0xE7
#define VEH_HOURS_MSB   0xFE
#define VEH_WT_REQ_LSB  0xEA
#define VEH_WT_REQ_MSB  0xFE
//Use this space to define additional request messages

/* Watchdog Timer Config */
#define WDT_8sec        0x21
#define WDT_4sec        0x20
#define WDT_2sec        0x07
#define WDT_1sec        0x06

/* WDT CONF Options */
#define WDT_INT         0x01
#define WDT_RESET       0x02
#define WDT_INT_RESET   0x03

/* CAN INTERVALS */
#define minutes5       0x05
#define minutes10      0x0A
#define minutes20      0x14
#define minutes60      0x3C

/* DEFAULT EEPROM VALUES */
#define max_REC_val     0x0C
#define EFLG_bool       0x00

void flash(int led){
    digitalWrite(led, HIGH);
    delay(10);
    digitalWrite(led, LOW);
}

void setup() {
  // Pinmode Setup:
    pinMode(GREEN, OUTPUT);
    pinMode(RED, OUTPUT);
  
  // Defining the EEPROM register values
    digitalWrite(GREEN, HIGH);
    EEPROM.write(CAN_BAUDRATE, CAN_250KBS); //DEFAULT: CAN_250KBS, This can be changed to change the initial baudrate on very first startup. 
    flash(RED);
    EEPROM.write(CAN_MSGCOUNT, 0x02); // Default: 0x02, Define how many messages you are wanting to send.
    flash(RED);
    EEPROM.write(CAN_ID1LSB, VIN_REQ_LSB); // Default: VIN_REQ_LSB, LSB for CANID1
    flash(RED);
    EEPROM.write(CAN_ID1MSB, VIN_REQ_MSB); // Default: VIN_REQ_MSB, MSB for CANID1
    flash(RED);
    EEPROM.write(CAN_ID2LSB, HOURS_REQ_LSB); // Default: HOURS_REQ_LSB, LSB for CANID2
    flash(RED);
    EEPROM.write(CAN_ID2MSB, HOURS_REQ_MSB); // Default: HOURS_REQ_MSB, MSB for CANID2
    flash(RED);
    EEPROM.write(MAX_REC, max_REC_val); //This defines when the silent pin will be triggered. The default is a decimal value of 12, when the TEC of the ELD will be about to hit 97.  
    flash(RED);
    EEPROM.write(EFLG, EFLG_bool); //Defualt: OFF, This monitors the EFLG register instead of the REC register
    flash(RED);
    EEPROM.write(WDT_TIME, WDT_8sec); //Default: WDT_8sec, this sets the timer length between wakeups and checks. 
    flash(RED);
    EEPROM.write(WDT_CONF, WDT_INT); //Default: WDT_INT, Change the behaviour of the watchdog timer based on the ATTINY861 Datasheet.
    flash(RED);
    EEPROM.write(CAN_ID1_int, minutes60); //Default: 60_minutes, sends CANID1 at the described interval.
    flash(RED);
    EEPROM.write(CAN_ID2_int, minutes60); //Default: 60_minutes, sends CANID2 at the described interval.
    flash(RED);
    flash(GREEN);
    //Note: add additional CAN_ID#_int lines to add a timed send. 
}

void loop() {
  // put your main code here, to run repeatedly:

}
