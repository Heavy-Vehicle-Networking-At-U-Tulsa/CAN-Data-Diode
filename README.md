# CAN-Data-Diode
Source Code for a Security Device to protect and isolate parts of a CAN on vehicles and other industrial controls.
# Upload Order
To upload to the Data Diode for proper functionality:
* First configure the device using the EEPROM_MAP.ino sketch
* Choose the appropriate settings and make changes as needed for your application
* This sketch writes the EEPROM memory of the device for the initial start up after this has been completed ONCE it is not necessary to complete again unless settings need to be modified. 
* This sketch needs to be uploaded to both sides of the device. 
* Second upload the ELD sketch to the ELD side of the Data Diode
* Third upload the J1939 sketch to the J1939 side of the Diode
* This will allow for the ELD side to function in accordance with the proper ELD config and the J1939 side to operate on the proper config. 

# EEPROM MAP
![EEPROM MAP](https://github.com/Heavy-Vehicle-Networking-At-U-Tulsa/CAN-Data-Diode/blob/Edit-of-README.md/EEPROM_memory_map.PNG)

# Current Objectives
- add can_val check to ELD sketch to make sure that the EEPROM value is set correctly. 
- add a J1939 sketch

# Discussion Topics
 - We need to add protections to make sure that in the event that the device is plugged in for 50 days straight the
code does not get stuck. The watchdog timer can also be used to reset the device with checks to make sure that the 
code is still functioning. In the event that the code locks up. 
 - We also need to discuss how the REC register will reduce if the silent pin is triggered since
    register decrements for successful receives. Or if the methodology needs to be changed. 
