# CAN-Data-Diode
Source Code for a Security Device to protect and isolate parts of a CAN on vehicles and other industrial controls.

# License
The intellectual property regarding the hardware descescribed in this project is owned by the University of Tulsa. Any non-commercial use of the hardware is permissible. However, commercialization of the hardware will require a license agreement through The University of Tulsa.

All the software in this repository is release under and MIT License:

Copyright 2018 Jeremy Daily, Hayden Allen

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

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
![EEPROM MAP](https://github.com/Heavy-Vehicle-Networking-At-U-Tulsa/CAN-Data-Diode/blob/master/extras/EEPROM_memory_map.PNG)

# Current Objectives
- Review code and Test to ensure functionality.

# Discussion Topics
 - We need to add protections to make sure that in the event that the device is plugged in for 50 days straight the
code does not get stuck. The watchdog timer can also be used to reset the device with checks to make sure that the 
code is still functioning. In the event that the code locks up. 
 - We also need to discuss how the REC register will reduce if the silent pin is triggered since
    register decrements for successful receives. Or if the methodology needs to be changed. 
