This is the github repository for a solar photovoltaic IV curver tracer load unit.
This project has been started by Matt Little from Renewable Energy Innovation.
Contact:
www.re-innovation.co.uk
matt@re-innovation.co.uk

This is licensed under Creative Commons 3.0 With Attribution Share Alike (CC BY-SA)

Overview:

 Photovoltaic Current-Voltage Tracer (PV I-V Tracer)
 
 This is the code to produce a solar PV current-voltage curve tracer.
 This is based upon the Pedalog v2 PCB (from www.re-innovation.co.uk). This is based upon the arduino 
 but with current and voltage monitoring along with dual MOSFET drivers.
 
 It displays data using an iTEAD TFT LCD screen
 (Example code: http://www.re-innovation.co.uk/web12/index.php/en/blog-75/269-tft-lcd-display)
 
 Voltage is read using a potential divider (680K // 10k).
 Current is read using an AD211 high side current monitor with a shunt resistor.
 The internal 1.1V reference voltage is used. This is calibrated (as it can range from 1.0 to 1.2V)
 The calbration factor is stored in EEPROM and is the data reading when 0.25V is applied.
 
 The load are two MOSFETs, which are short circuited and controlled via PWM.

 The connections are as follows:
 Arduino:
 D0        Serial 
 D1        Serial
 D2        Switch 1
 D3        TFT  RESET
 D4        Switch 2
 D5        TFT  RS
 D6        TFT  Chip Select (CS)  
 D7
 D8        Buzzer output
 D9        MOSFET DUMP 1  (PWM)
 D10       MOSFET DUMP 2  (PWM)  
 D11       TFT  SDA  Serial Data
 D12  
 D13       TFT  SCLK  Serial Clock
 
 A0          Voltage input (via 680k/10k potential divider)
 A1          Current input (via AD8211 high side monitor and 0.05 ohm current shunt resistor)
 A2
 A3
 A4
 A5
 
 Created on 9/10/13 
 by Matt Little
 matt@re-innovation.co.uk
 www.re-innovation.co.uk
 This code is open source and public domain.
 
see www.re-innovation.co.uk for more details


Files in the repository:
	
	Readme
	Arduino code
	Frizting Layout diagram


Modified:

31/10/13	Matt Little	Created


To Do:

 