EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:wbraun_ic_lib
LIBS:host_board-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 128x64_OLED-I2C U501
U 1 1 56CAB7C8
P 2500 3550
F 0 "U501" H 2150 3950 60  0000 C CNN
F 1 "128x64_OLED-I2C" H 2500 3150 60  0000 C CNN
F 2 "wbraun_smd:OLED_128x64_Display" H 2500 3550 60  0001 C CNN
F 3 "" H 2500 3550 60  0000 C CNN
	1    2500 3550
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR065
U 1 1 56CAB8B1
P 3100 3300
F 0 "#PWR065" H 3100 3150 50  0001 C CNN
F 1 "+3V3" H 3100 3440 50  0000 C CNN
F 2 "" H 3100 3300 50  0000 C CNN
F 3 "" H 3100 3300 50  0000 C CNN
	1    3100 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR066
U 1 1 56CAB8C7
P 3400 3500
F 0 "#PWR066" H 3400 3250 50  0001 C CNN
F 1 "GND" H 3400 3350 50  0000 C CNN
F 2 "" H 3400 3500 50  0000 C CNN
F 3 "" H 3400 3500 50  0000 C CNN
	1    3400 3500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3100 3300 3100 3400
Wire Wire Line
	3100 3400 3000 3400
Wire Wire Line
	3400 3500 3000 3500
Text HLabel 3100 3700 2    60   Input ~ 0
Display_SDA
Text HLabel 3100 3600 2    60   Input ~ 0
Display_SCL
Wire Wire Line
	3100 3600 3000 3600
Wire Wire Line
	3100 3700 3000 3700
$EndSCHEMATC
