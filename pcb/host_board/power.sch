EESchema Schematic File Version 2
LIBS:host_board-rescue
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
Sheet 2 4
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
L C C202
U 1 1 56C0ED79
P 4100 4150
F 0 "C202" H 4125 4250 50  0000 L CNN
F 1 "4.7uF" H 4125 4050 50  0000 L CNN
F 2 "wbraun_smd:C_0603" H 4138 4000 50  0001 C CNN
F 3 "" H 4100 4150 50  0000 C CNN
	1    4100 4150
	1    0    0    -1  
$EndComp
$Comp
L C C201
U 1 1 56C0EE52
P 2300 4150
F 0 "C201" H 2325 4250 50  0000 L CNN
F 1 "4.7uF" H 2325 4050 50  0000 L CNN
F 2 "wbraun_smd:C_0603" H 2338 4000 50  0001 C CNN
F 3 "" H 2300 4150 50  0000 C CNN
	1    2300 4150
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR01
U 1 1 56C0EE95
P 2300 3900
F 0 "#PWR01" H 2300 3750 50  0001 C CNN
F 1 "+5V" H 2300 4040 50  0000 C CNN
F 2 "" H 2300 3900 50  0000 C CNN
F 3 "" H 2300 3900 50  0000 C CNN
	1    2300 3900
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR02
U 1 1 56C0EEAF
P 4100 3900
F 0 "#PWR02" H 4100 3750 50  0001 C CNN
F 1 "+3V3" H 4100 4040 50  0000 C CNN
F 2 "" H 4100 3900 50  0000 C CNN
F 3 "" H 4100 3900 50  0000 C CNN
	1    4100 3900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 56C0EEC9
P 2300 4500
F 0 "#PWR03" H 2300 4250 50  0001 C CNN
F 1 "GND" H 2300 4350 50  0000 C CNN
F 2 "" H 2300 4500 50  0000 C CNN
F 3 "" H 2300 4500 50  0000 C CNN
	1    2300 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 56C0EEE3
P 4100 4500
F 0 "#PWR04" H 4100 4250 50  0001 C CNN
F 1 "GND" H 4100 4350 50  0000 C CNN
F 2 "" H 4100 4500 50  0000 C CNN
F 3 "" H 4100 4500 50  0000 C CNN
	1    4100 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4000 2300 4000
Wire Wire Line
	2300 4000 2300 3900
Wire Wire Line
	2300 4300 2300 4500
$Comp
L GND #PWR05
U 1 1 56C0EF26
P 2800 4500
F 0 "#PWR05" H 2800 4250 50  0001 C CNN
F 1 "GND" H 2800 4350 50  0000 C CNN
F 2 "" H 2800 4500 50  0000 C CNN
F 3 "" H 2800 4500 50  0000 C CNN
	1    2800 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4200 2800 4500
Wire Wire Line
	3700 4200 3700 4000
Wire Wire Line
	3700 4000 4100 4000
Wire Wire Line
	4100 4000 4100 3900
Wire Wire Line
	4100 4300 4100 4500
$Comp
L AP2114H U201
U 1 1 56CA476F
P 3250 4100
F 0 "U201" H 3050 4350 60  0000 C CNN
F 1 "AP2114H" H 3200 3850 60  0000 C CNN
F 2 "wbraun_smd:SOT-223" H 3250 4100 60  0001 C CNN
F 3 "" H 3250 4100 60  0000 C CNN
	1    3250 4100
	1    0    0    -1  
$EndComp
$EndSCHEMATC
