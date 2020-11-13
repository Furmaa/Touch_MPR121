EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 3200 2100 1    50   Input ~ 0
+21V
$Comp
L Transistor_FET:IPP060N06N Q1
U 1 1 5F4BE07F
P 6050 2500
F 0 "Q1" H 6254 2500 50  0000 L CNN
F 1 "IPP060N06N" H 6255 2455 50  0001 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 6300 2425 50  0001 L CIN
F 3 "https://www.infineon.com/dgdl/Infineon-IPP060N06N-DS-v02_02-en.pdf?fileId=db3a30433727a44301372c06d9d7498a" H 6050 2500 50  0001 L CNN
	1    6050 2500
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_FET:IPP060N06N Q2
U 1 1 5F4C3589
P 5250 2900
F 0 "Q2" H 5454 2900 50  0000 L CNN
F 1 "IPP060N06N" H 5455 2855 50  0001 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 5500 2825 50  0001 L CIN
F 3 "https://www.infineon.com/dgdl/Infineon-IPP060N06N-DS-v02_02-en.pdf?fileId=db3a30433727a44301372c06d9d7498a" H 5250 2900 50  0001 L CNN
	1    5250 2900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 5F4D0236
P 2800 2500
F 0 "R1" V 2593 2500 50  0000 C CNN
F 1 "R680" V 2684 2500 50  0000 C CNN
F 2 "" V 2730 2500 50  0001 C CNN
F 3 "~" H 2800 2500 50  0001 C CNN
	1    2800 2500
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x10_Male J1
U 1 1 5F4D07C0
P 1500 2800
F 0 "J1" H 1608 3381 50  0000 C CNN
F 1 "Conn_01x10_Male" H 1608 3465 50  0000 C CNN
F 2 "" H 1500 2800 50  0001 C CNN
F 3 "~" H 1500 2800 50  0001 C CNN
	1    1500 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 2600 1700 2700
Wire Wire Line
	1700 3100 1700 3000
Wire Wire Line
	2950 2500 3200 2500
Text GLabel 3900 2050 1    50   Input ~ 0
+9V
$Comp
L Device:R R2
U 1 1 5F4DAEA4
P 2800 2900
F 0 "R2" V 2593 2900 50  0000 C CNN
F 1 "R220" V 2684 2900 50  0000 C CNN
F 2 "" V 2730 2900 50  0001 C CNN
F 3 "~" H 2800 2900 50  0001 C CNN
	1    2800 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	2950 2900 3200 2900
Wire Wire Line
	2950 3300 3200 3300
$Comp
L Device:R R3
U 1 1 5F4D9030
P 2800 3300
F 0 "R3" V 2593 3300 50  0000 C CNN
F 1 "R680" V 2684 3300 50  0000 C CNN
F 2 "" V 2730 3300 50  0001 C CNN
F 3 "~" H 2800 3300 50  0001 C CNN
	1    2800 3300
	0    1    1    0   
$EndComp
$Comp
L Transistor_FET:IPP060N06N Q3
U 1 1 5F4C5F7E
P 5450 3650
F 0 "Q3" H 5654 3650 50  0000 L CNN
F 1 "IPP060N06N" H 5655 3605 50  0001 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 5700 3575 50  0001 L CIN
F 3 "https://www.infineon.com/dgdl/Infineon-IPP060N06N-DS-v02_02-en.pdf?fileId=db3a30433727a44301372c06d9d7498a" H 5450 3650 50  0001 L CNN
	1    5450 3650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4550 2600 4550 3550
Wire Wire Line
	4550 3550 5250 3550
Connection ~ 1700 2600
$Comp
L power:GND #PWR?
U 1 1 5F4F212E
P 6550 2800
F 0 "#PWR?" H 6550 2550 50  0001 C CNN
F 1 "GND" H 6555 2627 50  0000 C CNN
F 2 "" H 6550 2800 50  0001 C CNN
F 3 "" H 6550 2800 50  0001 C CNN
	1    6550 2800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6250 2400 6550 2400
Wire Wire Line
	5450 2800 6550 2800
Connection ~ 6550 2800
Wire Wire Line
	5650 3550 6550 3550
Text GLabel 5900 2700 0    50   Input ~ 0
D10
$Comp
L Device:R R6
U 1 1 5F4F6F8A
P 6300 2700
F 0 "R6" V 6093 2700 50  0000 C CNN
F 1 "R220k" V 6184 2700 50  0000 C CNN
F 2 "" V 6230 2700 50  0001 C CNN
F 3 "~" H 6300 2700 50  0001 C CNN
	1    6300 2700
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5F4F8824
P 6300 3100
F 0 "R7" V 6093 3100 50  0000 C CNN
F 1 "R220k" V 6184 3100 50  0000 C CNN
F 2 "" V 6230 3100 50  0001 C CNN
F 3 "~" H 6300 3100 50  0001 C CNN
	1    6300 3100
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 5F4F8D88
P 6300 3850
F 0 "R8" V 6093 3850 50  0000 C CNN
F 1 "R220k" V 6184 3850 50  0000 C CNN
F 2 "" V 6230 3850 50  0001 C CNN
F 3 "~" H 6300 3850 50  0001 C CNN
	1    6300 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	6450 3850 6550 3850
Wire Wire Line
	6550 3850 6550 3550
Wire Wire Line
	6150 3100 5250 3100
Wire Wire Line
	5450 3850 6150 3850
Text GLabel 5250 3200 3    50   Input ~ 0
D6
Text GLabel 5450 4000 3    50   Input ~ 0
D3
Wire Wire Line
	5450 4000 5450 3850
Connection ~ 5450 3850
Wire Wire Line
	5250 3200 5250 3100
Connection ~ 5250 3100
Wire Wire Line
	4800 3200 4800 2400
Wire Wire Line
	4800 2400 5850 2400
$Comp
L power:GNDPWR #PWR?
U 1 1 5F509235
P 6550 3550
F 0 "#PWR?" H 6550 3350 50  0001 C CNN
F 1 "GNDPWR" V 6555 3442 50  0000 R CNN
F 2 "" H 6550 3500 50  0001 C CNN
F 3 "" H 6550 3500 50  0001 C CNN
	1    6550 3550
	0    -1   -1   0   
$EndComp
Connection ~ 6550 3550
Wire Wire Line
	5900 2700 6050 2700
Connection ~ 6050 2700
Wire Wire Line
	6050 2700 6150 2700
Wire Wire Line
	6550 2400 6550 2800
Wire Wire Line
	6450 2700 6450 3100
Wire Wire Line
	6450 3100 6550 3100
Wire Wire Line
	6550 3100 6550 3550
Connection ~ 6450 3100
Text Label 3900 3100 0    50   ~ 0
9-2.8=6.2V
Text Label 1950 3000 0    50   ~ 0
6.2÷0.24=26mA
Wire Wire Line
	1700 3300 2650 3300
Wire Wire Line
	1700 2900 2650 2900
Wire Wire Line
	1700 2500 2650 2500
Text Label 1950 2500 0    50   ~ 0
4.2÷0.68=6mA
Text Label 3200 2500 0    50   ~ 0
21-6*2.8=4.2V
Text Label 1950 2900 0    50   ~ 0
1.4÷0.22=6mA
Wire Wire Line
	3200 2100 3200 2500
Connection ~ 3200 2500
Wire Wire Line
	3200 2500 3200 2900
Connection ~ 3200 2900
Wire Wire Line
	3200 2900 3200 3300
Wire Wire Line
	1700 2400 4800 2400
Connection ~ 4800 2400
Wire Wire Line
	1700 3200 4800 3200
Wire Wire Line
	1700 2800 5050 2800
Wire Wire Line
	1700 2600 4550 2600
Wire Wire Line
	1700 3100 3300 3100
Text Label 3200 2900 0    50   ~ 0
21-7.28=1.4V
Connection ~ 1700 3100
$Comp
L Device:R R4
U 1 1 5F785CCC
P 3450 3100
F 0 "R4" V 3243 3100 50  0000 C CNN
F 1 "R120" V 3334 3100 50  0000 C CNN
F 2 "" V 3380 3100 50  0001 C CNN
F 3 "~" H 3450 3100 50  0001 C CNN
	1    3450 3100
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5F786489
P 3750 3100
F 0 "R5" V 3543 3100 50  0000 C CNN
F 1 "R120" V 3634 3100 50  0000 C CNN
F 2 "" V 3680 3100 50  0001 C CNN
F 3 "~" H 3750 3100 50  0001 C CNN
	1    3750 3100
	0    1    1    0   
$EndComp
Wire Wire Line
	3900 2050 3900 3100
Text Label 1150 2400 0    50   ~ 0
blue
Text Label 1150 3300 0    50   ~ 0
black
$EndSCHEMATC