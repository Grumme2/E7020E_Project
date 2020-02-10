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
$Comp
L power:+3.3V #PWR02
U 1 1 5E2B99C7
P 1600 6500
F 0 "#PWR02" H 1600 6350 50  0001 C CNN
F 1 "+3.3V" H 1615 6673 50  0000 C CNN
F 2 "" H 1600 6500 50  0001 C CNN
F 3 "" H 1600 6500 50  0001 C CNN
	1    1600 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5E2BBB8C
P 4250 5850
F 0 "#PWR01" H 4250 5600 50  0001 C CNN
F 1 "GND" H 4255 5677 50  0000 C CNN
F 2 "" H 4250 5850 50  0001 C CNN
F 3 "" H 4250 5850 50  0001 C CNN
	1    4250 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5E2C7370
P 3150 5150
F 0 "R2" V 2943 5150 50  0000 C CNN
F 1 "10k" V 3034 5150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3080 5150 50  0001 C CNN
F 3 "~" H 3150 5150 50  0001 C CNN
	1    3150 5150
	0    1    1    0   
$EndComp
Wire Wire Line
	3300 5150 3350 5150
$Comp
L Device:C C6
U 1 1 5E28D2CD
P 3950 2000
F 0 "C6" H 4065 2046 50  0000 L CNN
F 1 "100n" H 4065 1955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3988 1850 50  0001 C CNN
F 3 "~" H 3950 2000 50  0001 C CNN
	1    3950 2000
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C7
U 1 1 5E28CEEE
P 3950 2200
F 0 "C7" H 4065 2246 50  0000 L CNN
F 1 "100n" H 4065 2155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3988 2050 50  0001 C CNN
F 3 "~" H 3950 2200 50  0001 C CNN
	1    3950 2200
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C5
U 1 1 5E28CA4C
P 3950 1800
F 0 "C5" V 3698 1800 50  0000 C CNN
F 1 "100n" V 3789 1800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3988 1650 50  0001 C CNN
F 3 "~" H 3950 1800 50  0001 C CNN
	1    3950 1800
	0    1    1    0   
$EndComp
$Comp
L Device:C C3
U 1 1 5E28C279
P 3300 2000
F 0 "C3" H 3415 2046 50  0000 L CNN
F 1 "100n" H 3415 1955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3338 1850 50  0001 C CNN
F 3 "~" H 3300 2000 50  0001 C CNN
	1    3300 2000
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C4
U 1 1 5E28B4AA
P 3300 2200
F 0 "C4" H 3415 2246 50  0000 L CNN
F 1 "100n" H 3415 2155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3338 2050 50  0001 C CNN
F 3 "~" H 3300 2200 50  0001 C CNN
	1    3300 2200
	0    -1   -1   0   
$EndComp
Connection ~ 3800 2000
Wire Wire Line
	3800 2000 3800 2200
Wire Wire Line
	2400 5650 2400 5150
Wire Wire Line
	3000 5150 2400 5150
Wire Wire Line
	3800 1800 3800 2000
$Comp
L Device:C C8
U 1 1 5E3138B4
P 4400 1850
F 0 "C8" H 4515 1896 50  0000 L CNN
F 1 "100n" H 4515 1805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4438 1700 50  0001 C CNN
F 3 "~" H 4400 1850 50  0001 C CNN
	1    4400 1850
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x05_Male J1
U 1 1 5E340471
P 1350 6700
F 0 "J1" H 1322 6632 50  0000 R CNN
F 1 "Conn_01x05_Male" H 1322 6723 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 1350 6700 50  0001 C CNN
F 3 "~" H 1350 6700 50  0001 C CNN
	1    1350 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 6700 1750 6700
Wire Wire Line
	1650 6600 1550 6600
Text Label 1700 6900 0    50   ~ 0
NRST
Wire Wire Line
	1700 6900 1550 6900
$Comp
L LM117ADJ:LM1117ADJ U2
U 1 1 5E29C2E5
P 7050 2550
F 0 "U2" H 7100 2875 50  0000 C CNN
F 1 "LM1117ADJ" H 7100 2784 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223" H 7050 2550 50  0001 C CNN
F 3 "" H 7050 2550 50  0001 C CNN
	1    7050 2550
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_OTG J2
U 1 1 5E2A6C12
P 6400 1500
F 0 "J2" H 6457 1967 50  0000 C CNN
F 1 "USB_OTG" H 6457 1876 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Wuerth_629105150521" H 6550 1450 50  0001 C CNN
F 3 " ~" H 6550 1450 50  0001 C CNN
	1    6400 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 2300 6650 2300
Wire Wire Line
	6650 2300 6650 2200
Wire Wire Line
	6650 2050 6750 2050
$Comp
L Device:C C10
U 1 1 5E2D92D2
P 7400 2950
F 0 "C10" H 7515 2996 50  0000 L CNN
F 1 "10n" H 7515 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7438 2800 50  0001 C CNN
F 3 "~" H 7400 2950 50  0001 C CNN
	1    7400 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 3400 7700 3400
Wire Wire Line
	7400 3100 7400 3400
Connection ~ 7400 3400
Wire Wire Line
	8350 1300 8350 2200
Wire Wire Line
	8350 2800 8200 2800
Connection ~ 7400 2800
NoConn ~ 5150 3250
NoConn ~ 5150 3350
NoConn ~ 5150 3450
NoConn ~ 5150 3550
NoConn ~ 5150 3650
NoConn ~ 5150 3750
NoConn ~ 5150 3850
NoConn ~ 3350 2950
NoConn ~ 3350 3250
NoConn ~ 3350 3350
NoConn ~ 3350 3450
NoConn ~ 3350 3750
NoConn ~ 3350 3850
NoConn ~ 3350 4450
NoConn ~ 3350 4950
NoConn ~ 5150 4050
NoConn ~ 5150 4150
NoConn ~ 5150 4450
NoConn ~ 5150 4250
NoConn ~ 5150 4550
NoConn ~ 5150 4650
NoConn ~ 5150 4750
NoConn ~ 5150 4850
NoConn ~ 5150 4950
Wire Wire Line
	3150 2200 3150 2000
Wire Wire Line
	3150 2000 3150 1450
Connection ~ 3150 2000
Wire Wire Line
	3800 1800 3800 1450
Wire Wire Line
	3800 1450 3150 1450
Connection ~ 3800 1800
Connection ~ 3150 1450
Wire Wire Line
	4550 1600 4550 1850
Wire Wire Line
	4550 1850 4550 2150
Wire Wire Line
	4350 2150 4550 2150
Connection ~ 4550 1850
Wire Wire Line
	4250 1850 4250 1450
Connection ~ 3800 1450
Wire Wire Line
	6400 1900 6400 2000
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5E35EA03
P 7300 1050
F 0 "#FLG0101" H 7300 1125 50  0001 C CNN
F 1 "PWR_FLAG" H 7300 1223 50  0000 C CNN
F 2 "" H 7300 1050 50  0001 C CNN
F 3 "~" H 7300 1050 50  0001 C CNN
	1    7300 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 1050 7300 1300
Connection ~ 7300 1300
Wire Wire Line
	7300 1300 8350 1300
$Comp
L Device:C C13
U 1 1 5E36D087
P 7700 2950
F 0 "C13" H 7815 2996 50  0000 L CNN
F 1 "100n" H 7815 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7738 2800 50  0001 C CNN
F 3 "~" H 7700 2950 50  0001 C CNN
	1    7700 2950
	1    0    0    -1  
$EndComp
Connection ~ 7700 2800
Wire Wire Line
	7700 2800 7400 2800
Wire Wire Line
	7700 3100 7700 3400
Connection ~ 7700 3400
Wire Wire Line
	7400 2700 7550 2700
Wire Wire Line
	7550 2700 7550 2400
Wire Wire Line
	7550 2400 6750 2400
$Comp
L Device:C C11
U 1 1 5E373A42
P 6550 2350
F 0 "C11" H 6665 2396 50  0000 L CNN
F 1 "100n" H 6665 2305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6588 2200 50  0001 C CNN
F 3 "~" H 6550 2350 50  0001 C CNN
	1    6550 2350
	0    1    1    0   
$EndComp
$Comp
L Device:C C12
U 1 1 5E3740F9
P 6550 2550
F 0 "C12" H 6665 2596 50  0000 L CNN
F 1 "100n" H 6665 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6588 2400 50  0001 C CNN
F 3 "~" H 6550 2550 50  0001 C CNN
	1    6550 2550
	0    1    1    0   
$EndComp
Wire Wire Line
	6700 2300 6700 2350
Connection ~ 6700 2350
Wire Wire Line
	6400 2350 6400 2550
Wire Wire Line
	6700 2350 6700 2550
Connection ~ 6700 2550
Connection ~ 6400 2550
Wire Wire Line
	6700 2550 6700 2600
Wire Wire Line
	6750 2400 6750 2600
Wire Wire Line
	6750 2600 6700 2600
Connection ~ 6700 2600
Wire Wire Line
	6700 2600 6700 2650
$Comp
L Device:CP C14
U 1 1 5E38CB04
P 6500 2200
F 0 "C14" V 6245 2200 50  0000 C CNN
F 1 "10u" V 6336 2200 50  0000 C CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x4.9" H 6538 2050 50  0001 C CNN
F 3 "~" H 6500 2200 50  0001 C CNN
	1    6500 2200
	0    1    1    0   
$EndComp
Connection ~ 6650 2200
Wire Wire Line
	6400 2350 6350 2350
Wire Wire Line
	6350 2350 6350 2200
Connection ~ 6400 2350
Connection ~ 6350 2200
$Comp
L Device:CP C15
U 1 1 5E39B696
P 8200 2950
F 0 "C15" H 8318 2996 50  0000 L CNN
F 1 "10u" H 8318 2905 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x4.9" H 8238 2800 50  0001 C CNN
F 3 "~" H 8200 2950 50  0001 C CNN
	1    8200 2950
	1    0    0    -1  
$EndComp
Connection ~ 8200 2800
Wire Wire Line
	8200 2800 7700 2800
Wire Wire Line
	8200 3100 8200 3400
Wire Wire Line
	4350 2150 4350 2350
Wire Wire Line
	4250 5350 4250 5850
$Comp
L power:GND #PWR0101
U 1 1 5E31A97D
P 2950 1450
F 0 "#PWR0101" H 2950 1200 50  0001 C CNN
F 1 "GND" H 2955 1277 50  0000 C CNN
F 2 "" H 2950 1450 50  0001 C CNN
F 3 "" H 2950 1450 50  0001 C CNN
	1    2950 1450
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5E33DD70
P 7400 5800
F 0 "#PWR0102" H 7400 5550 50  0001 C CNN
F 1 "GND" H 7405 5627 50  0000 C CNN
F 2 "" H 7400 5800 50  0001 C CNN
F 3 "" H 7400 5800 50  0001 C CNN
	1    7400 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 3400 7400 5800
$Comp
L power:+3.3V #PWR0103
U 1 1 5E394F91
P 4100 1700
F 0 "#PWR0103" H 4100 1550 50  0001 C CNN
F 1 "+3.3V" H 4115 1873 50  0000 C CNN
F 2 "" H 4100 1700 50  0001 C CNN
F 3 "" H 4100 1700 50  0001 C CNN
	1    4100 1700
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0104
U 1 1 5E39A0A9
P 3450 1750
F 0 "#PWR0104" H 3450 1600 50  0001 C CNN
F 1 "+3.3V" H 3465 1923 50  0000 C CNN
F 2 "" H 3450 1750 50  0001 C CNN
F 3 "" H 3450 1750 50  0001 C CNN
	1    3450 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0106
U 1 1 5E3A549B
P 4600 1550
F 0 "#PWR0106" H 4600 1400 50  0001 C CNN
F 1 "+3.3V" H 4615 1723 50  0000 C CNN
F 2 "" H 4600 1550 50  0001 C CNN
F 3 "" H 4600 1550 50  0001 C CNN
	1    4600 1550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1600 6500 1550 6500
Wire Wire Line
	4250 1450 3800 1450
$Comp
L power:GND #PWR0108
U 1 1 5E3BA20B
P 2400 5650
F 0 "#PWR0108" H 2400 5400 50  0001 C CNN
F 1 "GND" H 2405 5477 50  0000 C CNN
F 2 "" H 2400 5650 50  0001 C CNN
F 3 "" H 2400 5650 50  0001 C CNN
	1    2400 5650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2950 1450 3150 1450
Wire Wire Line
	4050 2350 3450 2350
Wire Wire Line
	3450 2350 3450 2200
Wire Wire Line
	3450 2200 3450 2000
Connection ~ 3450 2200
Wire Wire Line
	3450 2000 3450 1750
Connection ~ 3450 2000
Wire Wire Line
	4100 1700 4100 1800
Wire Wire Line
	4100 1800 4100 2000
Connection ~ 4100 1800
Wire Wire Line
	4100 2000 4100 2200
Connection ~ 4100 2000
Wire Wire Line
	4100 2200 4100 2250
Wire Wire Line
	4100 2250 4200 2250
Wire Wire Line
	4200 2250 4200 2300
Wire Wire Line
	4200 2300 4250 2300
Wire Wire Line
	4250 2300 4250 2350
Connection ~ 4100 2200
Wire Wire Line
	4200 2300 4150 2300
Wire Wire Line
	4150 2300 4150 2350
Connection ~ 4200 2300
Wire Wire Line
	4550 1600 4600 1600
Wire Wire Line
	4600 1550 4600 1600
Wire Wire Line
	4600 1600 4600 2350
Connection ~ 4600 1600
Wire Wire Line
	4600 2350 4450 2350
Wire Wire Line
	3350 4250 3250 4250
Wire Wire Line
	3350 4050 2300 4050
$Comp
L Device:R R8
U 1 1 5E3EB753
P 8850 1950
F 0 "R8" V 8643 1950 50  0000 C CNN
F 1 "2k" V 8734 1950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8780 1950 50  0001 C CNN
F 3 "~" H 8850 1950 50  0001 C CNN
	1    8850 1950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9000 1950 9150 1950
Wire Wire Line
	9150 1950 9150 1900
Wire Wire Line
	8700 1950 8700 1700
Wire Wire Line
	8700 1700 8750 1700
$Comp
L Device:R R9
U 1 1 5E40C3CB
P 9550 1950
F 0 "R9" V 9343 1950 50  0000 C CNN
F 1 "470" V 9434 1950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9480 1950 50  0001 C CNN
F 3 "~" H 9550 1950 50  0001 C CNN
	1    9550 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 1800 9550 1700
Wire Wire Line
	9550 2200 9550 2100
$Comp
L power:GND #PWR0112
U 1 1 5E4178ED
P 9150 1950
F 0 "#PWR0112" H 9150 1700 50  0001 C CNN
F 1 "GND" H 9155 1777 50  0000 C CNN
F 2 "" H 9150 1950 50  0001 C CNN
F 3 "" H 9150 1950 50  0001 C CNN
	1    9150 1950
	1    0    0    -1  
$EndComp
Connection ~ 9150 1950
Text Label 9650 1500 0    50   ~ 0
output_to_battery
$Comp
L Device:C C17
U 1 1 5E43DB6D
P 9700 1650
F 0 "C17" H 9815 1696 50  0000 L CNN
F 1 "4.7u" H 9815 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9738 1500 50  0001 C CNN
F 3 "~" H 9700 1650 50  0001 C CNN
	1    9700 1650
	-1   0    0    1   
$EndComp
Connection ~ 9700 1500
Wire Wire Line
	9700 1500 9550 1500
$Comp
L power:GND #PWR0114
U 1 1 5E43DF37
P 9700 1900
F 0 "#PWR0114" H 9700 1650 50  0001 C CNN
F 1 "GND" H 9705 1727 50  0000 C CNN
F 2 "" H 9700 1900 50  0001 C CNN
F 3 "" H 9700 1900 50  0001 C CNN
	1    9700 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 1900 9700 1850
Text Label 3150 3150 2    50   ~ 0
3DF(3D-FiX_indicator)
Wire Wire Line
	2950 3050 3350 3050
Wire Wire Line
	3150 3150 3350 3150
Text Label 4900 6800 0    50   ~ 0
temperature
Text Label 2850 3250 2    50   ~ 0
temperature
Wire Wire Line
	2850 3250 3350 3250
$Comp
L Device:LED D1
U 1 1 5E40DD7F
P 5400 2400
F 0 "D1" H 5393 2616 50  0000 C CNN
F 1 "LED" H 5393 2525 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5400 2400 50  0001 C CNN
F 3 "~" H 5400 2400 50  0001 C CNN
	1    5400 2400
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x03_Female J4
U 1 1 5E41D872
P 10600 2200
F 0 "J4" H 10492 1875 50  0000 C CNN
F 1 "Conn_01x03_Female" H 10492 1966 50  0000 C CNN
F 2 "Connector_JST:JST_PH_B3B-PH-K_1x03_P2.00mm_Vertical" H 10600 2200 50  0001 C CNN
F 3 "~" H 10600 2200 50  0001 C CNN
	1    10600 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 1500 9950 1500
$Comp
L power:GND #PWR0117
U 1 1 5E43C4C2
P 10200 2200
F 0 "#PWR0117" H 10200 1950 50  0001 C CNN
F 1 "GND" V 10205 2072 50  0000 R CNN
F 2 "" H 10200 2200 50  0001 C CNN
F 3 "" H 10200 2200 50  0001 C CNN
	1    10200 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	10200 2200 10400 2200
Text GLabel 10400 2300 0    50   Input ~ 0
no_vonnect
$Comp
L Device:CP C20
U 1 1 5E4732F3
P 9950 1650
F 0 "C20" H 10068 1696 50  0000 L CNN
F 1 "10u" H 10068 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9988 1500 50  0001 C CNN
F 3 "~" H 9950 1650 50  0001 C CNN
	1    9950 1650
	1    0    0    -1  
$EndComp
Connection ~ 9950 1500
Wire Wire Line
	9950 1500 10300 1500
Wire Wire Line
	9950 1800 9950 1850
Wire Wire Line
	9950 1850 9700 1850
Connection ~ 9700 1850
Wire Wire Line
	9700 1850 9700 1800
Wire Wire Line
	10300 2100 10400 2100
Wire Wire Line
	5400 2850 5400 2950
Wire Wire Line
	5400 2950 5150 2950
$Comp
L power:GND #PWR0118
U 1 1 5E3FE89A
P 5400 2250
F 0 "#PWR0118" H 5400 2000 50  0001 C CNN
F 1 "GND" H 5405 2077 50  0000 C CNN
F 2 "" H 5400 2250 50  0001 C CNN
F 3 "" H 5400 2250 50  0001 C CNN
	1    5400 2250
	-1   0    0    1   
$EndComp
Text Label 2000 6800 0    50   ~ 0
SWDIO
Wire Wire Line
	2000 6800 1550 6800
Text Label 2000 3950 2    50   ~ 0
SWDIO
Wire Wire Line
	2000 3950 3350 3950
Text Label 1900 6550 0    50   ~ 0
SWCLK
Wire Wire Line
	1900 6550 1650 6550
Wire Wire Line
	1650 6550 1650 6600
Text Label 1900 4100 0    50   ~ 0
SWCLK
Wire Wire Line
	1900 4100 2300 4100
Wire Wire Line
	2300 4050 2300 4100
$Comp
L Device:R R7
U 1 1 5E4ABBE7
P 6100 4700
F 0 "R7" V 5893 4700 50  0000 C CNN
F 1 "10k" V 5984 4700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6030 4700 50  0001 C CNN
F 3 "~" H 6100 4700 50  0001 C CNN
	1    6100 4700
	0    -1   1    0   
$EndComp
$Comp
L power:+3.3V #PWR0119
U 1 1 5E4B4F97
P 5550 4700
F 0 "#PWR0119" H 5550 4550 50  0001 C CNN
F 1 "+3.3V" H 5565 4873 50  0000 C CNN
F 2 "" H 5550 4700 50  0001 C CNN
F 3 "" H 5550 4700 50  0001 C CNN
	1    5550 4700
	0    -1   1    0   
$EndComp
Wire Wire Line
	5950 4500 6050 4500
Wire Wire Line
	6050 4500 6050 3050
Wire Wire Line
	5150 3050 6050 3050
$Comp
L Device:R R6
U 1 1 5E4D68C6
P 5900 4000
F 0 "R6" V 5693 4000 50  0000 C CNN
F 1 "10k" V 5784 4000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5830 4000 50  0001 C CNN
F 3 "~" H 5900 4000 50  0001 C CNN
	1    5900 4000
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5E4AC924
P 6250 4700
F 0 "#PWR0120" H 6250 4450 50  0001 C CNN
F 1 "GND" H 6255 4527 50  0000 C CNN
F 2 "" H 6250 4700 50  0001 C CNN
F 3 "" H 6250 4700 50  0001 C CNN
	1    6250 4700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 5E4DF6DC
P 5900 4150
F 0 "#PWR0121" H 5900 3900 50  0001 C CNN
F 1 "GND" H 5905 3977 50  0000 C CNN
F 2 "" H 5900 4150 50  0001 C CNN
F 3 "" H 5900 4150 50  0001 C CNN
	1    5900 4150
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0122
U 1 1 5E4DFE2E
P 5500 3850
F 0 "#PWR0122" H 5500 3700 50  0001 C CNN
F 1 "+3.3V" H 5515 4023 50  0000 C CNN
F 2 "" H 5500 3850 50  0001 C CNN
F 3 "" H 5500 3850 50  0001 C CNN
	1    5500 3850
	0    -1   1    0   
$EndComp
Wire Wire Line
	5900 3650 6000 3650
Wire Wire Line
	6000 3650 6000 3150
Wire Wire Line
	6000 3150 5150 3150
$Comp
L Device:R R5
U 1 1 5E3FF3F8
P 5400 2700
F 0 "R5" V 5193 2700 50  0000 C CNN
F 1 "470" V 5284 2700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5330 2700 50  0001 C CNN
F 3 "~" H 5400 2700 50  0001 C CNN
	1    5400 2700
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5E3E2535
P 5700 3850
F 0 "SW1" H 5700 4135 50  0000 C CNN
F 1 "SW_Push" H 5700 4044 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 5700 4050 50  0001 C CNN
F 3 "~" H 5700 4050 50  0001 C CNN
	1    5700 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 3650 5900 3850
Connection ~ 5900 3850
$Comp
L Switch:SW_Push SW3
U 1 1 5E410ECB
P 5750 4700
F 0 "SW3" H 5750 4985 50  0000 C CNN
F 1 "SW_Push" H 5750 4894 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 5750 4900 50  0001 C CNN
F 3 "~" H 5750 4900 50  0001 C CNN
	1    5750 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 4500 5950 4700
Connection ~ 5950 4700
$Comp
L RF_Module:CMWX1ZZABZ-078 U1
U 1 1 5E286E45
P 4250 3850
F 0 "U1" H 4250 2261 50  0000 C CNN
F 1 "CMWX1ZZABZ-078" H 4250 2170 50  0000 C CNN
F 2 "RF_Module:CMWX1ZZABZ" H 4250 3850 50  0001 C CNN
F 3 "https://wireless.murata.com/RFM/data/type_abz.pdf" H 6700 2350 50  0001 C CNN
	1    4250 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 4150 3250 4250
Connection ~ 3250 4250
Text Label 3250 4150 1    50   ~ 0
NRST
Wire Wire Line
	1650 4400 1600 4400
Wire Wire Line
	1950 4400 2000 4400
Wire Wire Line
	2650 4400 2650 4250
Wire Wire Line
	2650 4250 3250 4250
$Comp
L Device:C C1
U 1 1 5E453D2C
P 1800 4400
F 0 "C1" V 1548 4400 50  0000 C CNN
F 1 "0.1u" V 1639 4400 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1838 4250 50  0001 C CNN
F 3 "~" H 1800 4400 50  0001 C CNN
	1    1800 4400
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 5E45543A
P 1800 4750
F 0 "SW2" H 1800 4565 50  0000 C CNN
F 1 "SW_Push" H 1800 4656 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 1800 4950 50  0001 C CNN
F 3 "~" H 1800 4950 50  0001 C CNN
	1    1800 4750
	-1   0    0    1   
$EndComp
Wire Wire Line
	2000 4750 2000 4400
Connection ~ 2000 4400
Wire Wire Line
	2000 4400 2650 4400
Wire Wire Line
	1600 4400 1600 4750
Connection ~ 1600 4400
Wire Wire Line
	1600 4400 1350 4400
$Comp
L power:GND #PWR0105
U 1 1 5E4691FF
P 1350 4400
F 0 "#PWR0105" H 1350 4150 50  0001 C CNN
F 1 "GND" H 1355 4227 50  0000 C CNN
F 2 "" H 1350 4400 50  0001 C CNN
F 3 "" H 1350 4400 50  0001 C CNN
	1    1350 4400
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x09_Male J3
U 1 1 5E4B23A1
P 1250 2300
F 0 "J3" V 1085 2278 50  0000 C CNN
F 1 "Conn_01x09_Male" V 1176 2278 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x09_P2.54mm_Horizontal" H 1250 2300 50  0001 C CNN
F 3 "~" H 1250 2300 50  0001 C CNN
	1    1250 2300
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0110
U 1 1 5E50E564
P 1650 2700
F 0 "#PWR0110" H 1650 2550 50  0001 C CNN
F 1 "+3.3V" H 1665 2873 50  0000 C CNN
F 2 "" H 1650 2700 50  0001 C CNN
F 3 "" H 1650 2700 50  0001 C CNN
	1    1650 2700
	-1   0    0    1   
$EndComp
Wire Wire Line
	1650 2500 1650 2700
Text Label 1550 2750 1    50   ~ 0
enable
Wire Wire Line
	1550 2750 1550 2500
Text Label 1450 2800 1    50   ~ 0
VbAt
Wire Wire Line
	1450 2500 1450 2800
Wire Wire Line
	1350 2500 1350 2750
Text Label 2750 3650 0    50   ~ 0
TX
Text Label 2700 3450 0    50   ~ 0
RX
Text Label 1250 2800 1    50   ~ 0
TX
Wire Wire Line
	1250 2500 1250 2800
Text Label 1150 2850 3    50   ~ 0
RX
Wire Wire Line
	1150 2850 1150 2500
$Comp
L power:GND #PWR0107
U 1 1 5E3B9CCE
P 1750 6700
F 0 "#PWR0107" H 1750 6450 50  0001 C CNN
F 1 "GND" H 1755 6527 50  0000 C CNN
F 2 "" H 1750 6700 50  0001 C CNN
F 3 "" H 1750 6700 50  0001 C CNN
	1    1750 6700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5E5776E8
P 1050 2700
F 0 "#PWR0111" H 1050 2450 50  0001 C CNN
F 1 "GND" H 1055 2527 50  0000 C CNN
F 2 "" H 1050 2700 50  0001 C CNN
F 3 "" H 1050 2700 50  0001 C CNN
	1    1050 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 2700 1050 2500
$Comp
L power:+3.3V #PWR0123
U 1 1 5E582883
P 950 2500
F 0 "#PWR0123" H 950 2350 50  0001 C CNN
F 1 "+3.3V" H 965 2673 50  0000 C CNN
F 2 "" H 950 2500 50  0001 C CNN
F 3 "" H 950 2500 50  0001 C CNN
	1    950  2500
	-1   0    0    1   
$EndComp
Text Label 850  2500 3    50   ~ 0
1pps(1_pulse_per_second)
Text Label 1350 2750 3    50   ~ 0
3DF(3D-FiX_indicator)
Wire Wire Line
	2700 3550 2700 3450
Wire Wire Line
	2750 3650 3350 3650
Wire Wire Line
	2700 3550 3350 3550
$Comp
L Connector:Conn_Coaxial J5
U 1 1 5E5E77C5
P 6100 5200
F 0 "J5" H 6200 5175 50  0000 L CNN
F 1 "Conn_Coaxial" H 6200 5084 50  0000 L CNN
F 2 "Connector_Coaxial:U.FL_Molex_MCRF_73412-0110_Vertical" H 6100 5200 50  0001 C CNN
F 3 " ~" H 6100 5200 50  0001 C CNN
	1    6100 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5E6289FD
P 5350 5350
F 0 "C2" V 5098 5350 50  0000 C CNN
F 1 "10u" V 5189 5350 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5388 5200 50  0001 C CNN
F 3 "~" H 5350 5350 50  0001 C CNN
	1    5350 5350
	-1   0    0    1   
$EndComp
$Comp
L Device:C C22
U 1 1 5E629B32
P 5650 5350
F 0 "C22" V 5398 5350 50  0000 C CNN
F 1 "10u" V 5489 5350 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5688 5200 50  0001 C CNN
F 3 "~" H 5650 5350 50  0001 C CNN
	1    5650 5350
	-1   0    0    1   
$EndComp
Wire Wire Line
	5900 5200 5650 5200
Wire Wire Line
	5350 5200 5350 5150
Wire Wire Line
	5350 5150 5150 5150
$Comp
L power:GND #PWR0124
U 1 1 5E63F7F1
P 6000 5700
F 0 "#PWR0124" H 6000 5450 50  0001 C CNN
F 1 "GND" H 6005 5527 50  0000 C CNN
F 2 "" H 6000 5700 50  0001 C CNN
F 3 "" H 6000 5700 50  0001 C CNN
	1    6000 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 5400 6100 5700
Wire Wire Line
	6100 5700 6000 5700
Wire Wire Line
	5650 5500 5650 5700
Connection ~ 6000 5700
Wire Wire Line
	5350 5500 5350 5700
Wire Wire Line
	5350 5700 5650 5700
Connection ~ 5650 5700
Wire Wire Line
	5650 5700 6000 5700
$Comp
L Device:C C21
U 1 1 5E66414E
P 5500 5200
F 0 "C21" V 5248 5200 50  0000 C CNN
F 1 "100n" V 5650 5200 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5538 5050 50  0001 C CNN
F 3 "~" H 5500 5200 50  0001 C CNN
	1    5500 5200
	0    -1   -1   0   
$EndComp
Connection ~ 5350 5200
Connection ~ 5650 5200
Wire Wire Line
	4300 6800 4300 6850
Wire Wire Line
	4900 6800 4300 6800
$Comp
L Sensor_Temperature:TMP36xS U4
U 1 1 5E4A1C79
P 3800 6850
F 0 "U4" H 4344 6896 50  0000 L CNN
F 1 "TMP36xS" H 4344 6805 50  0000 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3800 6400 50  0001 C CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/TMP35_36_37.pdf" H 3800 6850 50  0001 C CNN
	1    3800 6850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5E4A3FDE
P 3800 7400
F 0 "#PWR0115" H 3800 7150 50  0001 C CNN
F 1 "GND" H 3805 7227 50  0000 C CNN
F 2 "" H 3800 7400 50  0001 C CNN
F 3 "" H 3800 7400 50  0001 C CNN
	1    3800 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 7400 3800 7250
$Comp
L Device:C C18
U 1 1 5E4AAE53
P 3450 6250
F 0 "C18" H 3565 6296 50  0000 L CNN
F 1 "4.7u" H 3565 6205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3488 6100 50  0001 C CNN
F 3 "~" H 3450 6250 50  0001 C CNN
	1    3450 6250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5E4AD842
P 3450 6100
F 0 "#PWR0116" H 3450 5850 50  0001 C CNN
F 1 "GND" H 3455 5927 50  0000 C CNN
F 2 "" H 3450 6100 50  0001 C CNN
F 3 "" H 3450 6100 50  0001 C CNN
	1    3450 6100
	-1   0    0    1   
$EndComp
Text Label 3250 6400 0    50   ~ 0
vin
Wire Wire Line
	3250 6400 3450 6400
Wire Wire Line
	3800 6400 3800 6450
Connection ~ 3450 6400
Wire Wire Line
	3450 6400 3650 6400
Wire Wire Line
	3650 6400 3650 6500
Wire Wire Line
	3650 6500 3300 6500
Wire Wire Line
	3300 6500 3300 6850
Connection ~ 3650 6400
Wire Wire Line
	3650 6400 3800 6400
Text Label 10400 2100 1    50   ~ 0
-------------mosfet
Connection ~ 8950 2200
Wire Wire Line
	9200 2600 9200 2500
Connection ~ 9200 2600
Wire Wire Line
	8950 2600 9200 2600
Wire Wire Line
	8950 2500 8950 2600
Wire Wire Line
	9200 2200 8950 2200
$Comp
L Device:CP C19
U 1 1 5E3EC996
P 8950 2350
F 0 "C19" H 8832 2304 50  0000 R CNN
F 1 "10u" H 8832 2395 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8988 2200 50  0001 C CNN
F 3 "~" H 8950 2350 50  0001 C CNN
	1    8950 2350
	1    0    0    -1  
$EndComp
$Comp
L Battery_Management:MCP73831-2-OT U3
U 1 1 5E3B7F3D
P 9150 1600
F 0 "U3" H 9150 2081 50  0000 C CNN
F 1 "MCP73831-2-OT" H 9150 1990 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 9200 1350 50  0001 L CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001984g.pdf" H 9000 1550 50  0001 C CNN
	1    9150 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 2650 9200 2600
$Comp
L power:GND #PWR0113
U 1 1 5E3FCED5
P 9200 2650
F 0 "#PWR0113" H 9200 2400 50  0001 C CNN
F 1 "GND" H 9205 2477 50  0000 C CNN
F 2 "" H 9200 2650 50  0001 C CNN
F 3 "" H 9200 2650 50  0001 C CNN
	1    9200 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C16
U 1 1 5E3FBD78
P 9200 2350
F 0 "C16" H 9315 2396 50  0000 L CNN
F 1 "4.7u" H 9315 2305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9238 2200 50  0001 C CNN
F 3 "~" H 9200 2350 50  0001 C CNN
	1    9200 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5E403B6F
P 9400 2200
F 0 "D2" H 9393 2416 50  0000 C CNN
F 1 "LED" H 9393 2325 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9400 2200 50  0001 C CNN
F 3 "~" H 9400 2200 50  0001 C CNN
	1    9400 2200
	-1   0    0    1   
$EndComp
Wire Wire Line
	9250 2200 9200 2200
Connection ~ 9200 2200
Wire Wire Line
	8350 2200 8950 2200
Connection ~ 8350 2200
Wire Wire Line
	8350 2200 8350 2650
Wire Wire Line
	9650 2750 9650 2900
Wire Wire Line
	8850 2900 8850 2650
Wire Wire Line
	8850 2650 8350 2650
Connection ~ 8350 2650
Wire Wire Line
	8350 2650 8350 2800
Wire Wire Line
	9150 1300 8350 1300
Connection ~ 8350 1300
Wire Wire Line
	10300 1500 10300 2100
Wire Wire Line
	10300 1500 11100 1500
Wire Wire Line
	11100 1500 11100 2750
Connection ~ 10300 1500
Wire Wire Line
	6400 3400 7400 3400
Wire Wire Line
	6400 2550 6400 3400
Wire Wire Line
	6350 2000 6400 2000
Wire Wire Line
	6350 2000 6350 2200
Wire Wire Line
	6350 2000 6300 2000
Wire Wire Line
	6300 2000 6300 1900
Connection ~ 6350 2000
Wire Wire Line
	6700 1300 7300 1300
NoConn ~ 6700 1500
NoConn ~ 6700 1600
NoConn ~ 6700 1700
NoConn ~ 5500 1050
Wire Wire Line
	7250 1750 7250 1850
$Comp
L power:+3.3V #PWR0109
U 1 1 5E350560
P 7250 1750
F 0 "#PWR0109" H 7250 1600 50  0001 C CNN
F 1 "+3.3V" H 7265 1923 50  0000 C CNN
F 2 "" H 7250 1750 50  0001 C CNN
F 3 "" H 7250 1750 50  0001 C CNN
	1    7250 1750
	1    0    0    -1  
$EndComp
Connection ~ 6750 1950
Wire Wire Line
	6750 1950 6750 2050
Wire Wire Line
	6750 1850 6750 1950
Connection ~ 7250 2150
Wire Wire Line
	7250 2150 7250 2250
Wire Wire Line
	7250 1950 7250 2150
Connection ~ 6650 2150
Wire Wire Line
	6650 2200 6650 2150
Wire Wire Line
	6650 2150 6650 2050
Wire Wire Line
	7250 1850 6750 1850
Wire Wire Line
	7050 1950 7250 1950
$Comp
L Device:R R4
U 1 1 5E2B6CCA
P 7100 2150
F 0 "R4" V 6893 2150 50  0000 C CNN
F 1 "560" V 6984 2150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7030 2150 50  0001 C CNN
F 3 "~" H 7100 2150 50  0001 C CNN
	1    7100 2150
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5E2B37A4
P 6800 2150
F 0 "R3" V 6593 2150 50  0000 C CNN
F 1 "330" V 6684 2150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6730 2150 50  0001 C CNN
F 3 "~" H 6800 2150 50  0001 C CNN
	1    6800 2150
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C9
U 1 1 5E2AFB31
P 6900 1950
F 0 "C9" H 7015 1996 50  0000 L CNN
F 1 "100n" H 7015 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6938 1800 50  0001 C CNN
F 3 "~" H 6900 1950 50  0001 C CNN
	1    6900 1950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7700 3400 8200 3400
$Comp
L power:GND #PWR0125
U 1 1 5E624AE8
P 7950 2250
F 0 "#PWR0125" H 7950 2000 50  0001 C CNN
F 1 "GND" H 7955 2077 50  0000 C CNN
F 2 "" H 7950 2250 50  0001 C CNN
F 3 "" H 7950 2250 50  0001 C CNN
	1    7950 2250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9650 2750 11100 2750
Wire Wire Line
	7250 2250 7950 2250
Wire Wire Line
	7400 2550 7400 2350
Wire Wire Line
	7400 2350 6950 2350
Wire Wire Line
	6950 2350 6950 2150
Connection ~ 6950 2150
$Comp
L Device:D_Zener D3
U 1 1 5E6BA8F4
P 9100 2900
F 0 "D3" H 9100 3116 50  0000 C CNN
F 1 "D_Zener" H 9100 3025 50  0000 C CNN
F 2 "Diode_SMD:D_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9100 2900 50  0001 C CNN
F 3 "~" H 9100 2900 50  0001 C CNN
	1    9100 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 2900 8850 2900
Wire Wire Line
	9250 2900 9650 2900
$EndSCHEMATC
