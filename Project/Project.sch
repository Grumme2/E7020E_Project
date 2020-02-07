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
$Comp
L power:+3.3V #PWR02
U 1 1 5E2B99C7
P 1600 6700
F 0 "#PWR02" H 1600 6550 50  0001 C CNN
F 1 "+3.3V" H 1615 6873 50  0000 C CNN
F 2 "" H 1600 6700 50  0001 C CNN
F 3 "" H 1600 6700 50  0001 C CNN
	1    1600 6700
	-1   0    0    1   
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
L Device:R R1
U 1 1 5E2C28BB
P 3050 4250
F 0 "R1" V 2843 4250 50  0000 C CNN
F 1 "10k" V 2934 4250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2980 4250 50  0001 C CNN
F 3 "~" H 3050 4250 50  0001 C CNN
	1    3050 4250
	0    1    1    0   
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
Wire Wire Line
	3250 4650 3350 4650
Wire Wire Line
	3250 4750 3350 4750
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
$Comp
L Device:Crystal Y1
U 1 1 5E2D2343
P 3000 4700
F 0 "Y1" V 2954 4831 50  0000 L CNN
F 1 "8Mhz" V 3045 4831 50  0000 L CNN
F 2 "Crystal:Crystal_HC49-U_Vertical" H 3000 4700 50  0001 C CNN
F 3 "~" H 3000 4700 50  0001 C CNN
	1    3000 4700
	0    1    1    0   
$EndComp
Connection ~ 3800 2000
Wire Wire Line
	3800 2000 3800 2200
Wire Wire Line
	2400 5650 2400 5150
Wire Wire Line
	3000 5150 2400 5150
Connection ~ 2400 5150
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
Wire Wire Line
	3250 4650 3250 4550
Wire Wire Line
	3250 4550 3000 4550
Wire Wire Line
	3250 4750 3250 4850
Wire Wire Line
	3250 4850 3000 4850
$Comp
L Device:C C2
U 1 1 5E32C578
P 2850 4850
F 0 "C2" V 2598 4850 50  0000 C CNN
F 1 "18p" V 2689 4850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2888 4700 50  0001 C CNN
F 3 "~" H 2850 4850 50  0001 C CNN
	1    2850 4850
	0    1    1    0   
$EndComp
Connection ~ 3000 4850
Wire Wire Line
	2700 4850 2400 4850
Connection ~ 2400 4850
Wire Wire Line
	2400 4850 2400 5150
$Comp
L Device:C C1
U 1 1 5E331296
P 2850 4550
F 0 "C1" V 2598 4550 50  0000 C CNN
F 1 "18p" V 2689 4550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2888 4400 50  0001 C CNN
F 3 "~" H 2850 4550 50  0001 C CNN
	1    2850 4550
	0    1    1    0   
$EndComp
Connection ~ 3000 4550
Wire Wire Line
	2700 4550 2400 4550
Wire Wire Line
	2400 4550 2400 4850
$Comp
L Connector:Conn_01x05_Male J1
U 1 1 5E340471
P 1850 6500
F 0 "J1" H 1822 6432 50  0000 R CNN
F 1 "Conn_01x05_Male" H 1822 6523 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 1850 6500 50  0001 C CNN
F 3 "~" H 1850 6500 50  0001 C CNN
	1    1850 6500
	-1   0    0    1   
$EndComp
Wire Wire Line
	1650 6500 1450 6500
Wire Wire Line
	1550 6600 1650 6600
Text Label 1500 6300 2    50   ~ 0
NRST
Wire Wire Line
	1500 6300 1650 6300
$Comp
L LM117ADJ:LM1117ADJ U2
U 1 1 5E29C2E5
P 9350 2550
F 0 "U2" H 9400 2875 50  0000 C CNN
F 1 "LM1117ADJ" H 9400 2784 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223" H 9350 2550 50  0001 C CNN
F 3 "" H 9350 2550 50  0001 C CNN
	1    9350 2550
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_OTG J2
U 1 1 5E2A6C12
P 8800 1500
F 0 "J2" H 8857 1967 50  0000 C CNN
F 1 "USB_OTG" H 8857 1876 50  0000 C CNN
F 2 "Connector_USB:USB_Mini-B_Wuerth_65100516121_Horizontal" H 8950 1450 50  0001 C CNN
F 3 " ~" H 8950 1450 50  0001 C CNN
	1    8800 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5E2AFB31
P 9200 1950
F 0 "C9" H 9315 1996 50  0000 L CNN
F 1 "100n" H 9315 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9238 1800 50  0001 C CNN
F 3 "~" H 9200 1950 50  0001 C CNN
	1    9200 1950
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 5E2B37A4
P 9100 2150
F 0 "R3" V 8893 2150 50  0000 C CNN
F 1 "10k" V 8984 2150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9030 2150 50  0001 C CNN
F 3 "~" H 9100 2150 50  0001 C CNN
	1    9100 2150
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 5E2B6CCA
P 9400 2150
F 0 "R4" V 9193 2150 50  0000 C CNN
F 1 "3.3k" V 9284 2150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9330 2150 50  0001 C CNN
F 3 "~" H 9400 2150 50  0001 C CNN
	1    9400 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	9000 2300 8950 2300
Wire Wire Line
	8950 2300 8950 2200
Wire Wire Line
	8950 2150 8950 2050
Wire Wire Line
	8950 2050 9050 2050
Connection ~ 8950 2150
Wire Wire Line
	9350 1950 9550 1950
Wire Wire Line
	9550 1950 9550 2150
Wire Wire Line
	9550 2150 9550 2250
Wire Wire Line
	9550 2250 9700 2250
Connection ~ 9550 2150
Wire Wire Line
	9250 2150 9250 2250
Connection ~ 9250 2150
$Comp
L Device:C C10
U 1 1 5E2D92D2
P 9700 2950
F 0 "C10" H 9815 2996 50  0000 L CNN
F 1 "10n" H 9815 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9738 2800 50  0001 C CNN
F 3 "~" H 9700 2950 50  0001 C CNN
	1    9700 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 3400 10000 3400
Wire Wire Line
	10250 3400 10250 2250
Wire Wire Line
	10250 2250 9700 2250
Connection ~ 9700 2250
Wire Wire Line
	9700 2250 9700 2550
Wire Wire Line
	9250 2250 9550 2250
Connection ~ 9550 2250
Wire Wire Line
	9700 3100 9700 3400
Connection ~ 9700 3400
Wire Wire Line
	10650 1300 10650 2800
Wire Wire Line
	10650 2800 10500 2800
Wire Wire Line
	9100 1300 9200 1300
Connection ~ 9700 2800
NoConn ~ 5150 3050
NoConn ~ 5150 2950
NoConn ~ 5150 3150
NoConn ~ 5150 3250
NoConn ~ 5150 3350
NoConn ~ 5150 3450
NoConn ~ 5150 3550
NoConn ~ 5150 3650
NoConn ~ 5150 3750
NoConn ~ 5150 3850
NoConn ~ 9100 1700
NoConn ~ 9100 1600
NoConn ~ 9100 1500
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
NoConn ~ 5150 5150
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
	9700 3400 8750 3400
Wire Wire Line
	8800 2000 8800 1900
Wire Wire Line
	8700 1900 8700 2000
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5E35EA03
P 9600 1050
F 0 "#FLG0101" H 9600 1125 50  0001 C CNN
F 1 "PWR_FLAG" H 9600 1223 50  0000 C CNN
F 2 "" H 9600 1050 50  0001 C CNN
F 3 "~" H 9600 1050 50  0001 C CNN
	1    9600 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 1050 9600 1300
Connection ~ 9600 1300
Wire Wire Line
	9600 1300 10650 1300
$Comp
L Device:C C13
U 1 1 5E36D087
P 10000 2950
F 0 "C13" H 10115 2996 50  0000 L CNN
F 1 "100n" H 10115 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10038 2800 50  0001 C CNN
F 3 "~" H 10000 2950 50  0001 C CNN
	1    10000 2950
	1    0    0    -1  
$EndComp
Connection ~ 10000 2800
Wire Wire Line
	10000 2800 9700 2800
Wire Wire Line
	10000 3100 10000 3400
Connection ~ 10000 3400
Wire Wire Line
	10000 3400 10250 3400
Wire Wire Line
	9700 2700 9850 2700
Wire Wire Line
	9850 2700 9850 2400
Wire Wire Line
	9850 2400 9050 2400
$Comp
L Device:C C11
U 1 1 5E373A42
P 8850 2350
F 0 "C11" H 8965 2396 50  0000 L CNN
F 1 "100n" H 8965 2305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8888 2200 50  0001 C CNN
F 3 "~" H 8850 2350 50  0001 C CNN
	1    8850 2350
	0    1    1    0   
$EndComp
$Comp
L Device:C C12
U 1 1 5E3740F9
P 8850 2550
F 0 "C12" H 8965 2596 50  0000 L CNN
F 1 "100n" H 8965 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8888 2400 50  0001 C CNN
F 3 "~" H 8850 2550 50  0001 C CNN
	1    8850 2550
	0    1    1    0   
$EndComp
Wire Wire Line
	8700 2000 8750 2000
Wire Wire Line
	8750 3400 8750 3050
Wire Wire Line
	8750 3050 8700 3050
Wire Wire Line
	8750 2100 8750 2000
Connection ~ 8750 2000
Wire Wire Line
	8750 2000 8800 2000
Wire Wire Line
	9000 2300 9000 2350
Connection ~ 9000 2350
Wire Wire Line
	8700 2350 8700 2550
Wire Wire Line
	9000 2350 9000 2550
Connection ~ 9000 2550
Connection ~ 8700 2550
Wire Wire Line
	9000 2550 9000 2600
Wire Wire Line
	9050 2400 9050 2600
Wire Wire Line
	9050 2600 9000 2600
Wire Wire Line
	8700 2550 8700 3050
Connection ~ 9000 2600
Wire Wire Line
	9000 2600 9000 2650
$Comp
L Device:CP C14
U 1 1 5E38CB04
P 8800 2200
F 0 "C14" V 8545 2200 50  0000 C CNN
F 1 "10u" V 8636 2200 50  0000 C CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x4.9" H 8838 2050 50  0001 C CNN
F 3 "~" H 8800 2200 50  0001 C CNN
	1    8800 2200
	0    1    1    0   
$EndComp
Connection ~ 8950 2200
Wire Wire Line
	8950 2200 8950 2150
Wire Wire Line
	8700 2350 8650 2350
Wire Wire Line
	8650 2350 8650 2200
Connection ~ 8700 2350
Wire Wire Line
	8650 2200 8650 2100
Wire Wire Line
	8650 2100 8750 2100
Connection ~ 8650 2200
$Comp
L Device:CP C15
U 1 1 5E39B696
P 10500 2950
F 0 "C15" H 10618 2996 50  0000 L CNN
F 1 "10u" H 10618 2905 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x4.9" H 10538 2800 50  0001 C CNN
F 3 "~" H 10500 2950 50  0001 C CNN
	1    10500 2950
	1    0    0    -1  
$EndComp
Connection ~ 10500 2800
Wire Wire Line
	10500 2800 10000 2800
Wire Wire Line
	10500 3100 10500 3400
Wire Wire Line
	10500 3400 10250 3400
Connection ~ 10250 3400
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
P 9700 5800
F 0 "#PWR0102" H 9700 5550 50  0001 C CNN
F 1 "GND" H 9705 5627 50  0000 C CNN
F 2 "" H 9700 5800 50  0001 C CNN
F 3 "" H 9700 5800 50  0001 C CNN
	1    9700 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 3400 9700 5800
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
L power:+3.3V #PWR0105
U 1 1 5E39EFEC
P 2600 4250
F 0 "#PWR0105" H 2600 4100 50  0001 C CNN
F 1 "+3.3V" H 2615 4423 50  0000 C CNN
F 2 "" H 2600 4250 50  0001 C CNN
F 3 "" H 2600 4250 50  0001 C CNN
	1    2600 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 4250 2600 4250
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
	1600 6700 1650 6700
Wire Wire Line
	4250 1450 3800 1450
$Comp
L power:GND #PWR0107
U 1 1 5E3B9CCE
P 1450 6500
F 0 "#PWR0107" H 1450 6250 50  0001 C CNN
F 1 "GND" H 1455 6327 50  0000 C CNN
F 2 "" H 1450 6500 50  0001 C CNN
F 3 "" H 1450 6500 50  0001 C CNN
	1    1450 6500
	0    1    1    0   
$EndComp
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
$Comp
L power:+3.3V #PWR0109
U 1 1 5E350560
P 9550 1750
F 0 "#PWR0109" H 9550 1600 50  0001 C CNN
F 1 "+3.3V" H 9565 1923 50  0000 C CNN
F 2 "" H 9550 1750 50  0001 C CNN
F 3 "" H 9550 1750 50  0001 C CNN
	1    9550 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 1750 9550 1850
Wire Wire Line
	9550 1850 9050 1850
Wire Wire Line
	9050 1850 9050 1950
Connection ~ 9050 1950
Wire Wire Line
	9050 1950 9050 2050
$Comp
L Connector:Conn_01x06_Female J3
U 1 1 5E3AC257
P 1450 3550
F 0 "J3" H 1342 3025 50  0000 C CNN
F 1 "Conn_01x06_Female" H 1342 3116 50  0000 C CNN
F 2 "" H 1450 3550 50  0001 C CNN
F 3 "~" H 1450 3550 50  0001 C CNN
	1    1450 3550
	-1   0    0    1   
$EndComp
Wire Wire Line
	1650 3650 3350 3650
Wire Wire Line
	3350 3550 1650 3550
$Comp
L power:+3.3V #PWR0110
U 1 1 5E3B7739
P 2050 3250
F 0 "#PWR0110" H 2050 3100 50  0001 C CNN
F 1 "+3.3V" H 2065 3423 50  0000 C CNN
F 2 "" H 2050 3250 50  0001 C CNN
F 3 "" H 2050 3250 50  0001 C CNN
	1    2050 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 3250 1650 3250
$Comp
L power:GND #PWR0111
U 1 1 5E3C0420
P 2150 3350
F 0 "#PWR0111" H 2150 3100 50  0001 C CNN
F 1 "GND" H 2155 3177 50  0000 C CNN
F 2 "" H 2150 3350 50  0001 C CNN
F 3 "" H 2150 3350 50  0001 C CNN
	1    2150 3350
	0    -1   1    0   
$EndComp
Wire Wire Line
	2150 3350 1650 3350
Connection ~ 3250 4250
Wire Wire Line
	3250 4250 3200 4250
Wire Wire Line
	3350 4250 3250 4250
Wire Wire Line
	3250 4150 3250 4250
Wire Wire Line
	3350 4050 2300 4050
Text Label 3250 4150 1    50   ~ 0
NRST
Text Label 1850 3450 0    50   ~ 0
1pps(1_pulse_per_second)
Text Label 1950 3750 0    50   ~ 0
3DF(3D-FiX_indicator)
Wire Wire Line
	1950 3750 1650 3750
Wire Wire Line
	1650 3450 1850 3450
$Comp
L Device:R R8
U 1 1 5E3EB753
P 6550 1800
F 0 "R8" V 6343 1800 50  0000 C CNN
F 1 "2k" V 6434 1800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6480 1800 50  0001 C CNN
F 3 "~" H 6550 1800 50  0001 C CNN
	1    6550 1800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6700 1800 6850 1800
Wire Wire Line
	6850 1800 6850 1750
Wire Wire Line
	6400 1800 6400 1550
Wire Wire Line
	6400 1550 6450 1550
$Comp
L Device:LED D2
U 1 1 5E403B6F
P 6700 1150
F 0 "D2" H 6693 1366 50  0000 C CNN
F 1 "LED" H 6693 1275 50  0000 C CNN
F 2 "" H 6700 1150 50  0001 C CNN
F 3 "~" H 6700 1150 50  0001 C CNN
	1    6700 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5E40C3CB
P 7250 1800
F 0 "R9" V 7043 1800 50  0000 C CNN
F 1 "470" V 7134 1800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7180 1800 50  0001 C CNN
F 3 "~" H 7250 1800 50  0001 C CNN
	1    7250 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 1650 7250 1550
Wire Wire Line
	6550 1150 6200 1150
Wire Wire Line
	6200 1150 6200 2050
Wire Wire Line
	6200 2050 7250 2050
Wire Wire Line
	7250 2050 7250 1950
$Comp
L power:GND #PWR0112
U 1 1 5E4178ED
P 6850 1800
F 0 "#PWR0112" H 6850 1550 50  0001 C CNN
F 1 "GND" H 6855 1627 50  0000 C CNN
F 2 "" H 6850 1800 50  0001 C CNN
F 3 "" H 6850 1800 50  0001 C CNN
	1    6850 1800
	1    0    0    -1  
$EndComp
Connection ~ 6850 1800
Text Label 7350 1350 0    50   ~ 0
output_to_battery
Connection ~ 6850 1150
Text Label 8350 900  0    50   ~ 0
vin_from_usb??
$Comp
L Device:C C16
U 1 1 5E3FBD78
P 7400 1000
F 0 "C16" H 7515 1046 50  0000 L CNN
F 1 "4.7u" H 7515 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7438 850 50  0001 C CNN
F 3 "~" H 7400 1000 50  0001 C CNN
	1    7400 1000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 5E3FCED5
P 7400 700
F 0 "#PWR0113" H 7400 450 50  0001 C CNN
F 1 "GND" H 7405 527 50  0000 C CNN
F 2 "" H 7400 700 50  0001 C CNN
F 3 "" H 7400 700 50  0001 C CNN
	1    7400 700 
	-1   0    0    1   
$EndComp
Wire Wire Line
	7400 700  7400 750 
Wire Wire Line
	6850 1150 7400 1150
Connection ~ 7400 1150
$Comp
L Battery_Management:MCP73831-2-OT U3
U 1 1 5E3B7F3D
P 6850 1450
F 0 "U3" H 6850 1931 50  0000 C CNN
F 1 "MCP73831-2-OT" H 6850 1840 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 6900 1200 50  0001 L CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001984g.pdf" H 6700 1400 50  0001 C CNN
	1    6850 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C17
U 1 1 5E43DB6D
P 7400 1500
F 0 "C17" H 7515 1546 50  0000 L CNN
F 1 "4.7u" H 7515 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7438 1350 50  0001 C CNN
F 3 "~" H 7400 1500 50  0001 C CNN
	1    7400 1500
	-1   0    0    1   
$EndComp
Connection ~ 7400 1350
Wire Wire Line
	7400 1350 7250 1350
$Comp
L power:GND #PWR0114
U 1 1 5E43DF37
P 7400 1750
F 0 "#PWR0114" H 7400 1500 50  0001 C CNN
F 1 "GND" H 7405 1577 50  0000 C CNN
F 2 "" H 7400 1750 50  0001 C CNN
F 3 "" H 7400 1750 50  0001 C CNN
	1    7400 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 1750 7400 1700
$Comp
L Sensor_Temperature:TMP36xS U4
U 1 1 5E4A1C79
P 7750 4300
F 0 "U4" H 8294 4346 50  0000 L CNN
F 1 "TMP36xS" H 8294 4255 50  0000 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 7750 3850 50  0001 C CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/TMP35_36_37.pdf" H 7750 4300 50  0001 C CNN
	1    7750 4300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5E4A3FDE
P 7750 4850
F 0 "#PWR0115" H 7750 4600 50  0001 C CNN
F 1 "GND" H 7755 4677 50  0000 C CNN
F 2 "" H 7750 4850 50  0001 C CNN
F 3 "" H 7750 4850 50  0001 C CNN
	1    7750 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 4850 7750 4700
$Comp
L Device:C C18
U 1 1 5E4AAE53
P 7400 3700
F 0 "C18" H 7515 3746 50  0000 L CNN
F 1 "4.7u" H 7515 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7438 3550 50  0001 C CNN
F 3 "~" H 7400 3700 50  0001 C CNN
	1    7400 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5E4AD842
P 7400 3550
F 0 "#PWR0116" H 7400 3300 50  0001 C CNN
F 1 "GND" H 7405 3377 50  0000 C CNN
F 2 "" H 7400 3550 50  0001 C CNN
F 3 "" H 7400 3550 50  0001 C CNN
	1    7400 3550
	-1   0    0    1   
$EndComp
Text Label 7200 3850 0    50   ~ 0
vin
Wire Wire Line
	7200 3850 7400 3850
Wire Wire Line
	7750 3850 7750 3900
Connection ~ 7400 3850
Wire Wire Line
	7400 3850 7600 3850
Wire Wire Line
	7600 3850 7600 3950
Wire Wire Line
	7600 3950 7250 3950
Wire Wire Line
	7250 3950 7250 4300
Connection ~ 7600 3850
Wire Wire Line
	7600 3850 7750 3850
Text Label 2950 3050 2    50   ~ 0
1pps(1_pulse_per_second)
Text Label 3150 3150 2    50   ~ 0
3DF(3D-FiX_indicator)
Wire Wire Line
	2950 3050 3350 3050
Wire Wire Line
	3150 3150 3350 3150
Text Label 8850 4250 0    50   ~ 0
temperature
Wire Wire Line
	8850 4250 8250 4250
Wire Wire Line
	8250 4250 8250 4300
Text Label 2850 3250 2    50   ~ 0
temperature
Wire Wire Line
	2850 3250 3350 3250
$Comp
L Switch:SW_Push_Dual SW2
U 1 1 5E40AB71
P 5750 4500
F 0 "SW2" H 5750 4785 50  0000 C CNN
F 1 "SW_Push_Dual" H 5750 4694 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 5750 4700 50  0001 C CNN
F 3 "~" H 5750 4700 50  0001 C CNN
	1    5750 4500
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push_Dual SW1
U 1 1 5E40D579
P 5700 3650
F 0 "SW1" H 5700 3935 50  0000 C CNN
F 1 "SW_Push_Dual" H 5700 3844 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 5700 3850 50  0001 C CNN
F 3 "~" H 5700 3850 50  0001 C CNN
	1    5700 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5E40DD7F
P 5400 2400
F 0 "D1" H 5393 2616 50  0000 C CNN
F 1 "LED" H 5393 2525 50  0000 C CNN
F 2 "" H 5400 2400 50  0001 C CNN
F 3 "~" H 5400 2400 50  0001 C CNN
	1    5400 2400
	0    1    1    0   
$EndComp
$Comp
L Device:CP C19
U 1 1 5E3EC996
P 7650 1000
F 0 "C19" H 7532 954 50  0000 R CNN
F 1 "10u" H 7532 1045 50  0000 R CNN
F 2 "" H 7688 850 50  0001 C CNN
F 3 "~" H 7650 1000 50  0001 C CNN
	1    7650 1000
	-1   0    0    1   
$EndComp
Wire Wire Line
	7400 1150 7650 1150
Wire Wire Line
	7650 850  7650 750 
Wire Wire Line
	7650 750  7400 750 
Connection ~ 7400 750 
Wire Wire Line
	7400 750  7400 850 
Wire Wire Line
	7650 1150 8350 1150
Wire Wire Line
	8350 1150 8350 900 
Wire Wire Line
	8350 900  9200 900 
Wire Wire Line
	9200 900  9200 1300
Connection ~ 7650 1150
Connection ~ 9200 1300
Wire Wire Line
	9200 1300 9600 1300
$Comp
L Connector:Conn_01x03_Female J4
U 1 1 5E41D872
P 8300 2050
F 0 "J4" H 8192 1725 50  0000 C CNN
F 1 "Conn_01x03_Female" H 8192 1816 50  0000 C CNN
F 2 "" H 8300 2050 50  0001 C CNN
F 3 "~" H 8300 2050 50  0001 C CNN
	1    8300 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 1350 7650 1350
$Comp
L power:GND #PWR0117
U 1 1 5E43C4C2
P 7900 2050
F 0 "#PWR0117" H 7900 1800 50  0001 C CNN
F 1 "GND" V 7905 1922 50  0000 R CNN
F 2 "" H 7900 2050 50  0001 C CNN
F 3 "" H 7900 2050 50  0001 C CNN
	1    7900 2050
	0    1    1    0   
$EndComp
Wire Wire Line
	7900 2050 8100 2050
Text GLabel 8100 2150 0    50   Input ~ 0
Minus_somehow
$Comp
L Device:CP C20
U 1 1 5E4732F3
P 7650 1500
F 0 "C20" H 7768 1546 50  0000 L CNN
F 1 "10u" H 7768 1455 50  0000 L CNN
F 2 "" H 7688 1350 50  0001 C CNN
F 3 "~" H 7650 1500 50  0001 C CNN
	1    7650 1500
	1    0    0    -1  
$EndComp
Connection ~ 7650 1350
Wire Wire Line
	7650 1350 8000 1350
Wire Wire Line
	7650 1650 7650 1700
Wire Wire Line
	7650 1700 7400 1700
Connection ~ 7400 1700
Wire Wire Line
	7400 1700 7400 1650
Wire Wire Line
	8000 1350 8000 1950
Wire Wire Line
	8000 1950 8100 1950
Text Label 8100 1950 1    50   ~ 0
-------------mosfet
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
Text Label 1200 6400 2    50   ~ 0
SWDIO
Wire Wire Line
	1200 6400 1650 6400
Text Label 2000 3950 2    50   ~ 0
SWDIO
Wire Wire Line
	2000 3950 3350 3950
Text Label 1300 6650 2    50   ~ 0
SWCLK
Wire Wire Line
	1300 6650 1550 6650
Wire Wire Line
	1550 6650 1550 6600
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
$EndSCHEMATC
