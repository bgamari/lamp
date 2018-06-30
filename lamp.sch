EESchema Schematic File Version 4
EELAYER 26 0
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
Text Notes 750  1200 0    50   ~ 0
4.5V <= Vin <= 15V\n2.8V <= Vout <= 3.5V\nIout = 4A
$Comp
L lamp:TPS54620 U103
U 1 1 5B36A4A5
P 7100 3850
F 0 "U103" H 7100 4365 50  0000 C CNN
F 1 "TPS54620" H 7100 4274 50  0000 C CNN
F 2 "Package_DFN_QFN:Texas_S-PVQFN-N14_ThermalVias" H 7100 3850 50  0001 C CNN
F 3 "" H 7100 3850 50  0001 C CNN
	1    7100 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 1700 1550 1700
$Comp
L power:GND #PWR0101
U 1 1 5B36A9A1
P 1400 2000
F 0 "#PWR0101" H 1400 1750 50  0001 C CNN
F 1 "GND" H 1405 1827 50  0000 C CNN
F 2 "" H 1400 2000 50  0001 C CNN
F 3 "" H 1400 2000 50  0001 C CNN
	1    1400 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 2000 1250 2000
$Comp
L power:GND #PWR0102
U 1 1 5B36A9CE
P 7100 4550
F 0 "#PWR0102" H 7100 4300 50  0001 C CNN
F 1 "GND" H 7105 4377 50  0000 C CNN
F 2 "" H 7100 4550 50  0001 C CNN
F 3 "" H 7100 4550 50  0001 C CNN
	1    7100 4550
	1    0    0    -1  
$EndComp
$Comp
L MCU_ST_STM32F0:STM32F030F4Px U101
U 1 1 5B36AAB9
P 2700 4250
F 0 "U101" H 3050 4900 50  0000 C CNN
F 1 "STM32F030F4Px" H 2650 4200 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 2300 3550 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00088500.pdf" H 2700 4250 50  0001 C CNN
	1    2700 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D101
U 1 1 5B36AC10
P 4250 3300
F 0 "D101" H 4242 3045 50  0000 C CNN
F 1 "LED" H 4242 3136 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 4250 3300 50  0001 C CNN
F 3 "~" H 4250 3300 50  0001 C CNN
	1    4250 3300
	-1   0    0    1   
$EndComp
$Comp
L Device:R R101
U 1 1 5B36ACCF
P 3850 3300
F 0 "R101" V 3643 3300 50  0000 C CNN
F 1 "1k" V 3734 3300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3780 3300 50  0001 C CNN
F 3 "~" H 3850 3300 50  0001 C CNN
	1    3850 3300
	0    1    1    0   
$EndComp
$Comp
L Device:L L101
U 1 1 5B36ADF2
P 8450 3850
F 0 "L101" V 8640 3850 50  0000 C CNN
F 1 "3.3u" V 8549 3850 50  0000 C CNN
F 2 "Inductor_SMD:L_Vishay_IHLP-1616" H 8450 3850 50  0001 C CNN
F 3 "~" H 8450 3850 50  0001 C CNN
	1    8450 3850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7750 3600 7800 3600
Text Label 1550 1700 2    50   ~ 0
VIN
Text Label 5400 3950 0    50   ~ 0
VIN
$Comp
L Device:R R103
U 1 1 5B36B0AB
P 6100 3650
F 0 "R103" V 5893 3650 50  0000 C CNN
F 1 "R" V 5984 3650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6030 3650 50  0001 C CNN
F 3 "~" H 6100 3650 50  0001 C CNN
	1    6100 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	6250 3650 6450 3650
$Comp
L power:GND #PWR0103
U 1 1 5B36B1D7
P 5900 3650
F 0 "#PWR0103" H 5900 3400 50  0001 C CNN
F 1 "GND" V 5905 3522 50  0000 R CNN
F 2 "" H 5900 3650 50  0001 C CNN
F 3 "" H 5900 3650 50  0001 C CNN
	1    5900 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	5900 3650 5950 3650
$Comp
L Device:C C107
U 1 1 5B36B26C
P 6350 4450
F 0 "C107" H 6465 4496 50  0000 L CNN
F 1 "10n" H 6465 4405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6388 4300 50  0001 C CNN
F 3 "~" H 6350 4450 50  0001 C CNN
	1    6350 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 4300 6350 4250
Wire Wire Line
	6350 4250 6450 4250
$Comp
L power:GND #PWR0104
U 1 1 5B36B2F3
P 6350 4600
F 0 "#PWR0104" H 6350 4350 50  0001 C CNN
F 1 "GND" H 6355 4427 50  0000 C CNN
F 2 "" H 6350 4600 50  0001 C CNN
F 3 "" H 6350 4600 50  0001 C CNN
	1    6350 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C110
U 1 1 5B36B3CD
P 9200 4050
F 0 "C110" H 9318 4096 50  0000 L CNN
F 1 "47u" H 9318 4005 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-6032-28_Kemet-C" H 9238 3900 50  0001 C CNN
F 3 "~" H 9200 4050 50  0001 C CNN
	1    9200 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 3850 9200 3900
$Comp
L power:GND #PWR0105
U 1 1 5B36B549
P 9200 4200
F 0 "#PWR0105" H 9200 3950 50  0001 C CNN
F 1 "GND" H 9205 4027 50  0000 C CNN
F 2 "" H 9200 4200 50  0001 C CNN
F 3 "" H 9200 4200 50  0001 C CNN
	1    9200 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 3850 8700 3850
Wire Wire Line
	7750 3850 8150 3850
$Comp
L Device:C C109
U 1 1 5B36B6D2
P 7950 3600
F 0 "C109" V 7698 3600 50  0000 C CNN
F 1 "100n" V 7789 3600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7988 3450 50  0001 C CNN
F 3 "~" H 7950 3600 50  0001 C CNN
	1    7950 3600
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 3600 8150 3600
Wire Wire Line
	8150 3600 8150 3850
Connection ~ 8150 3850
Wire Wire Line
	8150 3850 8300 3850
$Comp
L power:GND #PWR0106
U 1 1 5B36BAD2
P 9900 4400
F 0 "#PWR0106" H 9900 4150 50  0001 C CNN
F 1 "GND" H 9905 4227 50  0000 C CNN
F 2 "" H 9900 4400 50  0001 C CNN
F 3 "" H 9900 4400 50  0001 C CNN
	1    9900 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R106
U 1 1 5B36C046
P 8700 4050
F 0 "R106" H 8770 4096 50  0000 L CNN
F 1 "R" H 8770 4005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8630 4050 50  0001 C CNN
F 3 "~" H 8700 4050 50  0001 C CNN
	1    8700 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R107
U 1 1 5B36C394
P 8700 4400
F 0 "R107" H 8770 4446 50  0000 L CNN
F 1 "R" H 8770 4355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8630 4400 50  0001 C CNN
F 3 "~" H 8700 4400 50  0001 C CNN
	1    8700 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 4200 8700 4250
Wire Wire Line
	8700 4200 8350 4200
Wire Wire Line
	8100 4200 8100 4000
Wire Wire Line
	8100 4000 7750 4000
Connection ~ 8700 4200
NoConn ~ 6450 4100
Wire Wire Line
	6450 3800 6150 3800
Text Label 6150 3800 0    50   ~ 0
EN
Wire Wire Line
	4000 3300 4100 3300
$Comp
L power:GND #PWR0107
U 1 1 5B36D22B
P 4500 3300
F 0 "#PWR0107" H 4500 3050 50  0001 C CNN
F 1 "GND" V 4505 3172 50  0000 R CNN
F 2 "" H 4500 3300 50  0001 C CNN
F 3 "" H 4500 3300 50  0001 C CNN
	1    4500 3300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4500 3300 4400 3300
Wire Wire Line
	3200 3850 3650 3850
Text Label 3650 3850 2    50   ~ 0
EN
$Comp
L Regulator_Linear:MIC5205-3.3 U102
U 1 1 5B36DA81
P 4850 1750
F 0 "U102" H 4850 2092 50  0000 C CNN
F 1 "MIC5205-3.3" H 4850 2001 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 4850 2075 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/devicedoc/mic5205.pdf" H 4850 1750 50  0001 C CNN
	1    4850 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0108
U 1 1 5B36DB53
P 5450 1650
F 0 "#PWR0108" H 5450 1500 50  0001 C CNN
F 1 "+3.3V" V 5465 1778 50  0000 L CNN
F 2 "" H 5450 1650 50  0001 C CNN
F 3 "" H 5450 1650 50  0001 C CNN
	1    5450 1650
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 1650 5150 1650
$Comp
L power:GND #PWR0109
U 1 1 5B36DFD9
P 4850 2050
F 0 "#PWR0109" H 4850 1800 50  0001 C CNN
F 1 "GND" H 4855 1877 50  0000 C CNN
F 2 "" H 4850 2050 50  0001 C CNN
F 3 "" H 4850 2050 50  0001 C CNN
	1    4850 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1750 4550 1650
Wire Wire Line
	4550 1650 4150 1650
Connection ~ 4550 1650
Text Label 4150 1650 0    50   ~ 0
VIN
$Comp
L Device:C C104
U 1 1 5B36E86B
P 5300 1950
F 0 "C104" H 5415 1996 50  0000 L CNN
F 1 "100n" H 5415 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5338 1800 50  0001 C CNN
F 3 "~" H 5300 1950 50  0001 C CNN
	1    5300 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 1750 5300 1750
Wire Wire Line
	5300 1750 5300 1800
$Comp
L power:GND #PWR0110
U 1 1 5B36ED9E
P 5300 2100
F 0 "#PWR0110" H 5300 1850 50  0001 C CNN
F 1 "GND" H 5305 1927 50  0000 C CNN
F 2 "" H 5300 2100 50  0001 C CNN
F 3 "" H 5300 2100 50  0001 C CNN
	1    5300 2100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0111
U 1 1 5B36EFD7
P 2700 3300
F 0 "#PWR0111" H 2700 3150 50  0001 C CNN
F 1 "+3.3V" H 2715 3473 50  0000 C CNN
F 2 "" H 2700 3300 50  0001 C CNN
F 3 "" H 2700 3300 50  0001 C CNN
	1    2700 3300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5B36F069
P 8700 4550
F 0 "#PWR0112" H 8700 4300 50  0001 C CNN
F 1 "GND" H 8705 4377 50  0000 C CNN
F 2 "" H 8700 4550 50  0001 C CNN
F 3 "" H 8700 4550 50  0001 C CNN
	1    8700 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 3900 8700 3850
Connection ~ 8700 3850
$Comp
L power:GND #PWR0113
U 1 1 5B36F592
P 2700 5050
F 0 "#PWR0113" H 2700 4800 50  0001 C CNN
F 1 "GND" H 2705 4877 50  0000 C CNN
F 2 "" H 2700 5050 50  0001 C CNN
F 3 "" H 2700 5050 50  0001 C CNN
	1    2700 5050
	1    0    0    -1  
$EndComp
Text Label 4650 3950 2    50   ~ 0
SETPOINT
$Comp
L Device:CP C105
U 1 1 5B3706D2
P 5550 4150
F 0 "C105" H 5668 4196 50  0000 L CNN
F 1 "10u" H 5668 4105 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-31_Kemet-D" H 5588 4000 50  0001 C CNN
F 3 "~" H 5550 4150 50  0001 C CNN
	1    5550 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5B37072E
P 5550 4300
F 0 "#PWR0114" H 5550 4050 50  0001 C CNN
F 1 "GND" H 5555 4127 50  0000 C CNN
F 2 "" H 5550 4300 50  0001 C CNN
F 3 "" H 5550 4300 50  0001 C CNN
	1    5550 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C106
U 1 1 5B370927
P 5950 4150
F 0 "C106" H 6065 4196 50  0000 L CNN
F 1 "100n" H 6065 4105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5988 4000 50  0001 C CNN
F 3 "~" H 5950 4150 50  0001 C CNN
	1    5950 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5B370991
P 5950 4300
F 0 "#PWR0115" H 5950 4050 50  0001 C CNN
F 1 "GND" H 5955 4127 50  0000 C CNN
F 2 "" H 5950 4300 50  0001 C CNN
F 3 "" H 5950 4300 50  0001 C CNN
	1    5950 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 3950 5550 3950
Wire Wire Line
	5550 3950 5550 4000
Connection ~ 5550 3950
Wire Wire Line
	5550 3950 5950 3950
Wire Wire Line
	5950 3950 5950 4000
Connection ~ 5950 3950
Wire Wire Line
	5950 3950 6450 3950
$Comp
L Device:R R104
U 1 1 5B3730FA
P 7850 4400
F 0 "R104" H 7920 4446 50  0000 L CNN
F 1 "1.8k" H 7920 4355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7780 4400 50  0001 C CNN
F 3 "~" H 7850 4400 50  0001 C CNN
	1    7850 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 4200 7850 4200
Wire Wire Line
	7850 4200 7850 4250
$Comp
L Device:C C108
U 1 1 5B373970
P 7850 4750
F 0 "C108" H 7735 4704 50  0000 R CNN
F 1 "8.2n" H 7735 4795 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7888 4600 50  0001 C CNN
F 3 "~" H 7850 4750 50  0001 C CNN
	1    7850 4750
	-1   0    0    1   
$EndComp
Wire Wire Line
	7850 4600 7850 4550
$Comp
L power:GND #PWR0116
U 1 1 5B374246
P 7850 4900
F 0 "#PWR0116" H 7850 4650 50  0001 C CNN
F 1 "GND" H 7855 4727 50  0000 C CNN
F 2 "" H 7850 4900 50  0001 C CNN
F 3 "" H 7850 4900 50  0001 C CNN
	1    7850 4900
	1    0    0    -1  
$EndComp
Text Notes 8850 4950 0    50   ~ 0
Vref = 0.8V
Wire Wire Line
	3200 4050 3650 4050
Text Label 3650 4050 2    50   ~ 0
BTN
$Comp
L Device:R R108
U 1 1 5B37AE90
P 9900 4250
F 0 "R108" H 9970 4296 50  0000 L CNN
F 1 "0.01" H 9970 4205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9830 4250 50  0001 C CNN
F 3 "~" H 9900 4250 50  0001 C CNN
	1    9900 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 4050 9900 4100
Wire Wire Line
	9900 4050 9600 4050
Connection ~ 9900 4050
Text Label 9600 4050 0    50   ~ 0
ISENSE
$Comp
L Device:C C103
U 1 1 5B37F421
P 4200 4150
F 0 "C103" H 4315 4196 50  0000 L CNN
F 1 "1u" H 4315 4105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4238 4000 50  0001 C CNN
F 3 "~" H 4200 4150 50  0001 C CNN
	1    4200 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R102
U 1 1 5B37FEB6
P 4000 3950
F 0 "R102" V 3793 3950 50  0000 C CNN
F 1 "10k" V 3884 3950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3930 3950 50  0001 C CNN
F 3 "~" H 4000 3950 50  0001 C CNN
	1    4000 3950
	0    1    1    0   
$EndComp
Wire Wire Line
	3400 3300 3700 3300
Wire Wire Line
	3200 3750 3650 3750
Text Label 3400 3300 0    50   ~ 0
LED1
Text Label 3650 3750 2    50   ~ 0
LED1
Wire Wire Line
	3200 3950 3850 3950
Wire Wire Line
	4200 4000 4200 3950
Wire Wire Line
	4150 3950 4200 3950
Connection ~ 4200 3950
Wire Wire Line
	4200 3950 4650 3950
$Comp
L power:GND #PWR0117
U 1 1 5B384FD6
P 4200 4300
F 0 "#PWR0117" H 4200 4050 50  0001 C CNN
F 1 "GND" H 4205 4127 50  0000 C CNN
F 2 "" H 4200 4300 50  0001 C CNN
F 3 "" H 4200 4300 50  0001 C CNN
	1    4200 4300
	1    0    0    -1  
$EndComp
Text Notes 4200 3800 0    50   ~ 0
f_cutoff = 10ms
$Comp
L Connector:Test_Point TP101
U 1 1 5B385402
P 1400 3750
F 0 "TP101" V 1595 3824 50  0000 C CNN
F 1 "Test_Point" V 1504 3824 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 1600 3750 50  0001 C CNN
F 3 "~" H 1600 3750 50  0001 C CNN
	1    1400 3750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1400 3750 2200 3750
Wire Wire Line
	3200 4750 3650 4750
Text Label 3650 4750 2    50   ~ 0
SWDIO
Wire Wire Line
	3200 4850 3650 4850
Text Label 3650 4850 2    50   ~ 0
SWCLK
$Comp
L power:GND #PWR0118
U 1 1 5B387FFA
P 2200 3950
F 0 "#PWR0118" H 2200 3700 50  0001 C CNN
F 1 "GND" V 2205 3822 50  0000 R CNN
F 2 "" H 2200 3950 50  0001 C CNN
F 3 "" H 2200 3950 50  0001 C CNN
	1    2200 3950
	0    1    1    0   
$EndComp
Text Notes 1000 4000 0    50   ~ 0
BOOT0=0 => FLASH
Connection ~ 9200 3850
Wire Wire Line
	8700 3850 9200 3850
$Comp
L Device:R R105
U 1 1 5B38AF5E
P 8350 4400
F 0 "R105" H 8420 4446 50  0000 L CNN
F 1 "R" H 8420 4355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8280 4400 50  0001 C CNN
F 3 "~" H 8350 4400 50  0001 C CNN
	1    8350 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 4250 8350 4200
Connection ~ 8350 4200
Wire Wire Line
	8350 4200 8100 4200
Wire Wire Line
	8350 4550 8350 5350
Wire Wire Line
	8350 5350 7700 5350
Text Label 7700 5350 0    50   ~ 0
SETPOINT
NoConn ~ 3200 4250
NoConn ~ 3200 4350
NoConn ~ 3200 4450
NoConn ~ 3200 4550
NoConn ~ 2200 4550
NoConn ~ 2200 4650
NoConn ~ 2200 4850
Text Notes 8250 3550 0    50   ~ 0
Imax > 5A
$Comp
L power:+3.3V #PWR0119
U 1 1 5B398160
P 1750 2750
F 0 "#PWR0119" H 1750 2600 50  0001 C CNN
F 1 "+3.3V" H 1765 2923 50  0000 C CNN
F 2 "" H 1750 2750 50  0001 C CNN
F 3 "" H 1750 2750 50  0001 C CNN
	1    1750 2750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0120
U 1 1 5B3981E3
P 2800 3500
F 0 "#PWR0120" H 2800 3350 50  0001 C CNN
F 1 "+3.3V" H 2815 3673 50  0000 C CNN
F 2 "" H 2800 3500 50  0001 C CNN
F 3 "" H 2800 3500 50  0001 C CNN
	1    2800 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 3500 2800 3550
Wire Wire Line
	2700 3300 2700 3550
$Comp
L Device:C C101
U 1 1 5B39C251
P 1750 2900
F 0 "C101" H 1865 2946 50  0000 L CNN
F 1 "100n" H 1865 2855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1788 2750 50  0001 C CNN
F 3 "~" H 1750 2900 50  0001 C CNN
	1    1750 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 5B39C2C3
P 1750 3050
F 0 "#PWR0121" H 1750 2800 50  0001 C CNN
F 1 "GND" H 1755 2877 50  0000 C CNN
F 2 "" H 1750 3050 50  0001 C CNN
F 3 "" H 1750 3050 50  0001 C CNN
	1    1750 3050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0122
U 1 1 5B39C3BA
P 2200 2750
F 0 "#PWR0122" H 2200 2600 50  0001 C CNN
F 1 "+3.3V" H 2215 2923 50  0000 C CNN
F 2 "" H 2200 2750 50  0001 C CNN
F 3 "" H 2200 2750 50  0001 C CNN
	1    2200 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C102
U 1 1 5B39C3C0
P 2200 2900
F 0 "C102" H 2315 2946 50  0000 L CNN
F 1 "100n" H 2315 2855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2238 2750 50  0001 C CNN
F 3 "~" H 2200 2900 50  0001 C CNN
	1    2200 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0123
U 1 1 5B39C3C6
P 2200 3050
F 0 "#PWR0123" H 2200 2800 50  0001 C CNN
F 1 "GND" H 2205 2877 50  0000 C CNN
F 2 "" H 2200 3050 50  0001 C CNN
F 3 "" H 2200 3050 50  0001 C CNN
	1    2200 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Test_Point TP102
U 1 1 5B39E639
P 2800 6100
F 0 "TP102" V 2754 6288 50  0000 L CNN
F 1 "Test_Point" V 2845 6288 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 3000 6100 50  0001 C CNN
F 3 "~" H 3000 6100 50  0001 C CNN
	1    2800 6100
	0    1    1    0   
$EndComp
Wire Wire Line
	2350 6100 2800 6100
Text Label 2350 6100 0    50   ~ 0
SWDIO
Wire Wire Line
	2350 6300 2800 6300
Text Label 2350 6300 0    50   ~ 0
SWCLK
$Comp
L Connector:Test_Point TP103
U 1 1 5B3A1EF7
P 2800 6300
F 0 "TP103" V 2754 6488 50  0000 L CNN
F 1 "Test_Point" V 2845 6488 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 3000 6300 50  0001 C CNN
F 3 "~" H 3000 6300 50  0001 C CNN
	1    2800 6300
	0    1    1    0   
$EndComp
$Comp
L Amplifier_Operational:LMV321 U104
U 1 1 5B3A6F0E
P 5250 6200
F 0 "U104" H 5300 6500 50  0000 C CNN
F 1 "LMV321" H 5400 6400 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 5250 6200 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/lmv324.pdf" H 5250 6200 50  0001 C CNN
	1    5250 6200
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0124
U 1 1 5B3A81EE
P 5350 5900
F 0 "#PWR0124" H 5350 5750 50  0001 C CNN
F 1 "+3.3V" H 5365 6073 50  0000 C CNN
F 2 "" H 5350 5900 50  0001 C CNN
F 3 "" H 5350 5900 50  0001 C CNN
	1    5350 5900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 5B3A8233
P 5350 6500
F 0 "#PWR0125" H 5350 6250 50  0001 C CNN
F 1 "GND" H 5355 6327 50  0000 C CNN
F 2 "" H 5350 6500 50  0001 C CNN
F 3 "" H 5350 6500 50  0001 C CNN
	1    5350 6500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R109
U 1 1 5B3A8579
P 5300 7050
F 0 "R109" V 5093 7050 50  0000 C CNN
F 1 "1k" V 5184 7050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5230 7050 50  0001 C CNN
F 3 "~" H 5300 7050 50  0001 C CNN
	1    5300 7050
	0    1    1    0   
$EndComp
$Comp
L Device:R R110
U 1 1 5B3A8635
P 5950 7050
F 0 "R110" V 5743 7050 50  0000 C CNN
F 1 "1k" V 5834 7050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5880 7050 50  0001 C CNN
F 3 "~" H 5950 7050 50  0001 C CNN
	1    5950 7050
	0    1    1    0   
$EndComp
Wire Wire Line
	5550 6300 5650 6300
Wire Wire Line
	5650 6300 5650 7050
Wire Wire Line
	5650 7050 5800 7050
Wire Wire Line
	5450 7050 5650 7050
Wire Wire Line
	5150 7050 4700 7050
Wire Wire Line
	4700 7050 4700 6200
Wire Wire Line
	4700 6200 4950 6200
Wire Wire Line
	3200 4150 3850 4150
Connection ~ 4700 6200
Wire Wire Line
	5550 6100 6000 6100
Text Label 6000 6100 2    50   ~ 0
ISENSE
$Comp
L power:GND #PWR0126
U 1 1 5B3AF929
P 6100 7050
F 0 "#PWR0126" H 6100 6800 50  0001 C CNN
F 1 "GND" H 6105 6877 50  0000 C CNN
F 2 "" H 6100 7050 50  0001 C CNN
F 3 "" H 6100 7050 50  0001 C CNN
	1    6100 7050
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x01_Female J101
U 1 1 5B3AFF8D
P 8800 1600
F 0 "J101" H 8827 1626 50  0000 L CNN
F 1 "Conn_01x01_Female" H 8827 1535 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 8800 1600 50  0001 C CNN
F 3 "~" H 8800 1600 50  0001 C CNN
	1    8800 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 1600 8000 1600
Text Label 8000 1600 0    50   ~ 0
BTN
$Comp
L power:+3.3V #PWR0127
U 1 1 5B3B4DA6
P 4800 5250
F 0 "#PWR0127" H 4800 5100 50  0001 C CNN
F 1 "+3.3V" H 4815 5423 50  0000 C CNN
F 2 "" H 4800 5250 50  0001 C CNN
F 3 "" H 4800 5250 50  0001 C CNN
	1    4800 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C111
U 1 1 5B3B4DAC
P 4800 5400
F 0 "C111" H 4915 5446 50  0000 L CNN
F 1 "100n" H 4915 5355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4838 5250 50  0001 C CNN
F 3 "~" H 4800 5400 50  0001 C CNN
	1    4800 5400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 5B3B4DB2
P 4800 5550
F 0 "#PWR0128" H 4800 5300 50  0001 C CNN
F 1 "GND" H 4805 5377 50  0000 C CNN
F 2 "" H 4800 5550 50  0001 C CNN
F 3 "" H 4800 5550 50  0001 C CNN
	1    4800 5550
	1    0    0    -1  
$EndComp
Connection ~ 5650 7050
Wire Wire Line
	3850 6200 4700 6200
Wire Wire Line
	3850 4150 3850 6200
$Comp
L Connector:Conn_01x01_Female J102
U 1 1 5B3BF396
P 1050 1700
F 0 "J102" H 944 1475 50  0000 C CNN
F 1 "Conn_01x01_Female" H 944 1566 50  0000 C CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 1050 1700 50  0001 C CNN
F 3 "~" H 1050 1700 50  0001 C CNN
	1    1050 1700
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x01_Female J103
U 1 1 5B3BF6C2
P 1050 2000
F 0 "J103" H 944 1775 50  0000 C CNN
F 1 "Conn_01x01_Female" H 944 1866 50  0000 C CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 1050 2000 50  0001 C CNN
F 3 "~" H 1050 2000 50  0001 C CNN
	1    1050 2000
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x01_Female J104
U 1 1 5B3C2C64
P 10400 3850
F 0 "J104" H 10427 3876 50  0000 L CNN
F 1 "Conn_01x01_Female" H 10427 3785 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 10400 3850 50  0001 C CNN
F 3 "~" H 10400 3850 50  0001 C CNN
	1    10400 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 3850 10200 3850
$Comp
L Connector:Conn_01x01_Female J105
U 1 1 5B3C4B5B
P 10400 4050
F 0 "J105" H 10427 4076 50  0000 L CNN
F 1 "Conn_01x01_Female" H 10427 3985 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 10400 4050 50  0001 C CNN
F 3 "~" H 10400 4050 50  0001 C CNN
	1    10400 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 4050 10200 4050
$Comp
L Connector:Test_Point TP104
U 1 1 5B3C84C4
P 2800 6500
F 0 "TP104" V 2754 6688 50  0000 L CNN
F 1 "Test_Point" V 2845 6688 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 3000 6500 50  0001 C CNN
F 3 "~" H 3000 6500 50  0001 C CNN
	1    2800 6500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 5B3C9FC4
P 2450 6500
F 0 "#PWR0129" H 2450 6250 50  0001 C CNN
F 1 "GND" V 2455 6372 50  0000 R CNN
F 2 "" H 2450 6500 50  0001 C CNN
F 3 "" H 2450 6500 50  0001 C CNN
	1    2450 6500
	0    1    1    0   
$EndComp
Wire Wire Line
	2450 6500 2800 6500
Text Label 9950 3850 0    50   ~ 0
LED_P
Text Label 9950 4050 0    50   ~ 0
LED_N
Text Label 7850 3850 0    50   ~ 0
SW
Wire Wire Line
	3200 4650 3650 4650
Text Label 3650 4650 2    50   ~ 0
LED2
$Comp
L Device:R R111
U 1 1 5B3CCDCF
P 8450 1950
F 0 "R111" V 8243 1950 50  0000 C CNN
F 1 "1k" V 8334 1950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8380 1950 50  0001 C CNN
F 3 "~" H 8450 1950 50  0001 C CNN
	1    8450 1950
	0    1    1    0   
$EndComp
Wire Wire Line
	8000 1950 8300 1950
Text Label 8000 1950 0    50   ~ 0
LED2
$Comp
L Connector:Conn_01x01_Female J106
U 1 1 5B3CEA94
P 8800 1950
F 0 "J106" H 8827 1976 50  0000 L CNN
F 1 "Conn_01x01_Female" H 8827 1885 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 8800 1950 50  0001 C CNN
F 3 "~" H 8800 1950 50  0001 C CNN
	1    8800 1950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J107
U 1 1 5B3D0753
P 8800 2250
F 0 "J107" H 8827 2276 50  0000 L CNN
F 1 "Conn_01x01_Female" H 8827 2185 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 8800 2250 50  0001 C CNN
F 3 "~" H 8800 2250 50  0001 C CNN
	1    8800 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0130
U 1 1 5B3D23FB
P 8100 2250
F 0 "#PWR0130" H 8100 2000 50  0001 C CNN
F 1 "GND" H 8105 2077 50  0000 C CNN
F 2 "" H 8100 2250 50  0001 C CNN
F 3 "" H 8100 2250 50  0001 C CNN
	1    8100 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 2250 8600 2250
$EndSCHEMATC
