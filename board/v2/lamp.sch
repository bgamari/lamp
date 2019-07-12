EESchema Schematic File Version 4
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "LED driver"
Date "2019-07-02"
Rev "2"
Comp "Ben Gamari"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 6350 3000 0    50   ~ 0
3.5V <= Vin < 15V\n2.8V <= Vout <= 3.5V\nIout = 4A\nf_sw = 2.5 MHz
Wire Wire Line
	1250 1350 1550 1350
$Comp
L power:GND #PWR02
U 1 1 5B36A9A1
P 1400 1650
F 0 "#PWR02" H 1400 1400 50  0001 C CNN
F 1 "GND" H 1405 1477 50  0000 C CNN
F 2 "" H 1400 1650 50  0001 C CNN
F 3 "" H 1400 1650 50  0001 C CNN
	1    1400 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 1650 1250 1650
$Comp
L power:GND #PWR029
U 1 1 5B36A9CE
P 6900 4400
F 0 "#PWR029" H 6900 4150 50  0001 C CNN
F 1 "GND" H 6905 4227 50  0000 C CNN
F 2 "" H 6900 4400 50  0001 C CNN
F 3 "" H 6900 4400 50  0001 C CNN
	1    6900 4400
	1    0    0    -1  
$EndComp
$Comp
L MCU_ST_STM32F0:STM32F030F4Px U1
U 1 1 5B36AAB9
P 2350 3900
F 0 "U1" H 2700 4550 50  0000 C CNN
F 1 "STM32F030F4P6" H 2300 3850 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 1950 3200 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00088500.pdf" H 2350 3900 50  0001 C CNN
F 4 "ST Micro" H 0   0   50  0001 C CNN "MFR"
F 5 "STM32F030F4P6" H 0   0   50  0001 C CNN "MPN"
	1    2350 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5B36AC10
P 3900 2950
F 0 "D1" H 3892 2695 50  0000 C CNN
F 1 "LED" H 3892 2786 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 3900 2950 50  0001 C CNN
F 3 "~" H 3900 2950 50  0001 C CNN
	1    3900 2950
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 5B36ACCF
P 3500 2950
F 0 "R3" V 3293 2950 50  0000 C CNN
F 1 "1k" V 3384 2950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3430 2950 50  0001 C CNN
F 3 "~" H 3500 2950 50  0001 C CNN
	1    3500 2950
	0    1    1    0   
$EndComp
$Comp
L Device:L L1
U 1 1 5B36ADF2
P 8100 3500
F 0 "L1" V 8290 3500 50  0000 C CNN
F 1 "2.2u" V 8199 3500 50  0000 C CNN
F 2 "Inductor_SMD:L_7.3x7.3_H4.5" H 8100 3500 50  0001 C CNN
F 3 "~" H 8100 3500 50  0001 C CNN
F 4 "Eaton" H 8100 3500 50  0001 C CNN "MFR"
F 5 "DR74-2R2-R " H 0   0   50  0001 C CNN "MPN"
	1    8100 3500
	0    -1   -1   0   
$EndComp
Text Label 1550 1350 2    50   ~ 0
VIN
Text Label 4650 3500 0    50   ~ 0
VIN
$Comp
L Device:C C10
U 1 1 5B36B26C
P 5700 4200
F 0 "C10" H 5815 4246 50  0000 L CNN
F 1 "3.3n" H 5815 4155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5738 4050 50  0001 C CNN
F 3 "~" H 5700 4200 50  0001 C CNN
	1    5700 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 4050 5700 4000
$Comp
L power:GND #PWR026
U 1 1 5B36B2F3
P 5700 4350
F 0 "#PWR026" H 5700 4100 50  0001 C CNN
F 1 "GND" H 5705 4177 50  0000 C CNN
F 2 "" H 5700 4350 50  0001 C CNN
F 3 "" H 5700 4350 50  0001 C CNN
	1    5700 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C14
U 1 1 5B36B3CD
P 9000 3700
F 0 "C14" H 9118 3746 50  0000 L CNN
F 1 "22u" H 9118 3655 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-15_Kemet-W" H 9038 3550 50  0001 C CNN
F 3 "~" H 9000 3700 50  0001 C CNN
F 4 "Panasonic" H 9000 3700 50  0001 C CNN "MFR"
F 5 "25TQC22MV" H 9000 3700 50  0001 C CNN "MPN"
	1    9000 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 3500 9000 3550
$Comp
L power:GND #PWR035
U 1 1 5B36B549
P 9000 3850
F 0 "#PWR035" H 9000 3600 50  0001 C CNN
F 1 "GND" H 9005 3677 50  0000 C CNN
F 2 "" H 9000 3850 50  0001 C CNN
F 3 "" H 9000 3850 50  0001 C CNN
	1    9000 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 3500 8350 3500
$Comp
L power:GND #PWR036
U 1 1 5B36BAD2
P 9700 4200
F 0 "#PWR036" H 9700 3950 50  0001 C CNN
F 1 "GND" H 9705 4027 50  0000 C CNN
F 2 "" H 9700 4200 50  0001 C CNN
F 3 "" H 9700 4200 50  0001 C CNN
	1    9700 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 5B36C046
P 8500 3700
F 0 "R11" H 8570 3746 50  0000 L CNN
F 1 "22k" H 8570 3655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8430 3700 50  0001 C CNN
F 3 "~" H 8500 3700 50  0001 C CNN
	1    8500 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 5B36C394
P 8500 4100
F 0 "R12" H 8570 4146 50  0000 L CNN
F 1 "4.7k" H 8570 4055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8430 4100 50  0001 C CNN
F 3 "~" H 8500 4100 50  0001 C CNN
	1    8500 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 3900 7900 3850
Text Label 5950 3700 0    50   ~ 0
EN
Wire Wire Line
	3650 2950 3750 2950
$Comp
L power:GND #PWR013
U 1 1 5B36D22B
P 4150 2950
F 0 "#PWR013" H 4150 2700 50  0001 C CNN
F 1 "GND" V 4155 2822 50  0000 R CNN
F 2 "" H 4150 2950 50  0001 C CNN
F 3 "" H 4150 2950 50  0001 C CNN
	1    4150 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4150 2950 4050 2950
Wire Wire Line
	2850 3500 3300 3500
Text Label 3300 3500 2    50   ~ 0
EN
$Comp
L Regulator_Linear:MIC5205-3.3 U3
U 1 1 5B36DA81
P 4850 1400
F 0 "U3" H 4850 1742 50  0000 C CNN
F 1 "MIC5205-3.3" H 4850 1651 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 4850 1725 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/devicedoc/mic5205.pdf" H 4850 1400 50  0001 C CNN
F 4 "Microchip" H 0   0   50  0001 C CNN "MFR"
F 5 "MIC5205-3.3YM5-TR" H 0   0   50  0001 C CNN "MPN"
	1    4850 1400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR022
U 1 1 5B36DB53
P 6000 1300
F 0 "#PWR022" H 6000 1150 50  0001 C CNN
F 1 "+3.3V" V 6015 1428 50  0000 L CNN
F 2 "" H 6000 1300 50  0001 C CNN
F 3 "" H 6000 1300 50  0001 C CNN
	1    6000 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	6000 1300 5900 1300
$Comp
L power:GND #PWR019
U 1 1 5B36DFD9
P 4850 1700
F 0 "#PWR019" H 4850 1450 50  0001 C CNN
F 1 "GND" H 4855 1527 50  0000 C CNN
F 2 "" H 4850 1700 50  0001 C CNN
F 3 "" H 4850 1700 50  0001 C CNN
	1    4850 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1400 4550 1300
Wire Wire Line
	4550 1300 4200 1300
Connection ~ 4550 1300
Text Label 3900 1300 0    50   ~ 0
VIN
$Comp
L Device:C C7
U 1 1 5B36E86B
P 5300 1600
F 0 "C7" H 5415 1646 50  0000 L CNN
F 1 "100n" H 5415 1555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5338 1450 50  0001 C CNN
F 3 "~" H 5300 1600 50  0001 C CNN
	1    5300 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 1400 5300 1400
Wire Wire Line
	5300 1400 5300 1450
$Comp
L power:GND #PWR020
U 1 1 5B36ED9E
P 5300 1750
F 0 "#PWR020" H 5300 1500 50  0001 C CNN
F 1 "GND" H 5305 1577 50  0000 C CNN
F 2 "" H 5300 1750 50  0001 C CNN
F 3 "" H 5300 1750 50  0001 C CNN
	1    5300 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 5B36EFD7
P 2350 2950
F 0 "#PWR09" H 2350 2800 50  0001 C CNN
F 1 "+3.3V" H 2365 3123 50  0000 C CNN
F 2 "" H 2350 2950 50  0001 C CNN
F 3 "" H 2350 2950 50  0001 C CNN
	1    2350 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR034
U 1 1 5B36F069
P 8500 4250
F 0 "#PWR034" H 8500 4000 50  0001 C CNN
F 1 "GND" H 8505 4077 50  0000 C CNN
F 2 "" H 8500 4250 50  0001 C CNN
F 3 "" H 8500 4250 50  0001 C CNN
	1    8500 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 3550 8500 3500
Connection ~ 8500 3500
$Comp
L power:GND #PWR010
U 1 1 5B36F592
P 2350 4700
F 0 "#PWR010" H 2350 4450 50  0001 C CNN
F 1 "GND" H 2355 4527 50  0000 C CNN
F 2 "" H 2350 4700 50  0001 C CNN
F 3 "" H 2350 4700 50  0001 C CNN
	1    2350 4700
	1    0    0    -1  
$EndComp
Text Label 4350 3800 2    50   ~ 0
SETPOINT
$Comp
L Device:CP C8
U 1 1 5B3706D2
P 5000 3700
F 0 "C8" H 5118 3746 50  0000 L CNN
F 1 "10u" H 5118 3655 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-3528-12_Kemet-T" H 5038 3550 50  0001 C CNN
F 3 "~" H 5000 3700 50  0001 C CNN
F 4 "Kemet" H 5000 3700 50  0001 C CNN "MFR"
F 5 "T520B106M016ATE100" H 5000 3700 50  0001 C CNN "MPN"
	1    5000 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5B37072E
P 5000 3850
F 0 "#PWR021" H 5000 3600 50  0001 C CNN
F 1 "GND" H 5005 3677 50  0000 C CNN
F 2 "" H 5000 3850 50  0001 C CNN
F 3 "" H 5000 3850 50  0001 C CNN
	1    5000 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5B370927
P 5450 3700
F 0 "C9" H 5565 3746 50  0000 L CNN
F 1 "100n" H 5565 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5488 3550 50  0001 C CNN
F 3 "~" H 5450 3700 50  0001 C CNN
	1    5450 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 5B370991
P 5450 3850
F 0 "#PWR025" H 5450 3600 50  0001 C CNN
F 1 "GND" H 5455 3677 50  0000 C CNN
F 2 "" H 5450 3850 50  0001 C CNN
F 3 "" H 5450 3850 50  0001 C CNN
	1    5450 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 3500 5000 3500
Wire Wire Line
	5000 3500 5000 3550
Connection ~ 5000 3500
Wire Wire Line
	5000 3500 5450 3500
Wire Wire Line
	5450 3500 5450 3550
Connection ~ 5450 3500
$Comp
L Device:R R8
U 1 1 5B3730FA
P 9000 3200
F 0 "R8" H 9070 3246 50  0000 L CNN
F 1 "100k" H 9070 3155 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8930 3200 50  0001 C CNN
F 3 "~" H 9000 3200 50  0001 C CNN
	1    9000 3200
	1    0    0    -1  
$EndComp
Text Notes 7950 3850 0    50   ~ 0
Vref = 0.8V
Wire Wire Line
	2850 4200 3300 4200
Text Label 3300 4200 2    50   ~ 0
BTN
$Comp
L Device:R R13
U 1 1 5B37AE90
P 9700 4050
F 0 "R13" H 9770 4096 50  0000 L CNN
F 1 "0.01" H 9770 4005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9630 4050 50  0001 C CNN
F 3 "~" H 9700 4050 50  0001 C CNN
F 4 " RL1220T-R010-J " H 9700 4050 50  0001 C CNN "MPN"
F 5 "" H 9700 4050 50  0001 C CNN "MFG"
F 6 "Susumu" H 0   0   50  0001 C CNN "MFR"
	1    9700 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 3850 9700 3900
Wire Wire Line
	9700 3850 9400 3850
Connection ~ 9700 3850
Text Label 9400 3850 0    50   ~ 0
ISENSE
$Comp
L Device:C C3
U 1 1 5B37F421
P 3900 4000
F 0 "C3" H 4015 4046 50  0000 L CNN
F 1 "1u" H 4015 3955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3938 3850 50  0001 C CNN
F 3 "~" H 3900 4000 50  0001 C CNN
	1    3900 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5B37FEB6
P 3700 3800
F 0 "R4" V 3493 3800 50  0000 C CNN
F 1 "10k" V 3584 3800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3630 3800 50  0001 C CNN
F 3 "~" H 3700 3800 50  0001 C CNN
	1    3700 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	3050 2950 3350 2950
Wire Wire Line
	2850 4000 3300 4000
Text Label 3050 2950 0    50   ~ 0
LED1
Text Label 3300 4000 2    50   ~ 0
LED1
Wire Wire Line
	3900 3850 3900 3800
Wire Wire Line
	3850 3800 3900 3800
Connection ~ 3900 3800
Wire Wire Line
	3900 3800 4350 3800
$Comp
L power:GND #PWR012
U 1 1 5B384FD6
P 3900 4150
F 0 "#PWR012" H 3900 3900 50  0001 C CNN
F 1 "GND" H 3905 3977 50  0000 C CNN
F 2 "" H 3900 4150 50  0001 C CNN
F 3 "" H 3900 4150 50  0001 C CNN
	1    3900 4150
	1    0    0    -1  
$EndComp
Text Notes 3900 3650 0    50   ~ 0
f_cutoff = 10 kHz
$Comp
L Connector:Test_Point TP1
U 1 1 5B385402
P 1050 3400
F 0 "TP1" V 1245 3474 50  0000 C CNN
F 1 "Test_Point" V 1154 3474 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 1250 3400 50  0001 C CNN
F 3 "~" H 1250 3400 50  0001 C CNN
	1    1050 3400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1050 3400 1850 3400
Wire Wire Line
	2850 4400 3300 4400
Text Label 3300 4400 2    50   ~ 0
SWDIO
Wire Wire Line
	2850 4500 3300 4500
Text Label 3300 4500 2    50   ~ 0
SWCLK
$Comp
L power:GND #PWR07
U 1 1 5B387FFA
P 1850 3600
F 0 "#PWR07" H 1850 3350 50  0001 C CNN
F 1 "GND" V 1855 3472 50  0000 R CNN
F 2 "" H 1850 3600 50  0001 C CNN
F 3 "" H 1850 3600 50  0001 C CNN
	1    1850 3600
	0    1    1    0   
$EndComp
Text Notes 650  3650 0    50   ~ 0
BOOT0=0 => FLASH
Connection ~ 9000 3500
Wire Wire Line
	8500 3500 9000 3500
$Comp
L Device:R R9
U 1 1 5B38AF5E
P 8150 4100
F 0 "R9" H 8220 4146 50  0000 L CNN
F 1 "50k" H 8220 4055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8080 4100 50  0001 C CNN
F 3 "~" H 8150 4100 50  0001 C CNN
	1    8150 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 3950 8150 3900
Wire Wire Line
	8150 3900 7900 3900
Wire Wire Line
	6700 5450 6050 5450
Text Label 6050 5450 0    50   ~ 0
SETPOINT
NoConn ~ 1850 4200
NoConn ~ 1850 4300
Text Notes 8050 3200 0    50   ~ 0
Imax < 3A
$Comp
L power:+3.3V #PWR03
U 1 1 5B398160
P 1400 2400
F 0 "#PWR03" H 1400 2250 50  0001 C CNN
F 1 "+3.3V" H 1415 2573 50  0000 C CNN
F 2 "" H 1400 2400 50  0001 C CNN
F 3 "" H 1400 2400 50  0001 C CNN
	1    1400 2400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR011
U 1 1 5B3981E3
P 2450 3150
F 0 "#PWR011" H 2450 3000 50  0001 C CNN
F 1 "+3.3V" H 2465 3323 50  0000 C CNN
F 2 "" H 2450 3150 50  0001 C CNN
F 3 "" H 2450 3150 50  0001 C CNN
	1    2450 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 3150 2450 3200
Wire Wire Line
	2350 2950 2350 3200
$Comp
L Device:C C1
U 1 1 5B39C251
P 1400 2550
F 0 "C1" H 1515 2596 50  0000 L CNN
F 1 "100n" H 1515 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1438 2400 50  0001 C CNN
F 3 "~" H 1400 2550 50  0001 C CNN
	1    1400 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5B39C2C3
P 1400 2700
F 0 "#PWR04" H 1400 2450 50  0001 C CNN
F 1 "GND" H 1405 2527 50  0000 C CNN
F 2 "" H 1400 2700 50  0001 C CNN
F 3 "" H 1400 2700 50  0001 C CNN
	1    1400 2700
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR05
U 1 1 5B39C3BA
P 1850 2400
F 0 "#PWR05" H 1850 2250 50  0001 C CNN
F 1 "+3.3V" H 1865 2573 50  0000 C CNN
F 2 "" H 1850 2400 50  0001 C CNN
F 3 "" H 1850 2400 50  0001 C CNN
	1    1850 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5B39C3C0
P 1850 2550
F 0 "C2" H 1965 2596 50  0000 L CNN
F 1 "100n" H 1965 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1888 2400 50  0001 C CNN
F 3 "~" H 1850 2550 50  0001 C CNN
	1    1850 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5B39C3C6
P 1850 2700
F 0 "#PWR06" H 1850 2450 50  0001 C CNN
F 1 "GND" H 1855 2527 50  0000 C CNN
F 2 "" H 1850 2700 50  0001 C CNN
F 3 "" H 1850 2700 50  0001 C CNN
	1    1850 2700
	1    0    0    -1  
$EndComp
$Comp
L Connector:Test_Point TP2
U 1 1 5B39E639
P 2150 6650
F 0 "TP2" V 2104 6838 50  0000 L CNN
F 1 "Test_Point" V 2195 6838 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 2350 6650 50  0001 C CNN
F 3 "~" H 2350 6650 50  0001 C CNN
	1    2150 6650
	0    1    1    0   
$EndComp
Wire Wire Line
	1700 6650 2150 6650
Text Label 1700 6650 0    50   ~ 0
SWDIO
Wire Wire Line
	1700 6850 2150 6850
Text Label 1700 6850 0    50   ~ 0
SWCLK
$Comp
L Connector:Test_Point TP3
U 1 1 5B3A1EF7
P 2150 6850
F 0 "TP3" V 2104 7038 50  0000 L CNN
F 1 "Test_Point" V 2195 7038 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 2350 6850 50  0001 C CNN
F 3 "~" H 2350 6850 50  0001 C CNN
	1    2150 6850
	0    1    1    0   
$EndComp
$Comp
L Amplifier_Operational:LMV321 U2
U 1 1 5B3A6F0E
P 4650 5850
F 0 "U2" H 4700 6150 50  0000 C CNN
F 1 "LMV321" H 4800 6050 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 4650 5850 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/lmv324.pdf" H 4650 5850 50  0001 C CNN
F 4 "TI" H 0   0   50  0001 C CNN "MFR"
F 5 "LMV321M5" H 0   0   50  0001 C CNN "MPN"
	1    4650 5850
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR017
U 1 1 5B3A81EE
P 4750 5550
F 0 "#PWR017" H 4750 5400 50  0001 C CNN
F 1 "+3.3V" H 4765 5723 50  0000 C CNN
F 2 "" H 4750 5550 50  0001 C CNN
F 3 "" H 4750 5550 50  0001 C CNN
	1    4750 5550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5B3A8233
P 4750 6150
F 0 "#PWR018" H 4750 5900 50  0001 C CNN
F 1 "GND" H 4755 5977 50  0000 C CNN
F 2 "" H 4750 6150 50  0001 C CNN
F 3 "" H 4750 6150 50  0001 C CNN
	1    4750 6150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5B3A8579
P 4600 6700
F 0 "R5" V 4393 6700 50  0000 C CNN
F 1 "100k" V 4484 6700 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4530 6700 50  0001 C CNN
F 3 "~" H 4600 6700 50  0001 C CNN
	1    4600 6700
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5B3A8635
P 5350 6850
F 0 "R6" V 5143 6850 50  0000 C CNN
F 1 "1k" V 5234 6850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5280 6850 50  0001 C CNN
F 3 "~" H 5350 6850 50  0001 C CNN
	1    5350 6850
	0    1    1    0   
$EndComp
Wire Wire Line
	4950 5950 5050 5950
Wire Wire Line
	5050 5950 5050 6700
Wire Wire Line
	5050 6850 5200 6850
Wire Wire Line
	4750 6700 5050 6700
Wire Wire Line
	4450 6700 4100 6700
Wire Wire Line
	4100 6700 4100 5850
Wire Wire Line
	4100 5850 4350 5850
Wire Wire Line
	4950 5750 5400 5750
Text Label 5400 5750 2    50   ~ 0
ISENSE
$Comp
L power:GND #PWR023
U 1 1 5B3AF929
P 5500 6850
F 0 "#PWR023" H 5500 6600 50  0001 C CNN
F 1 "GND" H 5505 6677 50  0000 C CNN
F 2 "" H 5500 6850 50  0001 C CNN
F 3 "" H 5500 6850 50  0001 C CNN
	1    5500 6850
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x01_Female J3
U 1 1 5B3AFF8D
P 8800 1300
F 0 "J3" H 8827 1326 50  0000 L CNN
F 1 "Conn_01x01_Female" H 8827 1235 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 8800 1300 50  0001 C CNN
F 3 "~" H 8800 1300 50  0001 C CNN
	1    8800 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 1300 8000 1300
Text Label 8000 1300 0    50   ~ 0
BTN
$Comp
L Device:C C5
U 1 1 5B3B4DAC
P 5900 1600
F 0 "C5" H 6015 1646 50  0000 L CNN
F 1 "2.2u" H 6015 1555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5938 1450 50  0001 C CNN
F 3 "~" H 5900 1600 50  0001 C CNN
	1    5900 1600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5B3B4DB2
P 5900 1750
F 0 "#PWR016" H 5900 1500 50  0001 C CNN
F 1 "GND" H 5905 1577 50  0000 C CNN
F 2 "" H 5900 1750 50  0001 C CNN
F 3 "" H 5900 1750 50  0001 C CNN
	1    5900 1750
	1    0    0    -1  
$EndComp
Connection ~ 5050 6700
Wire Wire Line
	3500 5850 4100 5850
$Comp
L Connector:Conn_01x01_Female J1
U 1 1 5B3BF396
P 1050 1350
F 0 "J1" H 944 1125 50  0000 C CNN
F 1 "Conn_01x01_Female" H 944 1216 50  0000 C CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 1050 1350 50  0001 C CNN
F 3 "~" H 1050 1350 50  0001 C CNN
	1    1050 1350
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x01_Female J2
U 1 1 5B3BF6C2
P 1050 1650
F 0 "J2" H 944 1425 50  0000 C CNN
F 1 "Conn_01x01_Female" H 944 1516 50  0000 C CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 1050 1650 50  0001 C CNN
F 3 "~" H 1050 1650 50  0001 C CNN
	1    1050 1650
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x01_Female J6
U 1 1 5B3C2C64
P 10200 3500
F 0 "J6" H 10227 3526 50  0000 L CNN
F 1 "Conn_01x01_Female" H 10227 3435 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 10200 3500 50  0001 C CNN
F 3 "~" H 10200 3500 50  0001 C CNN
	1    10200 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 3500 10000 3500
$Comp
L Connector:Conn_01x01_Female J7
U 1 1 5B3C4B5B
P 10200 3850
F 0 "J7" H 10227 3876 50  0000 L CNN
F 1 "Conn_01x01_Female" H 10227 3785 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 10200 3850 50  0001 C CNN
F 3 "~" H 10200 3850 50  0001 C CNN
	1    10200 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 3850 10000 3850
$Comp
L Connector:Test_Point TP4
U 1 1 5B3C84C4
P 2150 7050
F 0 "TP4" V 2104 7238 50  0000 L CNN
F 1 "Test_Point" V 2195 7238 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 2350 7050 50  0001 C CNN
F 3 "~" H 2350 7050 50  0001 C CNN
	1    2150 7050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5B3C9FC4
P 1800 7050
F 0 "#PWR08" H 1800 6800 50  0001 C CNN
F 1 "GND" V 1805 6922 50  0000 R CNN
F 2 "" H 1800 7050 50  0001 C CNN
F 3 "" H 1800 7050 50  0001 C CNN
	1    1800 7050
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 7050 2150 7050
Text Label 9750 3500 0    50   ~ 0
LED_P
Text Label 9750 3850 0    50   ~ 0
LED_N
Text Label 7650 3500 0    50   ~ 0
SW
Wire Wire Line
	2850 4100 3300 4100
Text Label 3300 4100 2    50   ~ 0
LED2
$Comp
L Device:R R10
U 1 1 5B3CCDCF
P 8450 1600
F 0 "R10" V 8243 1600 50  0000 C CNN
F 1 "1k" V 8334 1600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8380 1600 50  0001 C CNN
F 3 "~" H 8450 1600 50  0001 C CNN
	1    8450 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	8000 1600 8300 1600
Text Label 8000 1600 0    50   ~ 0
LED2
$Comp
L Connector:Conn_01x01_Female J4
U 1 1 5B3CEA94
P 8800 1600
F 0 "J4" H 8827 1626 50  0000 L CNN
F 1 "Conn_01x01_Female" H 8827 1535 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 8800 1600 50  0001 C CNN
F 3 "~" H 8800 1600 50  0001 C CNN
	1    8800 1600
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J5
U 1 1 5B3D0753
P 8800 1900
F 0 "J5" H 8827 1926 50  0000 L CNN
F 1 "Conn_01x01_Female" H 8827 1835 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 8800 1900 50  0001 C CNN
F 3 "~" H 8800 1900 50  0001 C CNN
	1    8800 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR033
U 1 1 5B3D23FB
P 8100 1900
F 0 "#PWR033" H 8100 1650 50  0001 C CNN
F 1 "GND" H 8105 1727 50  0000 C CNN
F 2 "" H 8100 1900 50  0001 C CNN
F 3 "" H 8100 1900 50  0001 C CNN
	1    8100 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 1900 8600 1900
Wire Wire Line
	8500 3850 8500 3900
Wire Wire Line
	8150 3900 8500 3900
Connection ~ 8150 3900
Connection ~ 8500 3900
Wire Wire Line
	8500 3900 8500 3950
$Comp
L Amplifier_Operational:LMV321 U5
U 1 1 5B3DFFA7
P 7000 5550
F 0 "U5" H 7050 5850 50  0000 C CNN
F 1 "LMV321" H 7150 5750 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 7000 5550 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/lmv324.pdf" H 7000 5550 50  0001 C CNN
F 4 "TI" H 0   0   50  0001 C CNN "MFR"
F 5 "LMV321M5" H 0   0   50  0001 C CNN "MPN"
	1    7000 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 6150 7450 5550
Wire Wire Line
	7450 5550 7300 5550
Wire Wire Line
	8150 5550 7450 5550
Connection ~ 7450 5550
Wire Wire Line
	8150 4250 8150 5550
$Comp
L power:+3.3V #PWR030
U 1 1 5B3EB79E
P 6900 5250
F 0 "#PWR030" H 6900 5100 50  0001 C CNN
F 1 "+3.3V" H 6915 5423 50  0000 C CNN
F 2 "" H 6900 5250 50  0001 C CNN
F 3 "" H 6900 5250 50  0001 C CNN
	1    6900 5250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 5B3EB7F9
P 6900 5850
F 0 "#PWR031" H 6900 5600 50  0001 C CNN
F 1 "GND" H 6905 5677 50  0000 C CNN
F 2 "" H 6900 5850 50  0001 C CNN
F 3 "" H 6900 5850 50  0001 C CNN
	1    6900 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 5650 6600 5650
Wire Wire Line
	6600 5650 6600 6150
Connection ~ 4100 5850
Wire Wire Line
	6600 6150 7450 6150
$Comp
L Device:C C6
U 1 1 5B40E092
P 4600 7000
F 0 "C6" H 4715 7046 50  0000 L CNN
F 1 "4.7n" H 4715 6955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4638 6850 50  0001 C CNN
F 3 "~" H 4600 7000 50  0001 C CNN
	1    4600 7000
	0    1    1    0   
$EndComp
Wire Wire Line
	4750 7000 5050 7000
Wire Wire Line
	5050 7000 5050 6850
Wire Wire Line
	4450 7000 4100 7000
Wire Wire Line
	4100 7000 4100 6700
Connection ~ 4100 6700
Connection ~ 5050 6850
Wire Wire Line
	5050 6850 5050 6700
Text Notes 5200 6150 0    50   ~ 0
G = 101\nf_co = 300 Hz
$Comp
L power:+3.3V #PWR027
U 1 1 5B41C393
P 6250 5900
F 0 "#PWR027" H 6250 5750 50  0001 C CNN
F 1 "+3.3V" H 6265 6073 50  0000 C CNN
F 2 "" H 6250 5900 50  0001 C CNN
F 3 "" H 6250 5900 50  0001 C CNN
	1    6250 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 5B41C399
P 6250 6050
F 0 "C11" H 6365 6096 50  0000 L CNN
F 1 "100n" H 6365 6005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6288 5900 50  0001 C CNN
F 3 "~" H 6250 6050 50  0001 C CNN
	1    6250 6050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR028
U 1 1 5B41C39F
P 6250 6200
F 0 "#PWR028" H 6250 5950 50  0001 C CNN
F 1 "GND" H 6255 6027 50  0000 C CNN
F 2 "" H 6250 6200 50  0001 C CNN
F 3 "" H 6250 6200 50  0001 C CNN
	1    6250 6200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5B41EEBB
P 4200 1500
F 0 "C4" H 4315 1546 50  0000 L CNN
F 1 "100n" H 4315 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4238 1350 50  0001 C CNN
F 3 "~" H 4200 1500 50  0001 C CNN
	1    4200 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 1350 4200 1300
Connection ~ 4200 1300
Wire Wire Line
	4200 1300 3900 1300
$Comp
L power:GND #PWR014
U 1 1 5B423A75
P 4200 1650
F 0 "#PWR014" H 4200 1400 50  0001 C CNN
F 1 "GND" H 4205 1477 50  0000 C CNN
F 2 "" H 4200 1650 50  0001 C CNN
F 3 "" H 4200 1650 50  0001 C CNN
	1    4200 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5B425754
P 1100 4300
F 0 "R1" V 893 4300 50  0000 C CNN
F 1 "47k" V 984 4300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1030 4300 50  0001 C CNN
F 3 "~" H 1100 4300 50  0001 C CNN
	1    1100 4300
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 5B4257E6
P 1100 4700
F 0 "R2" V 893 4700 50  0000 C CNN
F 1 "10k" V 984 4700 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1030 4700 50  0001 C CNN
F 3 "~" H 1100 4700 50  0001 C CNN
	1    1100 4700
	-1   0    0    1   
$EndComp
Wire Wire Line
	1100 4450 1100 4500
Connection ~ 1100 4500
Wire Wire Line
	1100 4500 1100 4550
$Comp
L power:GND #PWR01
U 1 1 5B42AD22
P 1100 4850
F 0 "#PWR01" H 1100 4600 50  0001 C CNN
F 1 "GND" H 1105 4677 50  0000 C CNN
F 2 "" H 1100 4850 50  0001 C CNN
F 3 "" H 1100 4850 50  0001 C CNN
	1    1100 4850
	1    0    0    -1  
$EndComp
Text Label 800  4050 0    50   ~ 0
VIN
Wire Wire Line
	1100 4050 800  4050
Wire Wire Line
	1100 4050 1100 4150
Wire Wire Line
	1100 4500 1850 4500
Text Label 1550 4500 0    50   ~ 0
VBAT
$Comp
L Device:C C15
U 1 1 5B442D62
P 700 4700
F 0 "C15" H 815 4746 50  0000 L CNN
F 1 "10n" H 815 4655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 738 4550 50  0001 C CNN
F 3 "~" H 700 4700 50  0001 C CNN
	1    700  4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5B442E96
P 700 4850
F 0 "#PWR0101" H 700 4600 50  0001 C CNN
F 1 "GND" H 705 4677 50  0000 C CNN
F 2 "" H 700 4850 50  0001 C CNN
F 3 "" H 700 4850 50  0001 C CNN
	1    700  4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 4500 700  4500
Wire Wire Line
	700  4500 700  4550
$Comp
L Device:C C16
U 1 1 5B4461FC
P 3500 6050
F 0 "C16" H 3615 6096 50  0000 L CNN
F 1 "10n" H 3615 6005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3538 5900 50  0001 C CNN
F 3 "~" H 3500 6050 50  0001 C CNN
	1    3500 6050
	1    0    0    -1  
$EndComp
Connection ~ 3500 5850
Wire Wire Line
	3500 5850 3500 5900
$Comp
L power:GND #PWR0102
U 1 1 5B44905C
P 3500 6200
F 0 "#PWR0102" H 3500 5950 50  0001 C CNN
F 1 "GND" H 3505 6027 50  0000 C CNN
F 2 "" H 3500 6200 50  0001 C CNN
F 3 "" H 3500 6200 50  0001 C CNN
	1    3500 6200
	1    0    0    -1  
$EndComp
Text Notes 1250 4900 0    50   ~ 0
G = 0.18
Text Notes 2000 6150 0    50   ~ 0
Pinning:\n  PA1: PA1\n  PA4: TIM14_CH1\n  PA5: ADC_IN5\n  PA6: TIM16_CH1\n  PA7: TIM17_CH1\n  PA9: PA9\n  PA13: SWDIO\n  PA14: SWCLK\n  PB1: ADC_IN9
NoConn ~ 2850 4300
NoConn ~ 2850 3400
NoConn ~ 2850 3700
Wire Wire Line
	2850 3900 3500 3900
Wire Wire Line
	3500 3900 3500 5850
Wire Wire Line
	2850 3800 3550 3800
NoConn ~ 2850 3600
Wire Wire Line
	5900 1450 5900 1300
Connection ~ 5900 1300
Wire Wire Line
	5900 1300 5150 1300
Wire Wire Line
	6200 3700 5950 3700
Wire Wire Line
	7550 3500 7950 3500
$Comp
L lamp:TLV62130x U4
U 1 1 5D1C83F4
P 6900 3700
F 0 "U4" H 6875 4215 50  0000 C CNN
F 1 "TLV62130x" H 6875 4124 50  0000 C CNN
F 2 "lamp:QFN-16_EP_3x3_Pitch0.5mm" H 6900 3700 50  0001 C CNN
F 3 "" H 6900 3700 50  0001 C CNN
F 4 "TI" H 0   0   50  0001 C CNN "MFR"
F 5 "TLV62130" H 0   0   50  0001 C CNN "MPN"
	1    6900 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 4000 6200 4000
Text Label 5950 4100 0    50   ~ 0
PWRGD
Wire Wire Line
	5950 4100 6200 4100
Wire Wire Line
	6200 3600 6200 3500
$Comp
L power:GND #PWR0103
U 1 1 5D1F19E9
P 6200 3800
F 0 "#PWR0103" H 6200 3550 50  0001 C CNN
F 1 "GND" V 6205 3672 50  0000 R CNN
F 2 "" H 6200 3800 50  0001 C CNN
F 3 "" H 6200 3800 50  0001 C CNN
	1    6200 3800
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5D1F1A50
P 6200 3900
F 0 "#PWR0104" H 6200 3650 50  0001 C CNN
F 1 "GND" V 6205 3772 50  0000 R CNN
F 2 "" H 6200 3900 50  0001 C CNN
F 3 "" H 6200 3900 50  0001 C CNN
	1    6200 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 3500 6200 3500
Connection ~ 6200 3500
Wire Wire Line
	7550 3850 7900 3850
Wire Wire Line
	7550 3700 8350 3700
Wire Wire Line
	8350 3700 8350 3500
Connection ~ 8350 3500
Wire Wire Line
	8350 3500 8500 3500
Wire Wire Line
	9000 3350 9000 3500
Wire Wire Line
	9000 3050 9000 2950
Wire Wire Line
	9000 2950 9350 2950
Text Label 9350 2950 2    50   ~ 0
PWRGD
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5D221A8F
P 900 5700
F 0 "#FLG0101" H 900 5775 50  0001 C CNN
F 1 "PWR_FLAG" V 900 5828 50  0000 L CNN
F 2 "" H 900 5700 50  0001 C CNN
F 3 "~" H 900 5700 50  0001 C CNN
	1    900  5700
	0    1    1    0   
$EndComp
Text Label 600  5550 0    50   ~ 0
VIN
Wire Wire Line
	900  5550 600  5550
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5D227A78
P 900 5550
F 0 "#FLG0102" H 900 5625 50  0001 C CNN
F 1 "PWR_FLAG" V 900 5678 50  0000 L CNN
F 2 "" H 900 5550 50  0001 C CNN
F 3 "~" H 900 5550 50  0001 C CNN
	1    900  5550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5D2301B9
P 900 5700
F 0 "#PWR0105" H 900 5450 50  0001 C CNN
F 1 "GND" V 905 5572 50  0000 R CNN
F 2 "" H 900 5700 50  0001 C CNN
F 3 "" H 900 5700 50  0001 C CNN
	1    900  5700
	0    1    1    0   
$EndComp
$EndSCHEMATC