EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "2022-05-17"
Rev "1.0"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:TestPoint TP1
U 1 1 6246096F
P 6000 5000
F 0 "TP1" H 6058 5118 50  0000 L CNN
F 1 "TestPoint" H 6058 5027 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 6200 5000 50  0001 C CNN
F 3 "~" H 6200 5000 50  0001 C CNN
	1    6000 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 5000 6000 5250
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 6245F89A
P 5500 5000
F 0 "#FLG0101" H 5500 5075 50  0001 C CNN
F 1 "PWR_FLAG" H 5500 5173 50  0000 C CNN
F 2 "" H 5500 5000 50  0001 C CNN
F 3 "~" H 5500 5000 50  0001 C CNN
	1    5500 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 5000 5500 5250
Wire Wire Line
	5500 5250 6000 5250
Connection ~ 6000 5250
Wire Wire Line
	6000 5250 6000 5500
$Comp
L Connector:TestPoint TP2
U 1 1 6245FB73
P 6000 4500
F 0 "TP2" H 6058 4618 50  0000 L CNN
F 1 "TestPoint" H 6058 4527 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 6200 4500 50  0001 C CNN
F 3 "~" H 6200 4500 50  0001 C CNN
	1    6000 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 4500 6000 5000
Connection ~ 6000 5000
$Comp
L power:Earth #PWR0101
U 1 1 62460A5C
P 6000 5500
F 0 "#PWR0101" H 6000 5250 50  0001 C CNN
F 1 "Earth" H 6000 5350 50  0001 C CNN
F 2 "" H 6000 5500 50  0001 C CNN
F 3 "~" H 6000 5500 50  0001 C CNN
	1    6000 5500
	1    0    0    -1  
$EndComp
$EndSCHEMATC
