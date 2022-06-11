# TashTwenty Tiny

This is a dual PCB design, using mostly through hole components (with the exeption of the SD connector). It's designed this way to be period correct and easy to build for people who don't like soldering SMT components

This folder contains the source KiKad files for the main board. Please check the DB19_IDC20 subfolder for the DB19 to IDC20 adapter. 

## BOM
### BOM (main board)
Here is the BOM for the main board. Part number are what was tested on prototypes board but you may find alternatives easily, especially resistors and sockets

| Reference(s)          | Value      | Quantity | Notes                                  | Part number           |
|-----------------------|------------|----------|----------------------------------------|-----------------------|
| C1, C2                | 2.2uF      | 2        | radial electrolytic capacitors 4x1.5mm | Jamicon SHR2R2M1HC07M |
| C3, C4                | 100nF      | 2        | ceramic capacitor 5.08mm               | Weltron 453358        |
| D1                    | Green      | 1        | 5mm green LED                          | Kingbright L-53SGD    |
| D2                    | Yellow+Red | 1        | 5mm dual color common cathode LED      | Kingbright L-59EYW    |
| J1                    | SD Card    | 1        | SD Card connector                      | TE 2041021-1          |
| J2                    | IDC20      | 1        | IDC 2x10 Header                        | BLK 10120560          |
| Q1                    | BC337      | 1        | NPN BJT Transistor                     | BC337                 |
| R10, R11              | 330Ω       | 2        | standard 0.25W carbon film resistor    | TRU TC-CFR0W4J0331    |
| R1, R3, R5, R8        | 1200Ω      | 4        | standard 0.25W carbon film resistor    | TRU TC-CFR0W4J0122    |
| R2, R4, R6            | 2200Ω      | 3        | standard 0.25W carbon film resistor    | TRU TC-CFR0W4J0222    |
| R7, R9, R12, R13, R14 | 10kΩ       | 5        | standard 0.25W carbon film resistor    | TRU TC-CFR0W4J0103    |
| U1                    | PIC16F1704 | 1        | PIC 8-bit Microcontroller (DIP-14)     | PIC16F1704-I/P        |
| U2                    | 74ACT08    | 1        | Quad TTL 2-Input AND Gate              | SN74ACT08N            |
| U3                    | 3.3V LDO   | 1        | 3.3V LDO voltage regulator             | MCP1700-3302E/TO      |

Optional

| Reference(s)          | Value      | Quantity | Notes                                  | Part number           |
|-----------------------|------------|----------|----------------------------------------|-----------------------|
| J1                    | -          | 1        | 1x2 header + jumper or wire            | -                     |
| U1, U2                | socket     | 2        | DIP-14 socket                          | TRU 14-LC-TT          |
| Case                  | ABS        | 1        | Instrument Case, ABS 2.6x2.6"          | HM 1593K(TBU\|GY\|BK) |

Using sockets is recommended because it will allow you to reclaim the gates and microcontroler if your board is broken. This will also allow you to upgrade the firmware if a future firmware is released.

Bridging J3 connects the !ENBL pin on the microcontroller to the !ENBL pin on the 2x20 header, which is what you want for the overwhelming majority of cases where you're connecting the board to an external/secondary floppy drive port that has !ENBL2 in the connector where it should be. It's fine to bridge the two pins with a piece of wire, but if you want this expandability available to you, it'd be better to solder a two-pin header and stick a jumper on it when not in use. The pin closest to the center of the board is the one connected to the microcontroller.

### BOM (adapter board)
This is a matching board for TashTwenty. Please use this one to avoid any damage to your computer or TashTwenty. Others adapters boards are wired differently

| Reference(s)          | Value      | Quantity | Notes                                  | Part number        |
|-----------------------|------------|----------|----------------------------------------|--------------------|
| J1                    | IDC20      | 1        | IDC 2x10 Header                        | BLK 10120560       |
| J2                    | DB19       | 1        | DB19 wire male connector               | Good luck.         |

### BOM (external links)
Here is a mouser link (without the case):
- https://www.mouser.fr/ProjectManager/ProjectDetail.aspx?AccessID=4e3ee30a16

Here us a digikey (without the case):
- https://www.digikey.com/short/n4d8tf04


### IDC cable
You need a cable to connect both boards
Any straight IDC-20 cable should work (pin 1 to pin 1, etc). Macintosh floppy cables are compatible. We used a few Apple cables for prototyping:
- Quadra 800 (Apple P/N 590-0862-B)
- PowerMac 7200-7600 (Apple P/N 590-4529-A)

If you are looking for a new one, a compatible assembly is made buy Advantech under the part number PCL-10120-1E.

## PCBs
the PCBs are simple 2 layers boards. The gerbers are avaible in the release section.
Check for the following files:
* TTT_12.zip: main board revision 1.2
* DB19_IDC20.zip: adapter board revision 1.0

You should be use any mainstream PCB manufacturer for theses. Nothing special about them. PCB thickness should FR-4 with a 1.6mm thickness
On JLCPCB, select "Specify a location" to the option "Remove Order Number"

## Building
Building is straightworfard. It's recommended to start with the SD Card connector, then components, and headers + sockets last.
