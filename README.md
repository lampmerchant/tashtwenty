# TashTwenty

## Elevator Pitch

It's a DCD (the interface used by the Hard Disk 20 to plug in through the disk port) interface, contained entirely within a PIC16F1825 (14 pins, ~$1.84) or PIC16F1704 (14 pins, ~$1.39) microcontroller. It bit-bangs the low-level IWM protocol using the read/write and phase lines on the "DB"-19 disk port, no programmable logic devices of any kind required.

It interfaces to an MMC or compatible card and can emulate up to four DCD devices.  It also has an enable output which can be used to chain more DCD devices (though ROMs only support a maximum of four) and a floppy disk drive.

(Despite the name, there is nothing limiting it to 20 megabytes.)


## Project Status

Functional, though not rigorously tested.


## Caveats

Due to the dearth of documentation on the DCD protocol (see below), the protocol only implements the commands whose formats are known, namely read, write, and device identification.  Responses to other commands are faked.  Fortunately, this seems to be enough for the device to function properly, including formatting.

The disk drive interface relies on use of the phase lines to read and write one-bit registers.  Because the PIC16F1825 firmware mimics the register set in code instead of programmable logic, it has a response time that is, in the worst case, approximately one microsecond.  This may interfere with the ability of faster Macs to detect it, though it is not known to do so at this point.  The PIC16F1704 firmware mimics the register set in logic and responds in effect instantly.

Certain Macs may have a limitation imposed by their ROM on the number of DCDs they support.  


### Compatibility

#### Hardware

| Macintosh  | PIC16F1825 | PIC16F1704 |
| ---------- | ---------- | ---------- |
| 512k¶      | 4?         | 4?         |
| 512ke      | 4          | 4          |
| Plus       | 4          | 4?         |
| SE         | 2?         | 2?         |
| Classic    | 2?         | 2?         |
| Portable   | 2?         | 2?         |
| IIci       | 2?         | 2?         |
| IIsi       | 2?         | 2?         |
| LC†        | 2?         | 2?         |
| LC II§     | 2?         | 2?         |
| IIx†‡      | 2?         | 2?         |
| IIcx‡      | 2          | 2?         |
| SE/30‡     | 2?         | 2?         |
| Classic II | 2          | 2          |

? Suspected, but not known to have been tested

¶ Requires Hard Disk 20 patch file (often erroneously called the Hard Disk 20 INIT)

§ Requires clipping onto the !ENBL2 pin of the SWIM IC

† Requires use of the secondary internal floppy drive header

‡ Requires use of a nonstandard ROM


#### System Software

| Version | Compatible? |
| ------- | ----------- |
| 6.0.8   | Yes         |
| 7.1     | Yes         |
| 7.5     | No          |


## Technical Details

### Building Firmware

Building the firmware requires Microchip MPASM, which is included with their development environment, MPLAB.  Note that you **must** use MPLAB X version 5.35 or earlier or MPLAB 8 as later versions of MPLAB X have removed MPASM.


### MMC Card Format

The MMC card must have an MBR (Master Boot Record) aka DOS-type partition table with up to four primary partitions of type 0xAF (HFS).  Extended partitions are not supported.


#### Custom Icons

In the PIC16F1704 firmware, custom icons are supported using other values besides 0xAF for the partition type byte.  Other partition types besides 0xAF must be the number of a sector (for example, a partition of type 0x01 would point to the second sector) on the card formatted as follows:

* Bytes 0-255 must be the word "ICON" repeated 64 times.
* Bytes 256-383 must be a 32x32 icon, one bit per pixel, with '1' bits denoting black and '0' bits denoting white.
* Bytes 384-511 must be a 32x32 icon mask, one bit per pixel, with '1' bits denoting opaque and '0' bits denoting transparent.

If the sector is not in this format, the partition will be ignored, in order to safely support cards with non-HFS partitions.  Partitions of type 0xAF will always use the built-in icon and the hundred-seventy-sixth sector will not be checked for the format described above.


### DCD (Directly Connected Disks) Protocol

#### Details Missing or Inaccurate in the [May 1985 DCD Document](http://bitsavers.trailing-edge.com/pdf/apple/disk/hd20/Directly_Connected_Disks_Specification_1.2a_May85.pdf)

* The sync byte, in either direction, is always 0xAA, 0x96 is not used.
* When Mac is transmitting a command, the sync byte is followed by two more raw IWM bytes before the 7-to-8 groups begin. DCD transmits only a sync byte and does not transmit these extra bytes.
   * The first is 0x80 plus the number of 7-to-8 groups in the command being transmitted by Mac.
   * The second is 0x80 plus the number of 7-to-8 groups that Mac expects to receive in response.
* The holdoff protocol is completely different than specified.
   * In either direction, a holdoff is initiated by Mac transitioning from state 1 to state 0.
   * If Mac is transmitting, it will finish the 7-to-8 group that it has begun transmitting. This data is valid.
   * If DCD is transmitting, it must finish the 7-to-8 group that it has begun transmitting. This data will be treated by Mac as valid.
   * A holdoff is ended by Mac transitioning from state 0 to state 1. There is no negotiation.
   * If Mac is transmitting and WR is high at the end of the last byte transmitted before holdoff, it will transition WR from high to low right before transitioning back to state 1.
   * Mac will resume transmission with an 0xAA byte, followed by the bytes in the next group after the group where the holdoff began.
   * DCD must resume transmission with an 0xAA byte, followed by the bytes in the next group after the group where the holdoff began.
* The Controller Status (command 0x03) block is slightly different than specified.
   * The total size of the Controller Status block is 336 bytes, not 532 bytes.
      * 336 bytes of data, 6 byte header, checksum byte == 343 bytes == 49 7-to-8 groups
   * The Icon field contains a 32x32 icon as a 1-bit bitmap, followed by its 32x32 mask, also as a 1-bit bitmap, for a total of 256 bytes.
      * The format of the bitmaps is identical to that of ICON resources.
   * The Filler field is replaced by a 16-byte Pascal string (first byte is length) that determines what appears in the "Where:" field of the Get Info dialog box.
* The checksum byte is chosen such that all data bytes in all 7-to-8 groups (not including the sync byte or the command/response length IWM bytes) sum to 0 modulo 256.


#### Details Out of Scope for DCD Documentation But Useful to Know

* The IWM transmits and receives MSB first and the MSB is always set; the chip uses this for timing.
* The IWM transmits at its "fast" speed, 47/96 MHz, or approximately 489.58 Kbps, data cell width 2.043 us.
* The IWM's output (on WR pin) is in NRZI format (inversion == 1, no inversion == 0).
* The IWM's input (on RD pin) detects only falling edges.


#### Beyond

Other details about the protocol at the signal level and the read, write, and controller status commands are accurately reported by the [May 1985 document](http://bitsavers.trailing-edge.com/pdf/apple/disk/hd20/Directly_Connected_Disks_Specification_1.2a_May85.pdf); other commands are unknown to me, but hopefully this information will be useful to anyone in the future who wishes to implement DCD.
