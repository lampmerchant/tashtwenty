# TashTwenty

## Elevator Pitch

It's a DCD (the interface used by the Hard Disk 20 to plug in through the disk port) interface, contained entirely within a PIC16F1825 (14 pins, ~$1.84) or PIC16F1704 (14 pins, ~$1.39) microcontroller. It bit-bangs the low-level IWM protocol using the read/write and phase lines on the "DB"-19 disk port, no programmable logic devices of any kind required.

It interfaces to an MMC or compatible card and can emulate up to four DCD devices.  It also has an enable output which can be used to chain more DCD devices (though ROMs only support a maximum of four) and a floppy disk drive.

(Despite the name, there is nothing limiting it to 20 megabytes.)


## Project Status

Functional, though not rigorously tested.


## Caveats

Due to the dearth of documentation on the DCD protocol, the protocol only implements the commands whose formats are known, namely read, write, and device identification.  Responses to other commands are faked.  Fortunately, this seems to be enough for the device to function properly, including formatting.

The disk drive interface relies on use of the phase lines to read and write one-bit registers.  Because the PIC16F1825 firmware mimics the register set in code instead of programmable logic, it has a response time that is, in the worst case, approximately one microsecond.  This may interfere with the ability of faster Macs to detect it, though it is not known to do so at this point.  The PIC16F1704 firmware mimics the register set in logic and responds in effect instantly.

Certain Macs may have a limitation imposed by their ROM on the number of DCDs they support.  See [this link](https://github.com/lampmerchant/tashnotes/blob/main/macintosh/floppy/dcd/dcd.md) for details.


## PCBs and Products

* TashTwenty Tiny by [demik](https://github.com/demik)
   * [Files](https://github.com/lampmerchant/tashtwenty/tree/main/pcb/TashTwenty%20Tiny)
   * See latest release in this repository for gerbers
   * [Buy](https://ko-fi.com/s/2bfee029f5) from [CayMac Vintage](https://ko-fi.com/caymacvintage/shop)
   * Buy [PCBs](https://ko-fi.com/s/01a52bed98), [case panels](https://ko-fi.com/s/50199947e3), and [programmed PICs](https://ko-fi.com/s/2b28fb5562) from [Tashtari](https://ko-fi.com/tashtari/shop)
* TashTwenty Internal by [cheesestraws](https://github.com/cheesestraws)
   * [Files](https://github.com/lampmerchant/tashtwenty/tree/main/pcb/Internal)
   * See latest release in this repository for gerbers
* TashTwenty Rev 3 by [Tashtari](https://github.com/tashtari)
   * [Files](https://github.com/lampmerchant/tashtwenty/blob/main/pcb/tashtari-tashtwenty.brd)
   * See latest release in this repository for gerbers
* TashTwenty Budget Version by [CayMac Vintage](https://ko-fi.com/caymacvintage/shop)
   * [Buy](https://ko-fi.com/s/77f53b293e)
* TashTwenty Mega by [warmech](https://68kmla.org/bb/index.php?members/warmech.2497/)
   * Pending release
* TashTwenty Vertical by [hideehoo](https://68kmla.org/bb/index.php?members/hideehoo.4611/)
   * Pending release


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

See [this link](https://github.com/lampmerchant/tashnotes/tree/main/macintosh/floppy/dcd) for details.
