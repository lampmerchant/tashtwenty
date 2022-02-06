# TashTwenty

## Elevator Pitch

It's a DCD (the interface used by the Hard Disk 20 to plug in through the disk port) interface, contained entirely within a PIC16F1825
(14 pins, ~$1.70) microcontroller. It bit-bangs the low-level IWM protocol using the read/write and phase lines on the "DB"-19 disk
port, no programmable logic devices of any kind required.

It interfaces to an MMC card and can emulate up to four DCD devices.  It also has an enable output which can be used to
chain more DCD devices (though Mac OS only supports a total of four) and a floppy disk drive.

(Despite the name, there is nothing limiting it to 20 megabytes.)


## Project Status

Functional, though not rigorously tested.


## Caveats

Due to the dearth of documentation on the DCD protocol (see below), the protocol only implements the commands whose formats are known,
namely read, write, and device identification.  Responses to other commands are faked.  Fortunately, this seems to be enough for the
device to function properly, including formatting.


## Technical Details

### MMC Card Format

The MMC card must have an MBR (Master Boot Record) aka DOS-type partition table with up to four primary partitions of type 0xAF (HFS).  Partitions of other types will be ignored.  Extended partitions are not supported.


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
