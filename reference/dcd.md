# Directly Connected Disks 

In order to be able to connect hard drives to Macintosh computers before adopting SCSI with the Macintosh Plus, Apple created the **Directly Connected Disks** (DCD) specification.

DCD uses the floppy disk drive controller (IWM/SWIM) as a high-speed serial interface communicating a high-level protocol at 47/96 (~0.490) MHz using a 7-to-8 encoding which reduces the effective speed to 329/768 (~0.428) MHz.  This is almost double the maximum speed (230.4 kHz) that the SCC can achieve when internally clocked, but considerably less than the speed achievable using SCSI.

Multiple DCD devices can be connected to a single Macintosh in a daisy chain, optionally with a single floppy disk drive at the end of the chain.  The protocol places no limit on the number of chained DCD devices, but in practice, it is limited by ROM support to 2 or 4.  On Macs that only support 2 chained DCD devices, connecting a chain of more than 2 will cause the Mac to only recognize the first.

All signals from the IWM/SWIM are common to all floppy disk drive connectors in a Macintosh except for the enable (!ENBL) signal; the IWM/SWIM has two of these (!ENBL1 and !ENBL2) with the intention of being able to control two floppy disk drives.  All known Macintosh ROMs that have support for DCD will only recognize DCD devices when !ENBL2 is used as the !ENBL signal.  When a Macintosh has an external 19-pin D-sub (Often referred to as "DB-19", but this is incorrect as "DB" in the de facto [D-subminiature](https://en.wikipedia.org/wiki/D-subminiature) standard refers to the shell size which accommodates 25 pins and is used for PC parallel ports.) floppy disk drive connector, !ENBL2 is always routed to it, but !ENBL2 can still be used to connect devices even on a Macintosh that does not have such a connector.

## Macintosh Compatibility Matrix

| Model         | Firmware Support        | !ENBL2 Location               | Max Devices | Tested By |
| ------------- | ----------------------- | ----------------------------- | ----------- | --------- |
| 512K          | With Hard Disk 20 Patch | External 19-Pin D-Sub         | 4?          |           |
| 512Ke         | In ROM                  | External 19-Pin D-Sub         | 4           | Tashtari  |
| Plus          | In ROM                  | External 19-Pin D-Sub         | 4           |           |
| SE            | In ROM                  | External 19-Pin D-Sub         | 2           |           |
| IIx           | With Nonstandard ROM    | Second Internal Floppy Header | 2           |           |
| SE/30         | With Nonstandard ROM    | External 19-Pin D-Sub         | 2           |           |
| IIcx          | With Nonstandard ROM    | External 19-Pin D-Sub         | 2           | demik     |
| IIci          | In ROM                  | External 19-Pin D-Sub         | 2?          |           |
| Portable      | In ROM                  | External 19-Pin D-Sub         | 2?          |           |
| Classic       | In ROM                  | External 19-Pin D-Sub         | 2           | Tashtari  |
| LC            | In ROM                  | Second Internal Floppy Header | 2?          |           |
| IIsi          | In ROM                  | External 19-Pin D-Sub         | 2?          |           |
| Classic II    | In ROM                  | External 19-Pin D-Sub         | 2           | Tashtari  |
| PowerBook 100 | In ROM                  | External HDI-20 Connector?    | 2?          |           |
| LC II         | In ROM                  | SWIM PLCC Pin 19              | 2?          |           |
| IIvi          | In ROM                  | SWIM PLCC Pin 19              | 2?          |           |
| IIvx          | In ROM                  | SWIM PLCC Pin 19              | 2?          | Fizzbinn  |

Cells marked with ? are speculated to be true but untested.

## Implementations

### Hard Disk 20

The Hard Disk 20 was the only contemporaneous product released by Apple or any other company following the DCD specification.

More Info:
  * [Hard Disk 20](https://en.wikipedia.org/wiki/Hard_Disk_20)

### Floppy Emu

In 2014, the Floppy Emu by Big Mess O' Wires added support for DCD.

More Info:
  * [Floppy Emu Homepage](https://www.bigmessowires.com/floppy-emu/)
  * [Blog Post, February 2014](https://www.bigmessowires.com/2014/02/06/emulating-the-apple-hd20/)
  * [Blog Post, November 2014](https://www.bigmessowires.com/2014/11/22/reverse-engineering-the-hd20/)

### TashTwenty

TashTwenty is an open source firmware implementation of DCD written by Tashtari for the Microchip PIC16F1704.

More Info:
  * [GitHub Repository](https://github.com/lampmerchant/tashtwenty/)
  * [68kMLA Forum Thread](https://68kmla.org/bb/index.php?threads/tashtwenty-single-chip-dcd-hard-disk-20-interface.39357/)

## References

  * [Directly Connected Disks Specification 1.2a](http://bitsavers.trailing-edge.com/pdf/apple/disk/hd20/Directly_Connected_Disks_Specification_1.2a_May85.pdf)
    * Note that this document has several differences from the protocol as implemented.
