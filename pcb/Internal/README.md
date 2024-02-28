# Internal

This is cheesey's board for use of TashTwenty internally to a Plus or a 512ke (as far as I know).  It interposes between the IWM and the logic board, and reroutes things so that the back floppy port on the computer still works.

Note that installing this requires the IWM on your Plus/512ke to be socketed!  While building the board is easy, desoldering the IWM from the logic board can be an exercise in applied pain, especially if the solder is crunchy.  Caveat haxor.

To build it, you will need:

* A 74HCT04, a 74HCT02 and a TashTwenty
* A MCP1700-3302E voltage regulator (other 3.3V regulators might do if the pinout is the same)
* An SD card socket.  I used a cheap breakout I got online.  Be aware that the Adafruit micro SD breakout boards have a weird pinout and won't work: the pins on this board are in the same order as they are on the surface mount sockets (and thus on the card themselves), please use one with a boring pinout.
* A handful of capacitors, resistors and LEDs.
* A socket for the IWM and some SIL header rows to 

The PCB is designed to be easy to build.  The component values are marked on the board, I wanted this to feel like a kit to build.  Some notes:

* The SIL headers sticking down from the board go into the leftmost set of DIP holes, the socket on top for the IWM (or the IWM soldered if you're feeling really brave) goes on the right.  Sorry this is unclear on the board; I couldn't work out how to make it clearer.
* Install the SIL headers before the socket or you will hate your life.  Use a DIP socket to hold them in place or you will find that the board won't plug into the DIP socket.
* The LEDs and the resistors next to them (R8-11) are optional.  R1-R7 are *not* optional.
* Do check again that your SD card socket pinout is correct

Notes on installation:

* You may wish to use low-profile IC sockets and low-profile SIL pins if you can: mine only just fits under the metal shielding
* Putting a self-adhesive rubber bumper under the board where it hovers over other chips might well help with keeping everything in place.