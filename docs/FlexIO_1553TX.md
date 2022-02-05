FlexIO 1553 Transmit class
==========================
## Background

MIL-STD-1553 is a serial communication protocol developed for the military
in the late 1970's, and is still used today in many military vehicles and
aircraft. It has also found its way into some industrial applications. It is
very reliable and fairly fast with a 1Mb/s bit rate. You could consider it
a predecessor to CAN bus.

On the down side, it is a fairly difficult standard to implement at the
physical layer and is generally must be done with custom peripheral IC's,
or in recent years, with an FPGA. FlexIO gives us a new tool to use for
custom or uncommon protocols. This might not be a military grade solution,
but it is good enough to communicate with 1553 devices, at least for test
purposes.

### Pin pairs

A 1553 transmitter needs two differential output pins to control the
required isolation transformer. Due to the way that FlexIO uses pins in
State Machine mode, these are the only combinations of pins available for
TX:

    Teensy 4.1
    FlexIO_1:         FlexIO_2:          FlexIO_3:
     pair1: n/a        pair1: 10,12       pair1: 19,18
     pair2: n/a        pair2: 11,13       pair2: 14,15
     pair3: 2,3        pair3: n/a         pair3: 40,41
     pair4: 4,33       pair4: n/a         pair4: 17,16

In military use, there are usually two 1553 transmitters, and two
receivers. This is for fail safe redundancy. Only one pin pair is needed
for normal operation, but if two are desired, we can drive two sets of
hardware from a single FlexIO transmitter by using multiple pin pairs. Only
one pair will be enabled at a time (by software) so that it appears to work
as two separate transmitters. Or both may be enabled at the same time to
have a true redundant transmitter.

Each pin pair is listed in the order of ***pos***, ***neg*** pins. These
will control the FET drivers and isolation transformer needed for standard
1553. Pay attention to the order. If you get them backwards, it wont work.
(Yes, it could be fixed in FlexIO).

Configure FlexIO module and the pins pair(s) that you would like to use when
instantiating FlexIO_1553TX().

### Configuration

### Hardware

### References

   https://www.milstd1553.com/wp-content/uploads/2012/12/MIL-STD-1553B.pdf
   https://nepp.nasa.gov/docuploads/43745C0A-323E-4346-A434F4342178CD0E/MIL-STD-1553.pdf
