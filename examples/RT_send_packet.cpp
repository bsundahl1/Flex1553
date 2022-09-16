/* This example configures the Teensy as a MIL-1553 Remote Termianl.
   This will wait for the BC to request a data packet, it will acknowledge
   the request and send the data. This is all done with interrupts.
   The data has to be setup in advance of the request.

   This uses two FlexIO modules, one for transimit, one for receive
*/

#include <Arduino.h>
#include <Flex1553.h>
#include <MIL1553.h>

// Send data from RT 5  subaddress 1 or 2 (no particular reason why these numbers)
#define RTA       5   // RT address
#define MB1_SA    1   // subaddress
#define MB1_WC    1   // word count
#define MB2_SA    2   // subaddress
#define MB2_WC    5   // word count

// prototypes
void printPacket(MIL_1553_packet *packet);


// global variables
const int ledPin = 13;
const int rx1553Pin = 6;
unsigned long loopCount = 0;
bool ledState  = false;
//int  rxCount   = 0;
//uint16_t mb1Data = 0x1234;


// Instantiate 1553 classes

FlexIO_1553TX flex1553TX(FLEXIO1, FLEX1553_PINPAIR_3);
// This configures FlexIO1 to send 1553 words.
// This is a physical transmitter layer, and is not related to the TRANSMIT packet direction.
// Pin pairs configure the Teensy pins to be used for sending differential data,
// in this case, pins 2 & 3. Pin pairs are defined in Flex1553.h

FlexIO_1553RX flex1553RX(FLEXIO2, rx1553Pin);
// This configures FlexIO2 to read 1553 words.
// This is a physical receive layer, and is not related to the RECEIVE packet direction.

MIL_1553_RT  myRemoteTemrinal(&flex1553TX, &flex1553RX, FLEX1553_BUS_A);
// This class supports the basic packet operations of a 1553 remote terminal.
// The class supports only one RT address and one Bus, and up to 32 subaddresses.
// Two instances of this class are allowed, intended for BusA and BusB
// The two instances could have the same, or different RT addresses, but they
// MUST be assigned to separate buses.
// This calls the FlexIO_1553TX and FlexIO_1553RX instances for the physical I/O

MIL_1553_packet mailBox1;
MIL_1553_packet mailBox2;
// These are contaiers for the packet data



void setup() {
   // initialize serial port
   Serial.begin(115200);
   while (!Serial)
      ; // wait for serial port to connect
        // This is only important if you want to capture the startup
        // messages below.
        // Because the Teensy uses a USB virtual serial port, it takes a
        // second or so after program startup for the serial port to
        // connect. Any serial data sent before this is lost.

   pinMode(ledPin, OUTPUT);

   // start 1553
   if(!myRemoteTemrinal.begin(RTA))
      Serial.println( "myRemoteTemrinal.begin() failed" );
      // This starts all three 1553 instances (flex1553TX, flex1553RX, and myRemoteTemrinal)
      // begin() will also print pin configuration info and other messages
      // which may be useful for debugging.  This can be turned off by
      // commenting out the FLEX_PRINT_MESSAGES flag in Flex1335TX.cpp
      // and Flex1335RX.cpp

   // one time user prompt
   Serial.println();
   Serial.println("1553 RT Example");

   // Setup mailbox 1
   // This class implements a 1553 subaddress as a mailbox. Incomming (Recive)
   // and outgoing (Transmit) mailboxes are independant.
   //mailBox1.setWordCount(MB1_WC);  // set word count in packet
   mailBox1.setData(0, (uint16_t)0x1234);    // set default data using index
   mailBox1.newMail = true;        // set flag so that we will know when the data is read
   ;
   // assign mailbox to specified subaddress
   if(!myRemoteTemrinal.openMailbox(MB1_SA, MB1_WC, MIL_1553_RT_OUTGOING, &mailBox1))
      Serial.println("Open mailbox1 failed");

   //Setup mailbox 2
   uint16_t data[] = {0x0012, 0x0034, 0x0056, 0x0078, 0x0910};
   //mailBox2.setWordCount(MB2_WC);  // set word count in packet
   mailBox2.setData(data, MB2_WC);   // set data from array, and sets newMail flag
   // assign mailbox to specified subaddress
   if(!myRemoteTemrinal.openMailbox(MB2_SA, MB2_WC, MIL_1553_RT_OUTGOING, &mailBox2))
      Serial.println("Open mailbox2 failed");

}



void loop()
{
   // This will check all mailboxes assigned to this class, and return the
   // first one that is "empty". A mailbox and a packet are really the
   // same thing. PACKET refers to the data structure, and a MAILBOX is
   // more of an analogy.
   MIL_1553_packet* packet = myRemoteTemrinal.mailSent();
   if(packet != NULL) {
      Serial.print("Data was read from SA ");
      Serial.println(packet->getSubAddress());
      // this is where you would write new data to the packet, and then
      // set the newMail flag. Here, the purpose of the flag is just so
      // that we know when the mail has been picked up, but it can also
      // be used to LOCK the mailbox, so that the BC can not read old data.
      packet->newMail = true;
   }

   // Check to see if something went wrong
   // This is totally optional, but helps a lot to figure out configuration problems
   if(myRemoteTemrinal.errorAvailable())
      myRemoteTemrinal.printErrReport();


   // check serial port
      if (Serial.available() > 0)
      {
         char c = Serial.read();
         if(c == 'n') {
            uint16_t newData = Serial.parseInt();
            Serial.print("new data = ");
            Serial.print(newData, DEC);
            Serial.print(", 0x");
            Serial.println(newData, HEX);
            mailBox1.setData(0, newData);
         }

         //Serial.print(c); // echo character back to serial port
         if(c == 'd')
            myRemoteTemrinal.dumpInternalState();

         if(c == 'p')
            myRemoteTemrinal.dumpMailboxAssigments();
      }


   // blink LED continuously so we can tell if something hangs
      if( (loopCount % 0x0400000L) == 0 ){  // once every few million loops (dang, this think is fast!)
          ledState = !ledState;             // toggle the LED
          digitalWrite(ledPin, ledState);
      }

      loopCount++;
}



// Dumps the packet content to the serial port
void printPacket(MIL_1553_packet *packet)
{
   int wc = packet->getWordCount();

   Serial.print(" RTA:");
   Serial.print(packet->getRta());
   Serial.print(" SA:");
   Serial.print(packet->getSubAddress());

   // print status word
   Serial.print(", STATUS:0x");
   Serial.print(packet->getStatusWord(), HEX);

   // Dump out the packet data
   Serial.print(", Payload[");
   Serial.print(wc);
   Serial.print("]:");
   for(int i=0; i<wc; i++)
   {
      Serial.print(packet->getData(i), HEX);
      Serial.print(" ,");
   }
   Serial.println();
}



