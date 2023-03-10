/* This example configures the Teensy as a MIL-1553 Bus Controller.
   Every time the ENTER key is pressed it will request data from an RT.
   If a properly configured RT is on the 1553 bus, it should respond
   with a status word and data.

   The number of data words requested may be be entered from the
   serial port. Captured data will be sent to the serial port.

   This uses two FlexIO modules, one for transimit, one for receive
*/

#include <Arduino.h>
#include <Flex1553.h>
#include <MIL1553.h>

// Get data from RT 5  subaddress 2 (no particular reason why these numbers)
#define RTA   5   // RT address
#define SA    3   // subaddress

// prototypes
void doIt(String str);
void printPacket(MIL_1553_packet *packet);


// global variables
const int ledPin = 13;
const int rx1553Pin = 6;
unsigned long loopCount = 0;
bool ledState  = false;
int  rxCount   = 0;


// Instantiate 1553 classes

FlexIO_1553TX flex1553TX(FLEXIO1, FLEX1553_PINPAIR_3);
// This configures FlexIO1 to send 1553 words.
// This is a physical transmitter layer, and is not related to the TRANSMIT packet direction.
// Pin pairs configure the Teensy pins to be used for sending differential data,
// in this case, pins 2 & 3. Pin pairs are defined in Flex1553.h

FlexIO_1553RX flex1553RX(FLEXIO2, rx1553Pin);
// This configures FlexIO2 to read 1553 words.
// This is a physical receive layer, and is not related to the RECEIVE packet direction.

MIL_1553_BC  myBusController(&flex1553TX, &flex1553RX);
// This class supports the basic packet operations of a 1553 bus controller.
// This calls the FlexIO_1553TX and FlexIO_1553RX instances for the physical I/O

MIL_1553_packet myPacket;
// This is just a contaier for the packet data



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
   if( !myBusController.begin() )
      Serial.println( "myBusController.begin() failed" );
      // This starts all three 1553 instances (flex1553TX, flex1553RX, and myBusController)
      // begin() will also print pin configuration info and other messages
      // which may be useful for debugging.  This can be turned off by
      // commenting out the FLEX_PRINT_MESSAGES flag in Flex1335TX.cpp
      // and Flex1335RX.cpp

   // one time user prompt
   Serial.println();
   Serial.println("1553 Get Example");
   Serial.println("Enter number of words to get, then press ENTER");
}



void loop()
{
   static String str;
   int wc;

   // check serial port
      if (Serial.available() > 0)
      {
         char c = Serial.read();
         Serial.print(c); // echo character back to serial port
         if(c == '\n') // watch for the ENTER key
         {
            // Pressing ENTER will request the packet
            doIt(str);
            str = "";  // empty the string for next time
         }
         else
            str += c;  // add new characters to the string
            // Enter a number on the keyboard from 1 to 32
      }

   // check for 1553 input
      if(rxCount != myPacket.getRxCount()) {
         rxCount = myPacket.getRxCount();
         wc      = myPacket.getWordCount();

         // Total words sent, including command word.
         // This should always be one. Only a command word is sent to request data.
         Serial.print("  Total words sent:");
         Serial.print(myBusController.wordsSent());

         // Words received.
         // MIL1335.cpp code will turn off RX interrupts after all
         // of the expected data has been recived.
         Serial.print(", Received:");
         Serial.print(rxCount);
         //Serial.print(", ");
         //Serial.print(myBusController.wordsReceived());
         // This is the status word returned by the RTA
         //Serial.print(", Received:0x");
         //Serial.print(myPacket.getStatusWord(), HEX);
         Serial.println();

         // check for errors
         if(myPacket.getParityErr() == true)
            Serial.println("Parity error detected");
         if(myPacket.getBitFault() == true)
            Serial.println("Transition fault detected");

         // if we got the whole packet, dump the data
         if(rxCount == wc + 1) {  // packet data words + STATUS
            printPacket(&myPacket);
            if(myPacket.validatePacket())
               Serial.println("  packet Ok");
            else
               Serial.println("  packet failed validation");
         }
      }


   // blink LED continuously so we can tell if something hangs
      if( (loopCount % 0x0400000L) == 0 ){  // once every few million loops (dang, this think is fast!)
          ledState = !ledState;             // toggle the LED
          digitalWrite(ledPin, ledState);
      }

      loopCount++;
}



// Request data from RT
// Word count is specified in str
void doIt(String str)
{
   int wc = atoi(str.c_str()); // convert input string to an integer value
   if( wc<1 )  wc=1;   // force it to stay between 1 and 32
   if( wc>32 ) wc=32;

   // if reusing a packet, the old contents should probably be cleared.
   // this is not required, but will make it easier to find errors.
   myPacket.clear();
   myPacket.setRta(RTA);
   myPacket.setSubAddress(SA);

   // 1553 defines data direction in terms of the RT (Remote Teminal)
   // Requesting data from an RT is defined as a TRANSMIT
   //myPacket.setTrDir(myPacket::TR_TRANSMIT);

   myPacket.setWordCount(wc);
   rxCount = 0;  // keep track of how much data we get back

   // this will send the request packet and capture a return data words.
   // the returned data will be loaded into the packet.
   // it does not wait for the hardware to finish, it just gets
   // it started and it runs on FlexIO and interrupts from there.
   myBusController.request(&myPacket, FLEX1553_CH_A);

   // just so we know what we are putting on the bus...
   Serial.print(" Command word:0x");
   Serial.print(myPacket.getCommandWord(), HEX);
   Serial.println();

   // the main loop() polls getRxCount() and displays the returned data
}



// Dumps the packet content to the serial port
// This is just to verify the packet content
void printPacket(MIL_1553_packet *packet)
{
   int wc = packet->getWordCount();

   Serial.print(" RTA:");
   Serial.print(packet->getRta());
   Serial.print(" SA:");
   Serial.print(packet->getSubAddress());

   // print status word
   Serial.print(", STATUS:0x");
   Serial.print(myPacket.getStatusWord(), HEX);

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
