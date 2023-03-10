/* This example configures the Teensy as a MIL-1553 Bus Controller
   and sends a 1553 data packet every time the ENTER key is pressed.
   The number of data words may be also be entered from the serial port.
   The data is just sequential numbers: 0,1,2,3...

   The packet will be sent, whether or not there is anything on the
   other end of the bus to capture it. If a valid RT is configured
   on the bus, it should capture this data and return a status word,
   which will display on the serial port.

   This uses two FlexIO modules, one for transimit, one for receive
*/

#include <Arduino.h>
#include <Flex1553.h>
#include <MIL1553.h>

// Send to RT 5 subaddress 2 (no particular reason why these numbers)
#define RTA   5   // RT address
#define SA    2   // subaddress

// prototypes
void doIt(String str);
bool loadDummyData(MIL_1553_packet *packet, int wc);
void printPacket(MIL_1553_packet *packet);


// global variables
const int ledPin = 13;
const int rx1553Pin = 6;
unsigned long loopCount = 0;
bool ledState  = false;
int  rxCount   = 0;
int  txCount   = 0;


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
   Serial.println("1553 Send Example");
   Serial.println("Enter number of words to send, then press ENTER");
}


void loop()
{
   static String str;

   // check serial port
      if (Serial.available() > 0)
      {
         char c = Serial.read();
         Serial.print(c); // echo character back to serial port
         if(c == '\n') // watch for the ENTER key
         {
            // Pressing ENTER will send the packet
            doIt(str);
            str = "";  // empty the string for next time
         }
         else
            str += c;  // add new characters to the string
            // Enter a number on the keyboard from 1 to 32
      }

   // check for change in 1553 TX
      if(txCount != myBusController.wordsSent()) {
         txCount = myBusController.wordsSent();

         // Total words sent, including command word.
         Serial.print("  Total words sent:");
         Serial.print(txCount);

         // Words received should always be one. This is by 1553 packet definition.
         // The MIL1335 code will turn off RX interrupts after receiving one word.
         Serial.print(", Received:");
         Serial.print(rxCount);
         Serial.println();
      }

   // check for 1553 RX input
      if(rxCount != myBusController.wordsReceived()) {
         rxCount = myBusController.wordsReceived();

         // Total words sent, including command word
         Serial.print("  Total words sent:");
         Serial.print(txCount);

         // Words received should always be one. This is by 1553 packet definition.
         // The MIL1335 code will turn off RX interrupts after receiving one word.
         Serial.print(", Received:");
         Serial.print(rxCount);
         Serial.println();

         // This is the status word returned by the RTA
         Serial.print(" STATUS word:0x");
         Serial.print(myPacket.getStatusWord(), HEX);
         Serial.println();

         // check for errors
         if(myPacket.getParityErr() == true)
            Serial.println("Parity error detected");
         if(myPacket.getBitFault() == true)
            Serial.println("Transition fault detected");
      }

   // blink LED continuously so we can tell if something hangs
      if( (loopCount % 0x0400000L) == 0 ){  // once every few million loops
          ledState = !ledState;             // toggle the LED
          digitalWrite(ledPin, ledState);
      }

      loopCount++;
}


// Build a packet and send it
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

   // This loads sequential numbers into the packet
   if(loadDummyData(&myPacket, wc) == false) {
      Serial.println("Error loading packet data");
      return;
   }
   myPacket.setWordCount(wc);
   rxCount = 0;  // keep track of how much data we get back
   txCount = 0;

   // this will send the packet and capture a return status word.
   // it does not wait for the hardware to finish, it just gets
   // it started and it runs on FlexIO and interrupts from there.
   myBusController.send(&myPacket, FLEX1553_CH_A);

   // the main loop() polls getRxCount() to get the Status word

   // print the packet contents to serial port
   // this is just so that we are sure what is in the packet
   printPacket(&myPacket);
   if(myPacket.validatePacket())
      Serial.println("  packet Ok");
   else
      Serial.println("  packet failed validation");

   // just so we know what we are putting on the bus...
   Serial.print(" Command word:0x");
   Serial.print(myPacket.getCommandWord(), HEX);
   Serial.println();

}


// This loads the packet payload with an incrementing pattern
// 0x0001, 0x0203, 0x0405 ...
// @param  packet      pointer to MIL_1553_packet class
// @param  words       word count
// @return boolean: false indicates an error
bool loadDummyData(MIL_1553_packet *packet, int wc)
{
   bool noErrors = true;

   for(int i=0; i<wc; i++) {
      uint16_t data = (i * 2 * 256) + (i * 2 + 1);
         // set data in the following format:
         // word 1: 0x0001
         // word 2: 0x0203
         // word 3: 0x0405
         //  and so on
      if(packet->setData(i, data) == false)
         noErrors = false;    // if any errors, clear the flag
   }

   return noErrors;
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

   // Dump out the packet data just to be sure it loaded right
   Serial.print(" Payload[");
   Serial.print(wc);
   Serial.print("]:");
   for(int i=0; i<wc; i++)
   {
      Serial.print(packet->getData(i), HEX);
      Serial.print(" ,");
   }
   Serial.println();

}
