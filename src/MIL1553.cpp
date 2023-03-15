// Flex1553
// A MIL-STD-1553 library for Teensy 4
// Copyright (c) 2022 Bill Sundahl
// MIT License

/*
   Packet level interface to MIL-STD-1553 transceiver. Lower level classes
   (Flex1553RX & TX) are directly controlling FlexIO hardware, either a
   transmitter or receiver, but not both. A full 1553 transaction requires
   both sending (command or data) and receiving (data or status). This
   class brings it all together.

   This supports sending/getting only a single 1553 transaction at a time
   (up to 32 words of data).
   This class replaces the ISR in Flex1553RX.cpp, with a more sophisticated
   version that work with a packet class.

   MIL_1553_packet
   This class is just a container for packet configuration and data.
   This is where you would set your RTA and data to send to an RT or BC

   MIL_1553_BC
   This is the bus controller class.
   This class supports the basic packet operations of a 1553 bus controller.
   It sets up the control word and handles the acknowledge.
   This requires the use of one FlexIO_1553TX and one or two
   FlexIO_1553RX instances. It could still use some work on the API.

   MIL_1553_RT
   This is the remote terminal class.
   This is setup as mailboxes, where each mailbox refers to one subaddress.
   There may be up to two instances of this class, where each instance is
   used for one 1553 bus.  If using two instances, each will use its own
   instance of FlexIO_1553RX and FlexIO_1553TX is shared between the two.
   If using two buses, the buses operate independently, they do not work as
   a redundant pair.

*/

#include <Arduino.h>
#include <MIL1553.h>
#include <Flex1553.h>


using namespace std;

// It seems that static variables declared inside a packet still have to
// be allocated outide the packet. Seems strange and redundant.
bool    MIL_1553_BC::beginOk = false;
bool    MIL_1553_BC::txActiveFlag = false;
uint8_t MIL_1553_BC::gWordsSent = 0;           // counter incremented by TX ISR to count TX words
uint8_t MIL_1553_BC::gWordsReceivedOnRX0 = 0;  // counter incremented by RX0 ISR to count RX0 words
uint8_t MIL_1553_BC::gWordsReceivedOnRX1 = 0;  // counter incremented by RX1 ISR to count RX1 words
uint8_t MIL_1553_BC::gWordsToSend = 0;         // used by TX ISR, tells TX when to stop
uint8_t MIL_1553_BC::gWordsToGetRX0 = 0;       // used by RX0 ISR, tells RX0 when to stop
uint8_t MIL_1553_BC::gWordsToGetRX1 = 0;       // used by RX1 ISR, tells RX1 when to stop
int     MIL_1553_BC::gDebug = 0;
FlexIO_1553TX   * MIL_1553_BC::gFlexIO_tx0 = NULL;  // allocate memory for static variables
FlexIO_1553RX   * MIL_1553_BC::gFlexIO_rx0 = NULL;
FlexIO_1553RX   * MIL_1553_BC::gFlexIO_rx1 = NULL;
MIL_1553_packet * MIL_1553_BC::gPacket     = NULL;
//MIL_1553_RT     * gRTBusB = NULL;



int8_t MIL_1553_RT::activeTxBus = FLEX1553_BUS_A;
FlexIO_1553TX   * MIL_1553_RT::gFlexIO_tx  = NULL;  // allocate memory for static variables
FlexIO_1553RX   * MIL_1553_RT::gFlexIO_rxA = NULL;
FlexIO_1553RX   * MIL_1553_RT::gFlexIO_rxB = NULL;
MIL_1553_RT     * MIL_1553_RT::gRTBusA     = NULL;
MIL_1553_RT     * MIL_1553_RT::gRTBusB     = NULL;

const int debugPin  = 38;



MIL_1553_BC::MIL_1553_BC(FlexIO_1553TX *tx0, FlexIO_1553RX *rx0, FlexIO_1553RX *rx1)
{
   //poFlexIO_tx0 = tx0;
   //poFlexIO_rx0 = rx0;
   //poFlexIO_rx1 = rx1;

   // static coppies of the pointers
   gFlexIO_tx0 = tx0;
   gFlexIO_rx0 = rx0;
   gFlexIO_rx1 = rx1;
}



bool MIL_1553_BC::begin(void)
{
   gWordsSent = 0;
   bool result = true;

   pinMode(debugPin, OUTPUT);   // debug

   if((gFlexIO_tx0 == NULL) || (gFlexIO_rx0 == NULL))
      return false; // these pointers are required

   // make sure all attached physical layer classes have been started
   if(gFlexIO_tx0 != NULL) {
      if( gFlexIO_tx0->begin() == false )
         result = false;  // function failed
   }
   if(gFlexIO_rx0 != NULL) {
      if( gFlexIO_rx0->begin() == false )
         result = false;  // function failed
   }
   if(gFlexIO_rx1 != NULL) {
      if( gFlexIO_rx1->begin() == false )
         result = false;  // function failed
   }

   // make sure the interrupt flag is clear
   // Shifter1 is the primary interrupt source for TX
   //gFlexIO_tx0->clearInterrupt(FLEXIO_SHIFTERS, 1);

   // attach our interrupt routine to the transmitter ISR callback
   if(!gFlexIO_tx0->attachInterruptCallback(&MIL_1553_BC::isrCallbackTx0))
      result = false;

   // attach our interrupt routine to the receiver ISR callback
   if(!gFlexIO_rx0->attachInterruptCallback(&MIL_1553_BC::isrCallbackRx0))
      result = false;

   if(gFlexIO_rx1 != NULL) {
      if(!gFlexIO_rx1->attachInterruptCallback(&MIL_1553_BC::isrCallbackRx1))
         result = false;
   }

   beginOk = result; // this is a flag thats says all these checks passed
   return(result);
}


// TX interrupt routine
// Note: this is implemented as a static function so that it can be called
// by an interrupt. That means it has direct access to only static variables
// but it can access in-class variables via static pointers.
void MIL_1553_BC::isrCallbackTx0(void)
{
   if(!beginOk)
      return;

   // check transmitter empty flay
   if(gFlexIO_tx0->readInterruptFlag(FLEXIO_SHIFTERS, 1)) {
      // if the Shifter Data flag is set, it means the transmitter is ready
      // to send a new data word
      if(gWordsSent == 0) {
         // the first word sent is always a command
         gFlexIO_tx0->write(FLEX1553_COMMAND_WORD, gPacket->getCommandWord());
         gWordsSent++;
         txActiveFlag = true;
      }
      //else if(gWordsSent <= gPacket->getWordCount()) {
      else if(gWordsSent < gWordsToSend) {
         // if there is still data to send, send the next word
         gFlexIO_tx0->write(FLEX1553_DATA_WORD, gPacket->getData(gWordsSent-1));
         gWordsSent++;
      }
      else { // all data has been sent
         // turn off TX interrupt
         //gFlexIO_tx0->disableInterruptSource(FLEXIO_SHIFTERS, 1);
         gFlexIO_tx0->disableInterruptSource(FLEX1553TX_TRANSMITTER_EMPTY_INTERRUPT);
         // enable RX to capture status word
         //gFlexIO_rx0->read_data(); // make sure receiver is empty
         //gFlexIO_rx0->enableInterruptSource(FLEXIO_SHIFTERS, 1); // RX data
      }
   }

   // check "end of transmit" flag
   if(gFlexIO_tx0->readInterruptFlag(FLEXIO_TIMERS, 6)) {
      // if the Timer2 flag is set, it indicates that the last data bit has been
      // sent from the transmitter, and the transmitter is tri-stated
      //gFlexIO_tx0->disableInterruptSource(FLEXIO_TIMERS, 6);
      gFlexIO_tx0->disableInterruptSource(FLEX1553TX_END_OF_TRANSMIT_DELAYED_INTERRUPT);
      gFlexIO_tx0->clearInterrupt(FLEXIO_TIMERS, 6);
      txActiveFlag = false;
      gDebug++;
   }
}


// RX interrupt routine
// Note: this is implemented as a static function so that it can be called
// by an interrupt. That means it has direct access to only static variables
// but it can access in-class variables via static pointers.
void MIL_1553_BC::isrCallbackRx0(void)
{
   uint32_t flags = gFlexIO_rx0->readInterruptFlags(FLEXIO_SHIFTERS);
   uint32_t data, faults;
   uint8_t parity_bit;

   //if(!beginOk)
   //   return;

   // received one word
   if(flags & 0x02) { // bit 1 is from the Shifter1 status flag
      // This indicates that a new word has been captured by the receiver
      data   = gFlexIO_rx0->read_data(); // get data from receiver and clear interrupt flag
      faults = gFlexIO_rx0->read_faults();
      parity_bit = data & 0x01;        // lowest bit is parity bit
      data = (data >> 1) & 0xffff;     // the next 16-bits is the actual data

      // ignore TX loopback
      if(txActiveFlag)
         return;
         // if transmitter is active, we are just catching TX data in the receiver - ignore it.
         // otherwise, this should be real RX data

      // save data
      if(gWordsReceivedOnRX0 == 0) {
         // the first word sent is always a STATUS word
         gPacket->setStatusWord((uint16_t)data);  // save data in packet statusWord
         gWordsReceivedOnRX0++;

         // the End of Receive interrupt is initially turned off, because it would trigger
         // on the end of the command word (during the bus turn around). But now it should
         // be safe to enable it.
         gFlexIO_rx0->clearInterrupt(FLEX1553RX_END_OF_RECEIVE_INTERRUPT);
         gFlexIO_rx0->enableInterruptSource(FLEX1553RX_END_OF_RECEIVE_INTERRUPT);
      }
      else if(gWordsReceivedOnRX0 < gWordsToGetRX0) {
         //digitalisrFlex3_defaultdebugPin, 1);  // debug
         // all other words must be DATA words
         gPacket->setWord(gWordsReceivedOnRX0 - 1, (uint16_t)data);
         gWordsReceivedOnRX0++;
         //digitalisrFlex3_defaultdebugPin, 0);  // debug
      }

      // turn off interrupt if all data received
      if(gWordsReceivedOnRX0 == gWordsToGetRX0) {
         //digitalisrFlex3_defaultdebugPin, 1);  // debug
         // turn off interrupt, this should be the end of the packet
     //    gFlexIO_rx0->disable();  // turn off RX interrupts
         //digitalisrFlex3_defaultdebugPin, 0);  // debug
      }

      // check for bit faults
      if(faults != 0)
         gPacket->setBitFault(true);
      else
         gPacket->setBitFault(false);

      // check for parity error
      if(gFlexIO_rx0->parity(data) != parity_bit)
         gPacket->setParityErr(true);
      else
         gPacket->setParityErr(false);

      gPacket->setRxCount(gWordsReceivedOnRX0);
   }
   //else if(gFlexIO_rx0->readInterruptFlag(FLEXIO_TIMERS, 2)) {
   else if(gFlexIO_rx0->readInterruptFlag(FLEX1553RX_END_OF_RECEIVE_INTERRUPT)) {
      // "End of Receive" interrupt - indicates that transmitter has stopped sending
      //gFlexIO_rx0->disableInterruptSource(FLEX1553RX_END_OF_RECEIVE_INTERRUPT);
      gFlexIO_rx0->disable();  // turn off RX interrupts - end of packet
      //gFlexIO_rx0->clearInterrupt(FLEXIO_TIMERS, 2);
      gFlexIO_rx0->clearInterrupt(FLEX1553RX_END_OF_RECEIVE_INTERRUPT);
      // not presently used, but this allows it to show up on a logic analyzer
   }
   else if(gFlexIO_rx0->readInterruptFlag(FLEX1553RX_SYNC_FOUND_INTERRUPT)) {
   //else { // found Sync pattern
      // This indicates a match in the sync trigger pattern.
      // This interrupt is unusual. Normally the interrupt is latched and is held
      // active until cleared. But "match continuous mode" of FlexIO does not seem
      // to latch the interrupt. I have not seen the interrupt missed, however if
      // you try to test for the interrupt flag, as in "if(flags & 0x08)", it seems
      // to be timing dependant as to whether or not it works.
      // Instead, test for all other enabled interrupts, and whatever is left
      // is assumed to be the SYNC interrupt.

      //if(flags & 0x08) { // bit 3 is from the Shifter3 status flag
      gFlexIO_rx0->clearInterrupt(FLEX1553RX_SYNC_FOUND_INTERRUPT);
      if(!txActiveFlag) {  // if TX is active, this is the COMMAND word. We are looking for STATUS
         // A valid packet will always receive a STATUS word first, and be followed by
         // zero or more DATA words. We only have the ability to watch for a single
         // sync pattern, so the pattern must initially be set to STATUS, and here we
         // change it to DATA.
         //gFlexIO_rx0->disableInterruptSource(FLEXIO_SHIFTERS, 3);  // SYNC interrupt
         gFlexIO_rx0->disableInterruptSource(FLEX1553RX_SYNC_FOUND_INTERRUPT);
         gFlexIO_rx0->set_sync(FLEX1553_DATA_WORD);
      }
   }
}


void MIL_1553_BC::isrCallbackRx1(void)
{
   uint32_t flags = gFlexIO_rx1->readInterruptFlags(FLEXIO_SHIFTERS);
   uint32_t data, faults;
   uint8_t parity_bit;

   //if(!beginOk)
   //   return;

   // received one word
   if(flags & 0x02) { // bit 1 is from the Shifter1 status flag
      // This indicates that a new word has been captured by the receiver
      data   = gFlexIO_rx1->read_data(); // get data from receiver and clear interrupt flag
      faults = gFlexIO_rx1->read_faults();
      parity_bit = data & 0x01;        // lowest bit is parity bit
      data = (data >> 1) & 0xffff;     // the next 16-bits is the actual data

      // ignore TX loopback
      if(txActiveFlag)
         return;
         // if transmitter is active, we are just catching TX data in the receiver - ignore it.
         // otherwise, this should be real RX data

      // save data
      if(gWordsReceivedOnRX1 == 0) {
         // the first word sent is always a STATUS word
         gPacket->setStatusWord((uint16_t)data);  // save data in packet statusWord
         gWordsReceivedOnRX1++;

         // the End of Receive interrupt is initially turned off, because it would trigger
         // on the end of the command word (during the bus turn around). But now it should
         // be safe to enable it.
         gFlexIO_rx1->clearInterrupt(FLEX1553RX_END_OF_RECEIVE_INTERRUPT);
         gFlexIO_rx1->enableInterruptSource(FLEX1553RX_END_OF_RECEIVE_INTERRUPT);
      }
      else if(gWordsReceivedOnRX1 < gWordsToGetRX1) {
         //digitalisrFlex3_defaultdebugPin, 1);  // debug
         // all other words must be DATA words
         gPacket->setWord(gWordsReceivedOnRX1 - 1, (uint16_t)data);
         gWordsReceivedOnRX1++;
         //digitalisrFlex3_defaultdebugPin, 0);  // debug
      }

      // turn off interrupt if all data received
      if(gWordsReceivedOnRX1 == gWordsToGetRX1) {
         //digitalisrFlex3_defaultdebugPin, 1);  // debug
         // turn off interrupt, this should be the end of the packet
     //    gFlexIO_rx0->disable();  // turn off RX interrupts
         //digitalisrFlex3_defaultdebugPin, 0);  // debug
      }

      // check for bit faults
      if(faults != 0)
         gPacket->setBitFault(true);
      else
         gPacket->setBitFault(false);

      // check for parity error
      if(gFlexIO_rx1->parity(data) != parity_bit)
         gPacket->setParityErr(true);
      else
         gPacket->setParityErr(false);

      gPacket->setRxCount(gWordsReceivedOnRX1);
   }
   //else if(gFlexIO_rx0->readInterruptFlag(FLEXIO_TIMERS, 2)) {
   else if(gFlexIO_rx1->readInterruptFlag(FLEX1553RX_END_OF_RECEIVE_INTERRUPT)) {
      // "End of Receive" interrupt - indicates that transmitter has stopped sending
      //gFlexIO_rx0->disableInterruptSource(FLEX1553RX_END_OF_RECEIVE_INTERRUPT);
      gFlexIO_rx1->disable();  // turn off RX interrupts - end of packet
      //gFlexIO_rx0->clearInterrupt(FLEXIO_TIMERS, 2);
      gFlexIO_rx1->clearInterrupt(FLEX1553RX_END_OF_RECEIVE_INTERRUPT);
      // not presently used, but this allows it to show up on a logic analyzer
   }
   else if(gFlexIO_rx1->readInterruptFlag(FLEX1553RX_SYNC_FOUND_INTERRUPT)) {
   //else { // found Sync pattern
      // This indicates a match in the sync trigger pattern.
      // This interrupt is unusual. Normally the interrupt is latched and is held
      // active until cleared. But "match continuous mode" of FlexIO does not seem
      // to latch the interrupt. I have not seen the interrupt missed, however if
      // you try to test for the interrupt flag, as in "if(flags & 0x08)", it seems
      // to be timing dependant as to whether or not it works.
      // Instead, test for all other enabled interrupts, and whatever is left
      // is assumed to be the SYNC interrupt.

      //if(flags & 0x08) { // bit 3 is from the Shifter3 status flag
      gFlexIO_rx1->clearInterrupt(FLEX1553RX_SYNC_FOUND_INTERRUPT);
      if(!txActiveFlag) {  // if TX is active, this is the COMMAND word. We are looking for STATUS
         // A valid packet will always receive a STATUS word first, and be followed by
         // zero or more DATA words. We only have the ability to watch for a single
         // sync pattern, so the pattern must initially be set to STATUS, and here we
         // change it to DATA.
         //gFlexIO_rx0->disableInterruptSource(FLEXIO_SHIFTERS, 3);  // SYNC interrupt
         gFlexIO_rx1->disableInterruptSource(FLEX1553RX_SYNC_FOUND_INTERRUPT);
         gFlexIO_rx1->set_sync(FLEX1553_DATA_WORD);
      }
   }
}


// this will send one packet to a Remote Terminal
// This uses the FlexIO_1553TX class to access the hardware
// Note that in 1553 nomenclature, this is called a RECEIVE command
bool MIL_1553_BC::send(MIL_1553_packet *packet, int8_t bus)
{
   FlexIO_1553RX *pFlexIO_rx = NULL; // local pointer to receiver

   if(packet == NULL)
      return false;

   if(!beginOk)
      return false; // begin() has not been called, or has failed

   // which receiver do we use?
   if(bus==FLEX1553_BUS_B) {
      if(gFlexIO_rx1)
         pFlexIO_rx = gFlexIO_rx1; // use receiver 1
      else
         return false; // no pointer to channel B receiver
   }
   else
      pFlexIO_rx = gFlexIO_rx0; // use receiver 0

   // set up global variables for ISR
   gDebug = 0;
   gPacket  = packet;       // The TX interrupt routine will get data from here
   gWordsSent          = 0;   // reset word counters
   gWordsReceivedOnRX0 = 0;
   gWordsReceivedOnRX0 = 0;
   gWordsToSend   = packet->getWordCount() + 1; // send command + data
   gWordsToGetRX0 = 1;  // get status only
   gWordsToGetRX1 = 1;

   // 1553 defines data direction in terms of the RT (Remote Teminal)
   // Sending data to an RT is defined as a RECEIVE command
   packet->setTrDir(TR_RECEIVE);

   // setup
   gFlexIO_tx0->set_channel(bus);  // set bus channel A or B
   pFlexIO_rx->disable();  // make sure RX interrupts are off
   pFlexIO_rx->clearInterrupt(FLEX1553_ALL_INTERRUPTS);
   pFlexIO_rx->set_sync(FLEX1553_STATUS_WORD);
   pFlexIO_rx->flush();  // get ready to read status word

   // enable TX interupts from FlexIO Shifter1 and Timer2
   gFlexIO_tx0->enableInterruptSource(FLEXIO_SHIFTERS, 1); // transmitter empty interrupt
   gFlexIO_tx0->enableInterruptSource(FLEXIO_TIMERS, 6);   // last bit sent interrupt
   // The TX interrupt routine (isrCallbackTx0) will take it from here, sending a
   // new data word on each interrupt.

   // enable RX interupts
   pFlexIO_rx->enableInterruptSource(FLEXIO_SHIFTERS, 1);  // receiver full interrupt

   return true;
}



// this will request one packet from a Remote Terminal
// This uses the FlexIO_1553RX class to access the hardware
// Note that in 1553 nomenclature, this is called a TRANSMIT command
bool MIL_1553_BC::request(MIL_1553_packet *packet, int8_t bus)
{
   FlexIO_1553RX *pFlexIO_rx = NULL; // local pointer to receiver

   if(packet == NULL)
      return false;

   if(!beginOk)
      return false; // begin() has not been called, or has failed

   // which receiver do we use?
   if(bus==FLEX1553_BUS_B) {
      if(gFlexIO_rx1)
         pFlexIO_rx = gFlexIO_rx1; // use receiver 1
      else
         return false; // no pointer to channel B receiver
   }
   else
      pFlexIO_rx = gFlexIO_rx0; // use receiver 0

   // set up global variables for ISR
   gDebug = 0;
   gPacket  = packet;       // The RX interrupt routine will get data from here
   gWordsSent          = 0;   // reset word counters
   gWordsReceivedOnRX0 = 0;
   gWordsReceivedOnRX0 = 0;
   gWordsToSend   = 1;   // sending a command only
   gWordsToGetRX0 = packet->getWordCount() + 1; // receiving status + data words
   gWordsToGetRX1 = packet->getWordCount() + 1;

   // 1553 defines data direction in terms of the RT (Remote Teminal)
   // Reciving data from an RT is defined as a TRANSMIT command
   packet->setTrDir(TR_TRANSMIT);

   // setup
   gFlexIO_tx0->set_channel(bus);  // set bus channel A or B
   pFlexIO_rx->disable();  // make sure RX interrupts are off
   pFlexIO_rx->clearInterrupt(FLEX1553_ALL_INTERRUPTS);
   pFlexIO_rx->set_sync(FLEX1553_STATUS_WORD);
   pFlexIO_rx->flush();    // get ready to read status word

   // enable TX interupts from FlexIO Shifter1 and Timer2
   gFlexIO_tx0->enableInterruptSource(FLEX1553TX_TRANSMITTER_EMPTY_INTERRUPT);
   gFlexIO_tx0->enableInterruptSource(FLEX1553TX_END_OF_TRANSMIT_DELAYED_INTERRUPT);
   // The TX interrupt routine (isrCallbackTx0) will take it from here, sending a
   // new data word on each interrupt.

   // enable RX interupts
   pFlexIO_rx->enableInterruptSource(FLEX1553RX_RECEIVER_FULL_INTERRUPT);
   pFlexIO_rx->enableInterruptSource(FLEX1553RX_SYNC_FOUND_INTERRUPT);
   //pFlexIO_rx->enableInterruptSource(FLEXIO_TIMERS, 2);    // End of Receive interrupt

   return true;
}




/////////////////////////////////////////////////////////////////////////
//                          MIL_1553_RT                                //

MIL_1553_RT::MIL_1553_RT(FlexIO_1553TX *tx, FlexIO_1553RX *rx, int8_t bus)
{
   myBus = bus;

   // because the pointers are static, there must be two coppies, one for each of
   // the two possible instances of MIL_1553_RT. "bus" is just a convenient way
   // to decide which is which. The TX pointers will be the same for both instances
   // because there is only one TX FlexIO Transmit module
   if(bus == FLEX1553_BUS_A) {
      gFlexIO_tx  = tx;   // static pointer to FlexIO_1553TX
      pFlexIO_rx  = rx;   // non-static pointer
      gFlexIO_rxA = rx;   // static pointer to FlexIO_1553RX
      gRTBusA = this;
   }
   else {
      gFlexIO_tx  = tx;   // static pointer to FlexIO_1553TX
      pFlexIO_rx  = rx;   // non-static pointer
      gFlexIO_rxB = rx;   // static pointer to FlexIO_1553RX
      gRTBusB = this;
   }
}

bool MIL_1553_RT::begin(uint8_t rta)
{
   wordsSentOnTX = 0;
   bool result = true;
   beginOk = false;
   myRta = rta;

   if(rta <= MIL_1553_MAX_RTA)
      myRta = rta;
   else
      result = false;

   pinMode(debugPin, OUTPUT);   // debug

   if(gFlexIO_tx == NULL)
         return false; // this pointer is required

   // start physical layer class
   // if there are two instances of this class, this begin() will be
   // called twice. That should be ok, I think
   if(gFlexIO_tx->begin() == false)
      result = false;  // function failed

   if(myBus == FLEX1553_BUS_A) {
      if( gFlexIO_rxA == NULL)
         return false;  // this pointer is required for bus A

      if( gFlexIO_rxA != pFlexIO_rx )
         return false;  // something went wrong in constructor

      if( gFlexIO_rxA->begin() == false ) // start physical layer class
         result = false;  // function failed

      activeTxBus = FLEX1553_BUS_A; // just to be sure that a valid bus is selected
   }
   else if(myBus == FLEX1553_BUS_B) {
      if(gFlexIO_rxB == NULL)
         return false;  // this pointer is required for bus B

      if( gFlexIO_rxB != pFlexIO_rx )
         return false;  // something went wrong in constructor

      if( gFlexIO_rxB->begin() == false ) // start physical layer class
         result = false;  // function failed

      activeTxBus = FLEX1553_BUS_B;
   }
   else
      return false; // myBus must be A or B

   // attach our interrupt routine to the transmitter ISR callback.
   // this is a static pointer address, so if there are two instances of this class
   // each will assign the same ISR to the same pointer (no harm done)
   if(!gFlexIO_tx->attachInterruptCallback(&MIL_1553_RT::isrCallbackTx))
      result = false;

   // attach our interrupt routine to the receiver ISR callback
   // if there are two instances of MIL_1553_RT, each instance will use
   // a different static copy of isrCallbackRx
   if(myBus == FLEX1553_BUS_A) {
      if(!gFlexIO_rxA->attachInterruptCallback(&MIL_1553_RT::isrCallbackRxA))
         result = false;
   }
   else{  // bus B
      if(!gFlexIO_rxB->attachInterruptCallback(&MIL_1553_RT::isrCallbackRxB))
         result = false;
   }

   beginOk = result; // this is a flag thats says all these checks passed

   // start the RX interrupts running
   pFlexIO_rx->clearInterrupt(FLEX1553_ALL_INTERRUPTS);
   pFlexIO_rx->enableInterruptSource(FLEX1553RX_SYNC_FOUND_INTERRUPT);
   pFlexIO_rx->enableInterruptSource(FLEX1553RX_RECEIVER_FULL_INTERRUPT);
   pFlexIO_rx->enableInterruptSource(FLEX1553RX_END_OF_RECEIVE_INTERRUPT);

   //Serial.print("  gRTBusA=");
   //Serial.print((long)gRTBusA, HEX);
   //Serial.print("  gRTBusB=");
   //Serial.print((long)gRTBusB, HEX);
   //Serial.print("  myBus=");
   //Serial.print(myBus);
   //Serial.println();
   //Serial.print(" gFlexIO_tx =");
   //Serial.print((long)gFlexIO_tx, HEX);
   //Serial.print("  gFlexIO_rxA=");
   //Serial.print((long)gFlexIO_rxA, HEX);
   //Serial.print("  gFlexIO_rxB=");
   //Serial.print((long)gFlexIO_rxB, HEX);
   //Serial.print("  pFlexIO_rx=");
   //Serial.print((long)pFlexIO_rx, HEX);
   //Serial.println();
   //Serial.println();

   return(result);
}


// ISR's are always static, meaning that these routines are shared amoung all
// instances of MIL_1553_RT. This copy is used for bus A.
// Each of these ISR's is attached to a different instance of the MIL_1553_RT class.
// It just detrmines which interrupt flag was triggered, and then
// calls isrRtStateMachine() to do the real work.
void MIL_1553_RT::isrCallbackRxA(void)
{
   if(gFlexIO_rxA == NULL)
      return; // initialization is not complete

   uint32_t sFlags = gFlexIO_rxA->readInterruptFlags(FLEXIO_SHIFTERS);
   uint32_t tFlags = gFlexIO_rxA->readInterruptFlags(FLEXIO_TIMERS);
   intrpt_t intrpt = INT_UNHANDLED_INT;

   // where did this interrupt come from?
   if(sFlags & 0x02) // RX_FULL = Shifter1
      intrpt = INT_RX_FULL;
  // else if(sFlags & 0x08) // RX_SYNC = Shifter3
   else if(tFlags & 0x80) // RX_SYNC = Timer7
      intrpt = INT_RX_SYNC;
   else if(tFlags & 0x04) // RX_EOR = Timer2
      intrpt = INT_RX_EOR;

   //Serial.print("  Callback RXA   INT=");
   //Serial.print(intrpt);
   //Serial.print("  shifters=");
   //Serial.print(sFlags);
   //Serial.print("  timers=");
   //Serial.print(tFlags);
   //Serial.print("  gRTBusA=");
   //Serial.print((long)gRTBusA, HEX);
   //Serial.println();

   // now call the state machine, which is NOT a static function
   if(gRTBusA != NULL)
      gRTBusA->isrRtStateMachine(intrpt);
}

// This copy is used for bus B
void MIL_1553_RT::isrCallbackRxB(void)
{
   if(gFlexIO_rxB == NULL)
      return; // initialization is not complete

   uint32_t sFlags = gFlexIO_rxB->readInterruptFlags(FLEXIO_SHIFTERS);
   uint32_t tFlags = gFlexIO_rxB->readInterruptFlags(FLEXIO_TIMERS);
   intrpt_t intrpt = INT_UNHANDLED_INT;

   // where did this interrupt come from?
   if(sFlags & 0x02) // RX_FULL = Shifter1
      intrpt = INT_RX_FULL;
   else if(tFlags & 0x80) // RX_SYNC = Shifter3
      intrpt = INT_RX_SYNC;
   else if(tFlags & 0x04) // RX_EOR = Timer2
      intrpt = INT_RX_EOR;

   //Serial.print("  Callback RXB   INT=");
   //Serial.print(intrpt);
   //Serial.print("  shifters=");
   //Serial.print(sFlags);
   //Serial.print("  timers=");
   //Serial.print(tFlags);
   //Serial.print("  gRTBusB=");
   //Serial.print((long)gRTBusB, HEX);
   //Serial.println();

   // now call the state machine, which is NOT a static function
   if(gRTBusB != NULL)
      gRTBusB->isrRtStateMachine(intrpt);
}

// only one TX module is allowed. It is shared between instances of MIL_1553_RT
void MIL_1553_RT::isrCallbackTx(void)
{
   if(gFlexIO_tx == NULL)
      return; // initialization is not complete

   intrpt_t intrpt = INT_UNHANDLED_INT;
   uint32_t sFlags = gFlexIO_tx->readInterruptFlags(FLEXIO_SHIFTERS);
   uint32_t tFlags = gFlexIO_tx->readInterruptFlags(FLEXIO_TIMERS);

   // where did this interrupt come from?
   if(sFlags & 0x02) // TX_EMTY = Shifter1
      intrpt = INT_TX_EMTY;
   else if(tFlags & 0x40) // TX_EOT = Timer6
      intrpt = INT_TX_EOT;

   // now call the state machine, which is NOT a static function
   if(activeTxBus == FLEX1553_BUS_A) {
      if(gRTBusA != NULL)
         gRTBusA->isrRtStateMachine(intrpt);
   }
   if(activeTxBus == FLEX1553_BUS_B) {
      if(gRTBusB != NULL)
         gRTBusB->isrRtStateMachine(intrpt);
   }
}


// all of the static ISR's call this one
// This seems like a very long interrupt routine, however only one "intrpt" within one "state"
// is executed per interrupt.
void MIL_1553_RT::isrRtStateMachine(intrpt_t intrpt)
{
   switch(state)
   {
      case ST_IDLE:
         // This state is watching for a command sync. If we find it, advance
         // to the next state.
         if(intrpt == INT_RX_SYNC) {
            // change sync to DATA_SYNC, in antisipation of a RECV command
            pFlexIO_rx->set_sync(FLEX1553_DATA_WORD);

            // get ready for new message
            nfuFlag    = false;
            phyErrFlag = 0;    // this flag is set to different non-zero values to aid in debugging
            msgErrFlag = 0;    // this flag is set to different non-zero values to aid in debugging
            stateErr   = 0;    // more error codes from the state machine
            goodExitFlag = false; // this defaults to flase, and is set true on a good exit
            pPacket    = NULL;
            wordsSentOnTX = 0;
            wordsReceivedOnRX = 0;
            ledRcvFlag = true;  // for RX monitor LED
            state = ST_GET_CMD;
         }

         // Any other interrupt can be assumed to be bus traffic that is
         // NotForUs, ignore it.
         clearInterrupt(intrpt);
         break;

      case ST_GET_CMD:
         // We have detected a CMD_SYNC and are expecting a command word
         // Or this could also be a status word sent by another RT on the bus
         if(intrpt == INT_RX_FULL) {
            // a new word has been captured by the receiver
            uint32_t cmd = pFlexIO_rx->read_data(); // get data from receiver and clear interrupt flag
            //cmd = (cmd >> 1) & 0xffff;     // the next 16-bits is the actual data
            cmd = cmd & 0x1ffff;     // mask off sync bits

            // Check for physical errors
            uint8_t parity_bit = cmd & 0x01;        // lowest bit is parity bit
            bool par_fault = (pFlexIO_rx->parity(cmd >> 1) != parity_bit); // check for parity fault
            bool bit_fault = (pFlexIO_rx->read_faults() != 0); // check for bit faults
            if(par_fault || bit_fault) {
               if(bit_fault) {
                  errBitCnt++;  // log errors
                  phyErrFlag = 1;
               }
               if(par_fault) {
                  errParityCnt++;
                  phyErrFlag = 2;
               }
               // if we didnt receive a valid command, then we cant be sure what we are
               // supposed to do. So we have to ignore the whole message
               pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);  // set receiver to watch for commands
               stateErr = 2; // did not receive a valid command
               state = ST_IDLE;  // ignore command and go back to Idle state
               break;
            }

            // Is this message for us?
            // We are expecting a command, but this could also be the status word of a transaction
            // with another RT which we have ignored. Either way, the RTA bits will be valid.
            msgRta = (cmd >> 12) & 0x1f;
            if(msgRta != myRta) {
               nfuCount++;
               nfuFlag = true;
               //pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);
               //state = ST_IDLE;  // ignore command and go back to Idle state
               break;
            }

            // read command fields
            msgSA    = (cmd >> 6) & 0x1f;  // get subaddress
            msgTrans = (cmd >> 11)& 0x01;  // get  T/R flag
            msgWC    = (cmd >> 1) & 0x1f;  // get word count
            if(msgWC == 0) msgWC = 32;

            // match this with a mailbox/packet class
            if(msgTrans == MIL_1553_TR_RCV) {  // receive = incoming to RT
               pPacket = mailboxInPacket[msgSA];
               state = ST_RX_GET_DATA;
            }
            else { // TR_TRANS  = outgoing from RT
               pPacket = mailboxOutPacket[msgSA];
               pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD); // receiver is not used
                  // in TRANSmit, but I dont want to completely disable it.
                  // CMD_SYNC will minimize transmitter echo.
               state = ST_TX_SEND_STS;
            }

            // check that this mailbox has been assigned
            if(!nfuFlag) {
               if(pPacket == NULL) {  // no mailbox assigned to this subaddress!
                  msgErrFlag = 1;     // this is an error condition, this SA does not exist!
                  // With this flag set, data will not be saved or sent, only a status
                  // will be returned.
               }
               else {
                  // check that data word count matches the expected count
                  if(msgWC != pPacket->getWordCount())
                     // should this really be an error?
                     msgErrFlag = 2;  // wrong word count requested for this SA
               }
            }
            break;
         }
         if(intrpt == INT_RX_EOR) {
            // this shouldnt happen, but we cant use RX_END state if the EOR has
            // has already occured
            clearInterrupt(intrpt);
            pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);  // set receiver to watch for commands
            state = ST_IDLE;
            break;
         }

         {
            //no other interrupt should occur in this state
            logUnhandledInterrupt(intrpt); // log this for debugging
            clearInterrupt(intrpt);
            //pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);  // set receiver to watch for commands
            stateErr = 3;  // unhandled interrupt
            //state = ST_IDLE;
            state = ST_RX_END; // wait for EOR
         }
         break;

      case ST_RX_GET_DATA:
         // A RECV Command has been received, expecting data
         if(intrpt == INT_RX_FULL) {
            uint32_t data = pFlexIO_rx->read_data(); // get data from receiver and clear interrupt flag
            uint8_t parity_bit = data & 0x01;        // lowest bit is parity bit
            data = (data >> 1) & 0xffff;     // the next 16-bits is the actual data

            // Check for errors
            bool par_fault = (pFlexIO_rx->parity(data) != parity_bit); // check for parity fault
            bool bit_fault = (pFlexIO_rx->read_faults() != 0); // check for bit faults
            if(par_fault || bit_fault) {
               if(bit_fault) {
                  errBitCnt++;  // log errors
                  phyErrFlag = 3;
               }
               if(par_fault) {
                  errParityCnt++;
                  phyErrFlag = 4;
               }
               // If we get a bit error at this point in the message, we are not allowed to
               // return anything, or to save any data.
               break;
            }

            // if no problems, copy the data to the packet
            if(!nfuFlag && !msgErrFlag && !phyErrFlag && !pPacket->isBusy()) {
               pPacket->setWord(wordsReceivedOnRX++, (uint16_t)data);
               pPacket->setRxCount(wordsReceivedOnRX);
            }
            break;
         }

         // BC has stopped transmitting, now we need to send the status word back
         if(intrpt == INT_RX_EOR) {
            clearInterrupt(INT_RX_EOR);

            // check for wrong word count
            if(msgWC != wordsReceivedOnRX)
               if(!msgErrFlag) msgErrFlag = 3;

            // generate status word
            uint16_t status = (uint16_t)myRta << 11;
            if(pPacket)
               if(pPacket->isBusy())  status = status | 0x0008;   // set busy bit
            if(msgErrFlag) status = status | 0x0400;              // set error bit

            // if no problems, send the status word
            if(!nfuFlag && !phyErrFlag && msgErrFlag != 3) {
               gFlexIO_tx->set_channel(myBus); // set proper BUS
               activeTxBus = myBus;  // this steers the TX interrupt back to this instance
               gFlexIO_tx->enableInterruptSource(FLEX1553TX_END_OF_TRANSMIT_DELAYED_INTERRUPT);
               if(pPacket)
                  pPacket->setStatusWord(status);
               state = ST_RX_SEND_STS;
               gFlexIO_tx->write(FLEX1553_STATUS_WORD, status); // send status word
            }
            else {
               // if we dont start the transmitter then there will be no EOT interrupt
               // so just go back to idle
               pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);  // set receiver to watch for commands
               stateErr = 4;  // aborting status word due to error
               state = ST_IDLE;
            }
            break;
         }

         if(intrpt == INT_RX_SYNC) {
            // This is bug fix.
            // I had intended for the EOR interrupt to determine the end of the receive
            // data, not the word count. However it turns out that when the BC transmitter
            // turns off, the bus floats high with timing that looks very much like a
            // data sync. Sometimes our sync detector will catch that edge and expect
            // another data word, causing the whole transaction to fail.
            // This fix changes the sync type while receiving the last data word, which
            // prevents the sync detector from being fooled.
            if(wordsReceivedOnRX == (msgWC  -1)) // if next to last word has been received
               // we are now receiving the last data word
               pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);  // set receiver to watch for commands
            clearInterrupt(INT_RX_SYNC);
            break;
         }

         // else log unexpected interrupt
         //errUnhandledRxInterrupt++;
         logUnhandledInterrupt(intrpt); // log this for debugging
         clearInterrupt(intrpt);
         break;

      case ST_RX_SEND_STS:
         // Data has been received, now we are sending a status back
         if(intrpt == INT_RX_FULL) {
            // this is just an echo from our own transmitter, ignore it
            pFlexIO_rx->read_data(); // get data from receiver and clear interrupt flag
            pFlexIO_rx->read_faults();
            break;
         }

         if(intrpt == INT_TX_EOT) {
            clearInterrupt(INT_TX_EOT);
            // disable transmitter interrupts
            gFlexIO_tx->disableInterruptSource(FLEX1553TX_END_OF_TRANSMIT_DELAYED_INTERRUPT);
            if(!nfuFlag && !msgErrFlag && !phyErrFlag) {
               pPacket->newMail = true;  // if mailbox was busy, this will not change anything.
               goodExitFlag = true;  // ok
            }
            pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);  // set receiver to watch for commands
            state = ST_IDLE;
            break;
         }

         // else unexpected interrupt
         logUnhandledInterrupt(intrpt);  // log this for debugging
         clearInterrupt(intrpt);
         break;

      case ST_TX_SEND_STS:
         // A TRANS Command has been received, now we are sending a status back
         if(intrpt == INT_RX_EOR) {
            clearInterrupt(intrpt);

            // generate status word
            uint16_t status = (uint16_t)myRta << 11;
            if(pPacket)
               if(pPacket->isBusy())  status = status | 0x0008;   // set busy bit
            if(msgErrFlag) status = status | 0x0400;              // set error bit

            // if no problems, send the status word
            gFlexIO_tx->set_channel(myBus); // set proper BUS
            activeTxBus = myBus;  // this steers the TX interrupt back to this instance
            if(!nfuFlag && !phyErrFlag && !msgErrFlag) {
               // enable TX interrupts
               gFlexIO_tx->enableInterruptSource(FLEX1553TX_END_OF_TRANSMIT_DELAYED_INTERRUPT);
               gFlexIO_tx->enableInterruptSource(FLEX1553TX_TRANSMITTER_EMPTY_INTERRUPT);
               if(pPacket)
                  pPacket->setStatusWord(status);  // save status word in packet class
               state = ST_TX_SEND_DATA;
               gFlexIO_tx->write(FLEX1553_STATUS_WORD, status); // send status word
            }
            // if there is a mesage error, send the status, but no data
            else if(msgErrFlag) {
               // enable TX EOT interrupt only
               gFlexIO_tx->enableInterruptSource(FLEX1553TX_END_OF_TRANSMIT_DELAYED_INTERRUPT);
               if(pPacket)
                  pPacket->setStatusWord(status);  // save status word in packet class
               //state = ST_TX_SEND_DATA;
               state = ST_TX_END;
               //pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);  // set receiver to watch for commands
               //gFlexIO_tx->disableInterruptSource(FLEX1553_ALL_INTERRUPTS);
               //state = ST_IDLE;
               gFlexIO_tx->write(FLEX1553_STATUS_WORD, status); // send status word
               stateErr = 5;  // aborting TX data due to invalid command
            }
            else { // if we report an error or NFU, we will send no status
               // if we dont start the transmitter then there will be no EOT interrupt
               // so just go back to idle
               pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);  // set receiver to watch for commands
               stateErr = 4;  // aborting status word due to error
               state = ST_IDLE;
               //state = ST_RX_END;
            }
            break;
         }

         logUnhandledInterrupt(intrpt);
         clearInterrupt(intrpt);
         break;

      case ST_TX_SEND_DATA:
         // Status has been sent, now we send the data
         if(intrpt == INT_TX_EMTY) {
            uint16_t data;
            ledTransFlag = true;    // used to trigger a transmit LED
            if(pPacket) {
               data = pPacket->getData(wordsSentOnTX++);  // get data from buffer

               // if last word, disable TX_EMTY interrupt
               if(wordsSentOnTX == pPacket->getWordCount())
                  gFlexIO_tx->disableInterruptSource(FLEX1553TX_TRANSMITTER_EMPTY_INTERRUPT);
            }
            else {
               // this should never happen
               // but if it does happen, we will send one dummy word, and wait for
               // the EOT interrpt
               data = 0;
               gFlexIO_tx->disableInterruptSource(FLEX1553TX_TRANSMITTER_EMPTY_INTERRUPT);
            }
            gFlexIO_tx->write(FLEX1553_DATA_WORD, data); // send data and clear interrupt
            break;
         }

         if(intrpt == INT_TX_EOT) {
            // transaction complete, just a few more house keeping chores
            // disable transmitter interrupts
            gFlexIO_tx->disableInterruptSource(FLEX1553_ALL_INTERRUPTS);
            clearInterrupt(intrpt);

            // if no errors, clear pPacket->newMail flag, which in this case means the new mail was sent
            if(!nfuFlag && !msgErrFlag && !phyErrFlag) {
               pPacket->newMail = false;
               goodExitFlag = true;  // ok
            }
            pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);  // set receiver to watch for commands
            state = ST_IDLE;
            break;
         }

         if(intrpt == INT_RX_FULL) {
            // this is just an echo from our own transmitter, ignore it
            pFlexIO_rx->read_data(); // get data from receiver and clear interrupt flag
            pFlexIO_rx->read_faults();
            break;
         }

         if(intrpt == INT_RX_SYNC) {
            // this is an echo from the status word, which is captured in the
            // ST_TX_SEND_DATA state because the INT_TX_EMTY interrupt will occur at the
            // very start of the status word transmit (as soon as the trasmitter loads).
            // We do not use this interrupt, it is just a transmitter echo
            clearInterrupt(intrpt);
            break;
         }

         if(intrpt == INT_RX_EOR) {
            // EOR occurs here because the RX SYNC is set to watch for commands, so it
            // does not capture the data echoes, the receiver times out after the sync
            // word and sends EOR.
            // We do not use this interrupt.
            clearInterrupt(intrpt);
            break;
         }

         logUnhandledInterrupt(intrpt);
         clearInterrupt(intrpt);
         break;

      case ST_RX_END:  // wait for EOR
         // We come here in the case of an error, to wait for the BC to finsish its
         // transaction before returning to the Idle state. This prevents various
         // error conditions from being flaged
         if(intrpt == INT_RX_FULL) {
            // make sure TX interrupts are turned off
            // throw away any data received
            pFlexIO_rx->read_data(); // get data from receiver and clear interrupt flag
            pFlexIO_rx->read_faults();
            break;
         }
         if(intrpt == INT_RX_EOR) {
            // EOR indicates that the BC has finished sending its message
            // Now it should be safe to go back to the idle state
            clearInterrupt(intrpt);
            pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);  // set receiver to watch for commands
            state = ST_IDLE;
            break;
         }
         // ignore any other interrupt
            logUnhandledInterrupt(intrpt);
            clearInterrupt(intrpt);
            break;

      case ST_TX_END:  // wait for EOT
         if(intrpt == INT_TX_EOT) {
            // EOT indicates that we finished sending a message
            // Now it should be safe to go back to the idle state
            clearInterrupt(intrpt);
            gFlexIO_tx->disableInterruptSource(FLEX1553_ALL_INTERRUPTS);
            pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);  // set receiver to watch for commands
            state = ST_IDLE;
            break;
         }
         // ignore any other interrupt
            logUnhandledInterrupt(intrpt);
            clearInterrupt(intrpt);
            break;

      default: // something went very wrong
         pFlexIO_rx->set_sync(FLEX1553_COMMAND_WORD);  // set receiver to watch for commands
         logUnhandledInterrupt(intrpt);
         clearInterrupt(intrpt);
         state = ST_IDLE;
   }
}



void MIL_1553_RT::logUnhandledInterrupt(intrpt_t intrpt)
{
   switch(intrpt)
   {
      case INT_RX_SYNC:
         errUnhandledIntrpt_rx_sync++;
         break;
      case INT_RX_FULL:
         errUnhandledIntrpt_rx_full++;
         break;
      case INT_RX_EOR:
         errUnhandledIntrpt_rx_eor++;
         break;
      case INT_TX_EMTY:
         errUnhandledIntrpt_tx_emty++;
         break;
      case INT_TX_EOT:
         errUnhandledIntrpt_tx_eot++;
         break;
      default:
         break;
   }
}


void MIL_1553_RT::clearInterrupt(intrpt_t intrpt)
{
   switch(intrpt)
   {
      case INT_RX_SYNC:
         pFlexIO_rx->clearInterrupt(FLEX1553RX_SYNC_FOUND_INTERRUPT);
         break;
      case INT_RX_FULL:
         pFlexIO_rx->clearInterrupt(FLEX1553RX_RECEIVER_FULL_INTERRUPT);
         break;
      case INT_RX_EOR:
         pFlexIO_rx->clearInterrupt(FLEX1553RX_END_OF_RECEIVE_INTERRUPT);
         break;
      case INT_TX_EMTY:
         gFlexIO_tx->clearInterrupt(FLEX1553TX_TRANSMITTER_EMPTY_INTERRUPT);
         break;
      case INT_TX_EOT:
         gFlexIO_tx->clearInterrupt(FLEX1553TX_END_OF_TRANSMIT_DELAYED_INTERRUPT);
         break;
      default:
         break;
   }
}



// This function sets up a packet to send or receive data on one RT subaddress.
// This can be used to setup a new mailbox, or to assign a different packet to an existing mailbox.
// Since an RT is a slave device, the transfer is initiated from the other end.
// in the RT mode, the subaddress is used as a index to a mailbox
// the subaddres field within the packet class is not used
// each subaddress may have one outgoing and one incoming mailbox
// add data transfer is handled by the interrupt routines
bool MIL_1553_RT::openMailbox(uint8_t subAddress, uint8_t wordCount, bool outgoing, MIL_1553_packet *packet, bool lock)
{
   if(subAddress > 31)
      return false;  // invalid subaddress

   if(wordCount > 32)
      return false;  // invalid subaddress

   if(packet == NULL)
      return false;  // must have a packet to assign to mailbox

   if(!beginOk)
      return false; // begin() has not been called, or has failed


   // set packet attributes
   packet->locking = lock;
   packet->newMail = false;
   packet->setRta(myRta);
   packet->setSubAddress(subAddress);
   packet->setWordCount(wordCount);

   // setup mailbox
   if(outgoing) {
      packet->setTrDir(TR_TRANSMIT);
      noInterrupts();
      mailboxOutPacket[subAddress] = packet;
      interrupts();
   }
   else { // incoming
      packet->setTrDir(TR_RECEIVE);
      noInterrupts();
      mailboxInPacket[subAddress] = packet;
      interrupts();
   }

   // make sure interrupts are enabled

   return true;
}


// get all information from packet
bool MIL_1553_RT::openMailbox(MIL_1553_packet *packet)
{
   if(packet == NULL)
      return false;  // must have a packet to assign to mailbox

   if(!beginOk)
      return false; // begin() has not been called, or has failed

   int sa = packet->getSubAddress();
   if(sa > 31)
      return false;

   // get packet RT Address and assign to array
   if(packet->getTrDir() == TR_TRANSMIT) {  // outgoing data
      noInterrupts();
      mailboxOutPacket[sa] = packet;
      interrupts();
   }
   else {  // incoming data
      noInterrupts();
      mailboxInPacket[sa] = packet;
      interrupts();
   }

   return true;
}


// This function disconnects the packet from the mailbox
bool MIL_1553_RT::closeMailbox(uint8_t subAddress, bool outgoing)
{
   if(subAddress > 31)
      return false;  // invalid subaddress

   // disable interrupts

   if(outgoing) {
      noInterrupts();
      mailboxOutPacket[subAddress] = NULL;
      interrupts();
   }
   else {
      noInterrupts();
      mailboxInPacket[subAddress] = NULL;
      interrupts();
   }
   return true;
}


// checks an incoming mailbox to see if there is a new message.
// if the mailbox is set for "locking", then a completed transfer
// will also lock the mailbox from any further transfers (returns
// a BUSY to the BC), until it is specifically unlocked.
// @param subAddress    mailbox address to check
bool MIL_1553_RT::mailAvailable(uint8_t subAddress)
{
   MIL_1553_packet *packet = NULL;

   if(subAddress > 31)
      return false;  // invalid subaddress

   // lookup packet assigned to this mailbox
   packet = mailboxInPacket[subAddress];

   if(packet == NULL)
      return false;  // no packet assigned

   return packet->newMail; //done;
}


// checks all mailboxes for new incoming mail
// checks incoming mailboxes only
MIL_1553_packet*  MIL_1553_RT::mailAvailable(void)
{
   for(int sa=0; sa<=31; sa++) {
      // check every assigned mailbox
      MIL_1553_packet* packet = mailboxInPacket[sa];

      if(packet != NULL) {
         if(packet->newMail)  // new mail found
            return packet;
      }
   }
   return NULL;  // no new mail found
}


// checks an outgoing mailbox to see if the mail has been
// picked up by the BC.
// if the mailbox is set for "locking", then a completed transfer
// will also lock the mailbox from any further transfers (returns
// a BUSY to the BC), until it is specifically unlocked.
bool MIL_1553_RT::mailSent(uint8_t subAddress)
{
   MIL_1553_packet *packet = NULL;

   if(subAddress > 31)
      return false;  // invalid subaddress

   // lookup packet assigned to this mailbox
   packet = mailboxOutPacket[subAddress];

   if(packet == NULL)
      return false;  // no packet assigned

   return !packet->newMail; // mail was picked up if newMail is low
}

// checks all outgoing mailboxes for "sent" data.
// a pointer will be retured to the first packet which has
// a clear newMail flag
// @return   pointer to the packet, or NULL if no empty mail boxes
MIL_1553_packet*  MIL_1553_RT::mailSent(void)
{
   for(int sa=0; sa<=31; sa++) {
      // check every assigned mailbox
      MIL_1553_packet* packet = mailboxOutPacket[sa];

      if(packet != NULL) {    // if no packet assigned, skip it
         if(!packet->newMail) // mail has been picked up
            return packet;
      }
   }
   return NULL;  // no new mail found
}


MIL_1553_packet*  MIL_1553_RT::getPacket(uint8_t subAddress, bool outgoing)
{
   if(subAddress > 31)
      return NULL;  // invalid subaddress

   if(outgoing) {
      return mailboxOutPacket[subAddress];
   }
   else {
      return mailboxInPacket[subAddress];
   }
}


bool MIL_1553_RT::errorAvailable(void)
{
   // in a good transaction, goodExitFlag will return true when
   // the statemachine goes back to the idle state. Otherwise, an
   // error occured

   if(state == ST_IDLE) {
      if(goodExitFlag)
         return false;  // no new errors
      else {
         goodExitFlag = true;  // set flag for next time
         return true;
      }
   }
   else
      return false;  // flag is not valid while state machine is running
}


// returns an error code for the last command sent or received
uint8_t MIL_1553_RT::getErrCode(void)
{
   // physical errors
   if(phyErrFlag)
      return phyErrFlag;
      // 1 = bit error detected on bus while attemting to capture command
      // 2 = parity error detected on captured command
      // 3 = bit error detected on bus while attemting to capture data
      // 4 = parity error detected on captured data

   // not for us
   if(nfuFlag)
      return 5;  // command RTA did not match our RTA

   // invalid SA
   if(msgErrFlag == 1)
      return 11;  // this SA does not exist!

   // no mailbox for SA
   if(pPacket == NULL)
      return 6;  // no mailbox assigned to this subaddress

   // word count does not match SA
   if(msgErrFlag == 2)
      return 7;  // command word count does not match the expected word count for this SA

   // word count does not match received data
   if(msgErrFlag == 3)
      return 8;  // data words received in packet does not match the command word count

   // mailbox busy
   if(pPacket != NULL && pPacket->isBusy())
      return 9;  // mailbox is busy

   switch(stateErr) {
      //case 0:
      //   break; // no error recorded

      case 2: // did not receive a valid command
      case 3: // unhandled interrupt
      case 4: // aborting RX status word due to error
      case 5: // aborting TX data due to invalid command
         return stateErr + 20;

      default:
         break;
   }

   return 10; // unknown error
}


void MIL_1553_RT::printErrReport(void)
{
   switch(getErrCode())
   {
      case 0:
         Serial.println("  No errors detected");
         Serial.print("  Bus: ");  Serial.print(myBus);
         Serial.print("  RTA: ");  Serial.print(msgRta);
         Serial.print("  SA: ");   Serial.print(msgSA);
         Serial.print("  WC: ");   Serial.print(msgWC);
         Serial.println();
         break;

      case 1:
         Serial.println("  Transition error detected while attemting to capture command");
         break;
      case 2:
         Serial.println("  Parity error detected on captured command");
         break;
      case 3:
         Serial.println("  Transition error detected while attemting to capture data");
         break;
      case 4:
         Serial.println("  Parity error detected on captured data word");
         break;

      case 5:
         Serial.print("  Message RTA (");
         Serial.print(msgRta);
         Serial.print(") did not match our RTA (");
         Serial.print(myRta);
         Serial.println(")");
         break;
      case 6:
         Serial.print("  No mailbox assigned to this subaddress (");
         Serial.print(msgSA);
         Serial.println(")");
         break;
      case 7:
         Serial.print("  Message word count (");
         Serial.print(msgWC);
         Serial.print(") does not match the expected word count (");
         if(pPacket)
            Serial.print(pPacket->getWordCount());
         else
            Serial.print("?");
         Serial.print(") for this SA (");
         Serial.print(msgSA);
         Serial.println(")");
         break;
      case 8:
         Serial.print("  Data words received in packet (");
         Serial.print(wordsReceivedOnRX);
         Serial.print(") does not match the command word count (");
         Serial.print(msgWC);
         Serial.println(")");
         break;
      case 9:
         Serial.print("  Mailbox ");
         Serial.print(msgSA);
         Serial.println(" is busy");
         break;
      case 11:
         Serial.print("  This subaddress (");
         Serial.print(msgSA);
         Serial.println(") is not assigned");
         break;
      case 22: //
         Serial.println("  Did not receive a valid command");
         break;
      case 23:
         Serial.println("  Unhandled interrupt");
         break;
      case 24:
         Serial.println("  Aborting status word due to error");
         break;
      case 25:
         Serial.println("  Aborting TX data due to invalid command");
         break;

      default:
         Serial.println("  Unknown error");
   }
}



// if a mailbox is setup as locking, once data has been transfered
// (i.e., the done flag goes true) the packet will be protected
// from further transactions until it is unlocked. This allows for
// the packet data to be safely read or modified.
bool MIL_1553_RT::unlock(uint8_t subAddress, bool outgoing)
{
   MIL_1553_packet *packet = NULL;

   if(subAddress > 31)
      return false;  // invalid subaddress

   // lookup packet assigned to this mailbox
   if(outgoing) {
      packet = mailboxOutPacket[subAddress];
   }
   else {
      packet = mailboxInPacket[subAddress];
   }

   if(packet == NULL)
      return false;  // no packet assigned

   packet->newMail = false; // clearing the flag unlocks transactions
   return true;
}


// If more than one 1553 bus is being, this sets which bus.
// this is global across all mailboxes.
// TX and RX buses would normally be the same, but dont have to be.
// This may be called at any time. Any mailbox that is in the
// middle of a transaction, will delay the bus change until it is done.
bool MIL_1553_RT::useBus(uint8_t busForTx, uint8_t busForRx)
{
   // error checking
   switch(busForTx) {
      case FLEX1553_BUS_A:
      case FLEX1553_BUS_B:
      case FLEX1553_BUS_ALL:
         break;
      default:
         return false;
   }
   switch(busForRx) {
      case FLEX1553_BUS_A:
      case FLEX1553_BUS_B:
      case FLEX1553_BUS_ALL:
         break;
      default:
         return false;
   }

   myBus = busForTx;
   //rxBus = busForRx;
   return true;
}


// this flag goes true if a CMD SYNC pattern is detected
// it goes low when calling this function
bool MIL_1553_RT::getLedRcvFlag(void)
{
   if(ledRcvFlag == false)
      return false;
   else {
      ledRcvFlag = false;
      return true;
   }
}


// this flag goes true if we send 1553 data back to the BC
// it goes low when calling this function
bool MIL_1553_RT::getLedTransFlag(void)
{
   if(ledTransFlag == false)
      return false;
   else {
      ledTransFlag = false;
      return true;
   }
}


void  MIL_1553_RT::dumpInternalState(void)
{
   Serial.print("  Bus: ");      Serial.print(myBus);
   Serial.print("  RTA: ");      Serial.print(myRta);
   Serial.print("  Words rcvd: ");  Serial.print(wordsReceivedOnRX);
   Serial.print("  Words sent: ");  Serial.print(wordsSentOnTX);
   Serial.println();

   Serial.print("  Msg: RTA=");  Serial.print(msgRta);
   Serial.print("  SA=");        Serial.print(msgSA);
   Serial.print("  WC=");        Serial.print(msgWC);
   Serial.print("  TRANS=");     Serial.print(msgTrans);
   Serial.println();

   Serial.print("  Flags: begin=");  Serial.print(beginOk);
   Serial.print("  NFU=");  Serial.print(nfuFlag);
   Serial.print("  busy=");   if(pPacket) Serial.print(pPacket->isBusy());
   Serial.println();

   Serial.print("  parityErr="); Serial.print(errParityCnt);
   Serial.print("  bitFaltErr=");  Serial.print(errBitCnt);
   Serial.print("  stateErr=");  Serial.print(stateErr);
   Serial.print("  msgErr=");    Serial.print(msgErrFlag);
   Serial.print("  phyErr=");    Serial.print(phyErrFlag);
   Serial.print("  ErrCode=");   Serial.print(getErrCode());
   Serial.println();

   Serial.print("  state=");     Serial.print(state);
   Serial.print("  NFUcount=");  Serial.print(nfuCount);
   Serial.print("  wordCount="); Serial.print(msgWC);
   Serial.print("  pPacket=");   Serial.print((int32_t)pPacket, HEX);
   Serial.print("  status=");   if(pPacket) Serial.print(pPacket->getStatusWord(), HEX);
   Serial.println();

   Serial.print("  Unhandled interrupts: ");
   Serial.print("  RX_SYNC=");   Serial.print(errUnhandledIntrpt_rx_sync);
   Serial.print("  RX_FULL=");   Serial.print(errUnhandledIntrpt_rx_full);
   Serial.print("  RX_EOR=");    Serial.print(errUnhandledIntrpt_rx_eor);
   Serial.print("  TX_EMTY=");   Serial.print(errUnhandledIntrpt_tx_emty);
   Serial.print("  TX_EOT=");    Serial.print(errUnhandledIntrpt_tx_eot);
   Serial.println();

   Serial.println();
}


void  MIL_1553_RT::dumpMailboxAssigments(void)
{
   Serial.println("  \tSA    \tIN     \tOUT ");
   for(int i=0; i<32; i++) {
      Serial.print("\t");   Serial.print(i);
      Serial.print("  \t"); Serial.print((int32_t)mailboxInPacket[i],HEX);
      Serial.print("  \t"); Serial.print((int32_t)mailboxOutPacket[i],HEX);
      Serial.println();
   }
}



// Dumps the packet content to the serial port
// This is just to verify the packet content
void MIL_1553_RT::dumpPacket(int sa)
{
   MIL_1553_packet *packet;
   // since we use the same packets for both TX and RX, there is not
   // reason to distinguish between the two

   switch(sa)
   {
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
         packet = mailboxOutPacket[sa];
         break;

      case 6:
         packet = mailboxInPacket[sa];
         break;

      default:
         return;
   }

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


bool MIL_1553_RT::setRta(uint8_t rta)
{
   if(rta <= MIL_1553_MAX_RTA) {
      myRta = rta;
      return true;
   }
   else
      return false;
}


uint8_t MIL_1553_RT::getRta(void)
{ return myRta; }


/////////////////////////////////////////////////////////////////////////
//                        MIL_1553_packet                              //


MIL_1553_packet::MIL_1553_packet(void)
{}


// set transmit or receive type
bool MIL_1553_packet::setTrDir(trDir_t tr)
{
   trDir = tr;
   return true;
}


trDir_t  MIL_1553_packet::getTrDir(void)
{ return trDir; }



// sets the packet content back to defaults
bool MIL_1553_packet::clear(void)
{
   trDir  = TR_UNDEFINED;
   rtAddress   = UNKNOWN;
   subAddress  = UNKNOWN;
   wordCount   = 0;
   statusWord  = 0;
   rxCount     = 0;
   parityErr   = false;
   bitFault    = false;
   newMail     = false;
   status = ST_NONE;

   for(int i=0; i<32; i++) {
      payload[i] = 0;
   }

   return true;
}


// write data to packet as a buffer
bool MIL_1553_packet::setData(uint16_t *data, uint8_t wc)
{
   //if(setWordCount(wc) == false)
   //   return false;  // abort if word count out of range

   if(wc > 32)  wc = 32;
   // Serial.print("  count=");
   // Serial.print(wc);
   // Serial.print("  data=");
   // Serial.println(data[0]);

   for(int i=0; i<wc; i++) {
      payload[i] = data[i];
   }
   //newMail = true;
   return true;
}


// write data to packet one word at a time, using 'index' to specify word
bool MIL_1553_packet::setWord(uint8_t index, uint16_t data)
{
   if(index >= 32) return false; // abort if word count out of range
   payload[index] = data;

   //Serial.print(index);
   //Serial.print(":");
   //Serial.print(data, HEX);
   //Serial.println();

   return true;
}


bool MIL_1553_packet::setString(const char* str, uint8_t offset)
{
   return setString(String(str), offset);
}


// packes a string, up to 64 charcters, into a 1553 packet (two bytes per word)
// strings will be truncated to fit within the previously set wordCount
// @param str      arduino String class
// @param offset   optional offset (in bytes) to reserve payload space before string
//                 must be an even number
bool MIL_1553_packet::setString(String str, uint8_t offset)
{
   int strIndex = 0;
   int allowedWC = 32;
   if(wordCount) allowedWC = wordCount; // if the packet wordCount has been configured, use it.
                                       // otherwise use 32 as a default. This allows initial data
                                       // to be set before the wordCount has been configured.
   uint16_t lowByte, highByte;
   int strLen = str.length() + offset; // total string length in bytes
   int wc = (strLen + 1) / 2;          // 16-bit words needed in packet
   if(wc > allowedWC) wc = allowedWC;  // truncate string to maximum configured wordCount
   offset = offset / 2;   // bytes to words (odd offset is not supported)

   for(int i=offset; i<allowedWC; i++) {
      if(i < wc) {   // copy string into buffer space between offset and wc
         highByte  = str[strIndex++];   // put two 8-bit characters into each 16-bit word
         if(strIndex < strLen)
            lowByte = str[strIndex++];
         else // if we have an odd number of characters in the string, set the last byte to 0
            lowByte = 0;
         //setWord(i, (highByte << 8) + lowByte);
         payload[i] = (highByte << 8) + lowByte;
      }
      else        // fill end of payload with zeros
         //setWord(i, 0);
         payload[i] = 0;
   }

   //wordCount = wc;
   //newMail = true;
   return true;
}


// if writing data one word at a time, the word count must be set here
bool MIL_1553_packet::setWordCount(uint8_t wc)
{
   if(wc > 32) return false;  // abort if word count out of range
   if(wc == 0) wc = 32;    // in the 1553 standard, WC=0 means 32 words

   wordCount = wc;
   return true;
}


bool MIL_1553_packet::setRta(uint8_t rta)
{
   if(rta <= MIL_1553_MAX_RTA) {
      rtAddress = rta;
      return true;
   }
   else
      return false;
}


bool MIL_1553_packet::setSubAddress(uint8_t sa)
{
   if(sa <= MIL_1553_MAX_SA) {
      subAddress = sa;
      return true;
   }
   else
      return false;
}



// data     pointer to array of 16-bit words
// size     number to 16-bit words in passed array
bool MIL_1553_packet::getData(uint16_t *data, uint8_t size)
{
   if(data == NULL) return false;
   if(size > wordCount) size = wordCount;

   for(int i=0; i<size; i++) {
      data[i] = payload[i];
   }
   newMail = false;
   return true;
}


uint16_t MIL_1553_packet::getData(uint8_t index)
{
   if(index >= wordCount) return 0;  // if index out of range, just return zero
   return payload[index];
}


// similar to getData() but returns a String class
// @param  offset to start of string within packet, in bytes.
String MIL_1553_packet::getString(uint8_t offset)
{
   //return String(&((char *)payload)[offset]); // this works, but bytes are in reverse order
   int stopByte = wordCount * 2;
   if(stopByte == 0) stopByte = 64;  // default incase wordCount has not been configured yet

   String str;
   char nextByte;
   for(int i=0; i<stopByte; i++) {
      if(i >= offset) {  // copy data only if we are between offset and stopByte
         if(isEven(i))   // if i is even then get the uppper half of the payload word
            nextByte = payload[i/2] >> 8;   // get upper byte
         else  // is odd
            nextByte = payload[i/2] & 0xff; // get lower byte
         str += nextByte;
         if(nextByte == 0)
            break;   // exit if NULL terminator found
      }
   }
   return str;
}


bool MIL_1553_packet::isEven(int val)
{
   return !(val % 2);
}


uint8_t MIL_1553_packet::getWordCount(void)
{
   if(wordCount == 0)
      return 32;
   else
      return wordCount;
}


uint8_t MIL_1553_packet::getRta(void)
{  return rtAddress; }


uint8_t MIL_1553_packet::getSubAddress(void)
{  return subAddress; }


// This builds the command word "on the fly" from other fields in the packet
// It is assumed that all of the necessary fields have already been loaded
uint16_t MIL_1553_packet::getCommandWord(void)
{
   int t_r = 0;
   if(trDir == TR_TRANSMIT)
      t_r = 1;

   uint16_t data = (rtAddress & 0x1f) << 11 | (t_r & 1) << 10 | (subAddress & 0x1f) << 5 | (wordCount & 0x1f);
   return(data);
}



bool MIL_1553_packet::validatePacket(void)
{
   if(trDir == TR_UNDEFINED)
      return false;
   if(rtAddress > 31)
      return false;
   if(subAddress > 31)
      return false;
   if(wordCount < 1 || wordCount > 32)
      return false;

   return true;
}


void MIL_1553_packet::setStatusWord(uint16_t status)
{
   statusWord = status;
   //rxCount++;
}

void MIL_1553_packet::setParityErr(bool parity)
{  parityErr = parity; }

void MIL_1553_packet::setBitFault(bool fault)
{  bitFault = fault; }

void MIL_1553_packet::setRxCount(uint16_t count)
{  rxCount = count; }

uint16_t MIL_1553_packet::getStatusWord(void)
{  return statusWord; }

bool MIL_1553_packet::getParityErr(void)
{  return parityErr; }

bool MIL_1553_packet::getBitFault(void)
{  return bitFault; }

uint16_t MIL_1553_packet::getRxCount(void)
{  return rxCount; }

bool MIL_1553_packet::isBusy(void)
{
   if(newMail && locking)
      return true;
   else
      return false;
}


