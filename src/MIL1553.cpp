/* Packet level interface to MIL-STD-1553 transceiver.
   Lower level classes (Flex1553xx) are directly controlling FlexIO hardware,
   either a transmitter or receiver, but not both. A full 1553 transaction
   requires both sending (command or data) and receiving (data or status).
   This class brings it all together.
*/

#include <Arduino.h>
#include <MIL1553.h>
#include <Flex1553.h>


using namespace std;

// It seems that static variables declared inside a packet still have to
// be allocated outide the packet. Seems strange and redundant.
bool    MIL_1553_BC::beginOk = false;
bool    MIL_1553_BC::txActiveFlag = false;
uint8_t MIL_1553_BC::gWordsSent = 0;           // conter incremented by TX ISR to count TX words
uint8_t MIL_1553_BC::gWordsReceivedOnRX0 = 0;  // conter incremented by RX0 ISR to count RX0 words
uint8_t MIL_1553_BC::gWordsReceivedOnRX1 = 0;  // conter incremented by RX1 ISR to count RX1 words
uint8_t MIL_1553_BC::gWordsToSend = 0;         // used by TX ISR, tells TX when to stop
uint8_t MIL_1553_BC::gWordsToGetRX0 = 0;       // used by RX0 ISR, tells RX0 when to stop
uint8_t MIL_1553_BC::gWordsToGetRX1 = 0;       // used by RX1 ISR, tells RX1 when to stop
int     MIL_1553_BC::gDebug = 0;
FlexIO_1553TX   * MIL_1553_BC::gFlexIO_tx0 = NULL;  // allocate memory for static variables
FlexIO_1553RX   * MIL_1553_BC::gFlexIO_rx0 = NULL;
FlexIO_1553RX   * MIL_1553_BC::gFlexIO_rx1 = NULL;
MIL_1553_packet * MIL_1553_BC::gTxPacket   = NULL;
//MIL_1553_packet * MIL_1553_BC::gRxPacket   = NULL;
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
         gFlexIO_tx0->send(FLEX1553_COMMAND_WORD, gTxPacket->getCommandWord());
         gWordsSent++;
         txActiveFlag = true;
      }
      //else if(gWordsSent <= gTxPacket->getWordCount()) {
      else if(gWordsSent < gWordsToSend) {
         // if there is still data to send, send the next word
         gFlexIO_tx0->send(FLEX1553_DATA_WORD, gTxPacket->getData(gWordsSent-1));
         gWordsSent++;
      }
      else { // all data has been sent
         // turn off TX interrupt
         gFlexIO_tx0->disableInterruptSource(FLEXIO_SHIFTERS, 1);
         // enable RX to capture status word
         //gFlexIO_rx0->read_data(); // make sure receiver is empty
         //gFlexIO_rx0->enableInterruptSource(FLEXIO_SHIFTERS, 1); // RX data
      }
   }

   // check "end of transmit" flag
   if(gFlexIO_tx0->readInterruptFlag(FLEXIO_TIMERS, 6)) {
      // if the Timer2 flag is set, it indicates that the last data bit has been
      // sent from the transmitter, and the transmitter is tri-stated
      gFlexIO_tx0->disableInterruptSource(FLEXIO_TIMERS, 6);
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
 //     data   = FLEXIO2_SHIFTBUFBIS1;
 //     faults = FLEXIO2_SHIFTBUFBIS2;
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
         gTxPacket->setStatusWord((uint16_t)data);  // save data in packet statusWord
         gWordsReceivedOnRX0++;
      }
      else if(gWordsReceivedOnRX0 < gWordsToGetRX0) {
         digitalWrite(debugPin, 1);  // debug
         // all other words must be DATA words
         gTxPacket->setData(gWordsReceivedOnRX0 - 1, (uint16_t)data);
         gWordsReceivedOnRX0++;
         digitalWrite(debugPin, 0);  // debug
      }

      // turn off interrupt if all data received
      if(gWordsReceivedOnRX0 == gWordsToGetRX0) {
         digitalWrite(debugPin, 1);  // debug
         // turn off interrupt, this should be the end of the packet
         gFlexIO_rx0->disable();  // turn off RX interrupts
         digitalWrite(debugPin, 0);  // debug
      }

      // check for bit faults
      //if(gFlexIO_rx0->read_faults() != 0)
      if(faults != 0)
         gTxPacket->setBitFault(true);
      else
         gTxPacket->setBitFault(false);

      // check for parity error
      if(gFlexIO_rx0->parity(data) != parity_bit)
         gTxPacket->setParityErr(true);
      else
         gTxPacket->setParityErr(false);

      gTxPacket->setRxCount(gWordsReceivedOnRX0);
   }
   else { // found Sync pattern
      // This indicates a match in the sync trigger pattern.
      // This interrupt is unusual. Normally the interrupt is latched and is held
      // active until cleared. But "match continuous mode" of FlexIO does not seem
      // to latch the interrupt. I have not seen the interrupt missed, however if
      // you try to test for the interrupt flag, as in "if(flags & 0x08)", it seems
      // to be timing dependant as to whether or not it works.
      // Instead, test for all other enabled interrupts, and whatever is left
      // is assumed to be the SYNC interrupt.

 //  if(flags & 0x08) { // bit 3 is from the Shifter3 status flag
      //flex1553RX.clearInterrupt(FLEXIO_SHIFTERS, 1);  // not needed in Match Continuous mode
 //     if(gWordsReceivedOnRX0 == 1) { // first word received
      if(!txActiveFlag) {  // if TX is active, this is the COMMAND word. We are looking for STATUS
         // A valid packet will always receive a STATUS word first, and be followed by
         // zero or more DATA words. We only have the ability to watch for a single
         // sync pattern, so the pattern must initially be set to STATUS, and here we
         // change it to DATA.
         gFlexIO_rx0->disableInterruptSource(FLEXIO_SHIFTERS, 3);  // SYNC interrupt
         gFlexIO_rx0->set_sync(FLEX1553_DATA_WORD);
      }
   }

}


void MIL_1553_BC::isrCallbackRx1(void)
{
   uint32_t flags = gFlexIO_rx1->readInterruptFlags(FLEXIO_SHIFTERS);
   uint32_t data;
   uint8_t parity_bit;

   if(!beginOk)
      return;
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
   if(bus==FLEX1553_CH_B) {
      if(gFlexIO_rx1)
         pFlexIO_rx = gFlexIO_rx1; // use receiver 1
      else
         return false; // no pointer to channel B receiver
   }
   else
      pFlexIO_rx = gFlexIO_rx0; // use receiver 0

   // set up global variables for ISR
   gDebug = 0;
   gTxPacket  = packet;       // The TX interrupt routine will get data from here
   gWordsSent          = 0;   // reset word counters
   gWordsReceivedOnRX0 = 0;
   gWordsReceivedOnRX0 = 0;
   gWordsToSend   = packet->getWordCount() + 1; // send command + data
   gWordsToGetRX0 = 1;  // get status only
   gWordsToGetRX1 = 1;

   // 1553 defines data direction in terms of the RT (Remote Teminal)
   // Sending data to an RT is defined as a RECEIVE command
   packet->setTrDir(MIL_1553_packet::TR_RECEIVE);

   // setup
   gFlexIO_tx0->set_channel(bus);  // set bus channel A or B
   pFlexIO_rx->disable();  // make sure RX interrupts are off
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
   if(bus==FLEX1553_CH_B) {
      if(gFlexIO_rx1)
         pFlexIO_rx = gFlexIO_rx1; // use receiver 1
      else
         return false; // no pointer to channel B receiver
   }
   else
      pFlexIO_rx = gFlexIO_rx0; // use receiver 0

   // set up global variables for ISR
   gDebug = 0;
   gTxPacket  = packet;       // The RX interrupt routine will get data from here
   gWordsSent          = 0;   // reset word counters
   gWordsReceivedOnRX0 = 0;
   gWordsReceivedOnRX0 = 0;
   gWordsToSend   = 1;   // sending a command only
   gWordsToGetRX0 = packet->getWordCount() + 1; // receiving status + data words
   gWordsToGetRX1 = packet->getWordCount() + 1;

   // 1553 defines data direction in terms of the RT (Remote Teminal)
   // Reciving data from an RT is defined as a TRANSMIT command
   packet->setTrDir(MIL_1553_packet::TR_TRANSMIT);

   // setup
   gFlexIO_tx0->set_channel(bus);  // set bus channel A or B
   pFlexIO_rx->disable();  // make sure RX interrupts are off
   pFlexIO_rx->set_sync(FLEX1553_STATUS_WORD);
   pFlexIO_rx->flush();    // get ready to read status word

   // enable TX interupts from FlexIO Shifter1 and Timer2
   gFlexIO_tx0->enableInterruptSource(FLEXIO_SHIFTERS, 1); // transmitter empty interrupt
   gFlexIO_tx0->enableInterruptSource(FLEXIO_TIMERS, 6);   // last bit sent interrupt
   // The TX interrupt routine (isrCallbackTx0) will take it from here, sending a
   // new data word on each interrupt.

   // enable RX interupts
   pFlexIO_rx->enableInterruptSource(FLEXIO_SHIFTERS, 1);  // receiver full interrupt
   pFlexIO_rx->enableInterruptSource(FLEXIO_SHIFTERS, 3);  // SYNC interrupt

   return true;
}




/////////////////////////////////////////////////////////////////////////
//                        MIL_1553_packet                              //


// set transmit or receive type
bool MIL_1553_packet::setTrDir(trDir_t tr)
{
   trDir = tr;
   return true;
}


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
   status = ST_NONE;

   for(int i=0; i<32; i++) {
      payload[i] = 0;
   }

   return true;
}


// write data to packet as a buffer
bool MIL_1553_packet::setData(uint16_t *data, uint8_t wc)
{
   if(setWordCount(wc) == false)
      return false;  // abort if word count out of range

   for(int i=0; i<wc; i++) {
      payload[i] = data[i];
   }

   return true;
}


// packes a string, up to 64 charcters, into a 1553 packet (two bytes per word)
bool MIL_1553_packet::setData(String *str)
{
   int strIndex = 0;
   uint16_t lowByte, highByte;
   int strLen = str->length();   // characters in string
   if(strLen > 64) strLen = 64;
   int wc = (strLen + 1) / 2;    // 16-bit words needed in packet

   for(int i=0; i<wc; i++) {
      highByte  = (*str)[strIndex++];   // put two 8-bit characters into each 16-bit word
      if(strIndex < strLen)
         lowByte = (*str)[strIndex++];
      else // if we have an odd number of characters in the string, set the last byte to 0
         lowByte = 0;

      setData(i, (highByte << 8) + lowByte);
   }

   wordCount = wc;
   return true;
}


// write data to packet one word at a time, using 'index' to specify word
bool MIL_1553_packet::setData(uint8_t index, uint16_t data)
{
   if(index >= 32) return false; // abort if word count out of range
   payload[index] = data;

   //Serial.print(index);
   //Serial.print(":");
   //Serial.print(data, HEX);
   //Serial.println();

   return true;
}


// if writing data one word at a time, the word count must be set here
bool MIL_1553_packet::setWordCount(uint8_t wc)
{
   if(wc < 1)  return false;  // abort if word count out of range
   if(wc > 32) return false;

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
   return true;
}


String MIL_1553_packet::getData(void)
{
   return false;
}


uint16_t MIL_1553_packet::getData(uint8_t index)
{
   if(index >= wordCount) return 0;  // if index out of range, just return zero
   return payload[index];
}


uint8_t MIL_1553_packet::getWordCount(void)
{  return wordCount; }


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

