#pragma once

#include <Flex1553.h>

#define MIL_1553_MAX_RTA   30
#define MIL_1553_MAX_SA    31


// This class defines a 1553 packet.
// This is just a data set to be used by other classes.
class MIL_1553_packet
{
   public:
      #define UNKNOWN   0xff;

      enum trDir_t   { TR_UNDEFINED, TR_TRANSMIT, TR_RECEIVE };
      enum packetStatus_t { ST_NONE, ST_OK, ST_PENDING, ST_ERROR, ST_TIMEOUT };

      bool clear(void);

      bool setTrDir(trDir_t tr);

      // write data to packet as a buffer
      bool setData(uint16_t *data, uint8_t wc);

      // write a string to packet
      bool setData(String *str);

      // write data to packet one word at a time, using 'index' to specify word
      bool setData(uint8_t index, uint16_t data);

      // if writing data one word at a time, the word count must be set here
      bool setWordCount(uint8_t wc);

      // if we are Bus Controller, this is the target RTA
      // if we are Remote Teminal, this is our RTA
      bool setRta(uint8_t rta);

      //
      bool setSubAddress(uint8_t sa);
      //bool settrDir(uint8_t type);  //transmit/receive

      uint16_t getData(uint8_t index);
      bool     getData(uint16_t *data, uint8_t size);
      String   getData(void);
      uint8_t  getWordCount(void);
      uint8_t  getRta(void);
      uint8_t  getSubAddress(void);
      uint16_t getCommandWord(void);
      bool     validatePacket(void);
      uint16_t getStatusWord(void);
      bool     getParityErr(void);
      bool     getBitFault(void);
      uint16_t getRxCount(void);
      void     setStatusWord(uint16_t status);
      void     setParityErr(bool parity);
      void     setBitFault(bool fault);
      void     setRxCount(uint16_t count);


   protected:
      trDir_t  trDir  = TR_UNDEFINED;
      uint8_t  rtAddress  = UNKNOWN;
      uint8_t  subAddress = UNKNOWN;
      uint8_t  wordCount  = 0;
      uint16_t statusWord = 0;
      uint16_t payload[32];
      uint8_t  rxCount    = 0; // number of words received
      bool     parityErr = false;
      bool     bitFault  = false;
      packetStatus_t status = ST_NONE;


};


// Bus Controller.
// This class supports the basic packet operations of a 1553 bus controller.
// It sets up the control word and handles the acknowledge
// This requires the use of one FlexIO_1553TX and one
// or two FlexIO_1553RX instances

class MIL_1553_BC
{
   public:
      #define RT_TO_BC  1
      #define BC_TO_RT  0
      #define TRANSMIT  RT_TO_BC
      #define RECEIVE   BC_TO_RT
      #define BUS_A     0
      #define BUS_B     1

      //MIL_1553_BC(uint8_t tx1_pin, uint8_t rx1_pin, uint8_t tx2_pin, uint8_t rx2_pin);
      MIL_1553_BC(FlexIO_1553TX *tx0, FlexIO_1553RX *rx0, FlexIO_1553RX *rx1 = NULL);
      //MIL_1553_BC(FlexIO_1553TX *tx0, FlexIO_1553RX *rx0);

      bool begin(void);
      bool send(MIL_1553_packet *packet, int8_t bus);
      bool request(MIL_1553_packet *packet, int8_t bus);

      // Number of data words that have been sent from current packet
      inline int  wordsSent(void) {return gWordsSent;}
      inline int  wordsReceived(void) {if(poTxBus == FLEX1553_CH_A) return gWordsReceivedOnRX0; else return gWordsReceivedOnRX1;}
      inline int  getDebug(void) {return gDebug;}


   protected:
      //FlexIO_1553TX *poFlexIO_tx0 = NULL;     // pointers to physical layer classes
      //FlexIO_1553RX *poFlexIO_rx0 = NULL;
      //FlexIO_1553RX *poFlexIO_rx1 = NULL;
      //MIL_1553_packet *poTxPacket = NULL;     // pointer to active packet
      uint8_t poTxBus = FLEX1553_CH_A;        // active bus channel

      static bool     beginOk;
      static bool     txActiveFlag;
      static uint8_t  gWordsSent;
      static uint8_t  gWordsReceivedOnRX0;
      static uint8_t  gWordsReceivedOnRX1;
      static uint8_t  gWordsToSend;
      static uint8_t  gWordsToGetRX0;
      static uint8_t  gWordsToGetRX1;
      static int gDebug;
      static FlexIO_1553TX  *gFlexIO_tx0;  // duplicate points with static linkage
      static FlexIO_1553RX  *gFlexIO_rx0;
      static FlexIO_1553RX  *gFlexIO_rx1;
      static MIL_1553_packet  *gTxPacket;
      static MIL_1553_packet  *gRxPacket;


   private:
      static void isrCallbackTx0(void);
      static void isrCallbackRx0(void);
      static void isrCallbackRx1(void);
};


/*
Bus Controller (BC)
   BC to RT
      isr
         setup TX isr
      send
         enable isr
         send RECEIVE COMMAND (with COMMAND SYNC)
         on TX interrupt, send DATA words (with DATA SYNC)
         on end of transmit, disable TX interrupt, enable RX
      wait for status word to be received
         on RX interrupt, capture STATUS word
         check status word for error bits
         or timeout if not recived

   RT to BC
      send
         send TRANSMIT COMMAND word (with DATA SYNC)
      wait for status word to be received
         watch for STATUS SYNC
         capture STATUS word
         check status word for error bits
         or timeout if not recived
      wait for data words to be received
         watch for DATA SYNC
         capture DATA words
         check for parity errors
         or timeout if not recived
      check for word count error


Remote Terminal (RT)
      wait for command word
         watch for COMMAND SYNC
         capture COMMAND word
         check command word for RTA
            if no match, ignore
         check command word for error bits
         check command word T/R bit
         check command word for number of data words
         check command word for subaddress

   BC to RT
      if T/R bit = RECEIVE
      wait for data words
         watch for DATA SYNC
         capture DATA words
         check for errors
         or timeout if not recived
      send STATUS
         if correct number or words received (else we dont know when to send the status)
         if any errors, set Message Error bit
         send STATUS word

   RT to BC
      if T/R bit = TRANSMIT
      send STATUS word (with STATUS SYNC)
      send data words (with DATA SYNC)





   STATUS word
      RTA                  verify that RTA is the expected address
      Message Error Bit    set if:
                              parity error
                              invalid Manchester II encoding
                              invalid word count
      Instrumentation Bit  set to 0 for STATUS words
                           set to 1 for COMMAND words?
      Service Request Bit  not supported - set to 0
      Reserved Bits        set to 0
      Broadcast Bit        not supported - set to 0
      Busy Bit             probably not needed - set to 0
      Subsystem Flag Bit   not supported - set to 0
      Dynamic Bus Control  not supported - set to 0
      Terminal Flag Bit    not supported - set to 0
      Parity bit           calculated parity of status word


Examples
   TX only
   RX only (sniffer)
   BC
   RT

*/
