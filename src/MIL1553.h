// Flex1553
// A MIL-STD-1553 library for Teensy 4
// Copyright (c) 2022 Bill Sundahl
// MIT License

#pragma once

#include <Flex1553.h>

#define MIL_1553_MAX_RTA   30
#define MIL_1553_MAX_SA    31

#define MIL_1553_TR_RCV     0
#define MIL_1553_TR_TRANS   1
#define MIL_1553_RT_INCOMING  0
#define MIL_1553_RT_OUTGOING  1

#define MIL_1553_INT_SYNC     1
#define MIL_1553_INT_RX_FULL  6
#define MIL_1553_INT_RX_EOR   7
#define MIL_1553_INT_TX_EMTY  16
#define MIL_1553_INT_TX_EOT   17

//#define MIL_1553_RT_ST_IDLE            1
//#define MIL_1553_RT_ST_GET_CMD         2
//#define MIL_1553_RT_ST_RX_GET_DATA     4
//#define MIL_1553_RT_ST_RX_SEND_STS     5
//#define MIL_1553_RT_ST_TX_SEND_STS     8
//#define MIL_1553_RT_ST_TX_SEND_DATA    9
//#define MIL_1553_RT_ST_END             10


enum trDir_t {TR_UNDEFINED, TR_TRANSMIT, TR_RECEIVE};


// This class defines a 1553 packet.
// This is just a data set to be used by other classes.
class MIL_1553_packet
{
   public:
      #define UNKNOWN   0xff;

      MIL_1553_packet(void);

      enum packetStatus_t { ST_NONE, ST_OK, ST_PENDING, ST_ERROR, ST_TIMEOUT };

      bool clear(void);

      bool setTrDir(trDir_t tr);

      // write data to packet as a buffer
      bool setData(uint16_t *data, uint8_t wc);

      // write data to packet one word at a time, using 'index' to specify word
      bool setWord(uint8_t index, uint16_t data);

      // write a string to packet
      bool setString(String str, uint8_t offset = 0);
      bool setString(const char* str, uint8_t offset = 0);

      // if writing data one word at a time, the word count must be set here
      bool setWordCount(uint8_t wc);

      // if we are Bus Controller, this is the target RTA
      // if we are Remote Teminal, this is our RTA
      bool setRta(uint8_t rta);

      //
      bool setSubAddress(uint8_t sa);
      //bool settrDir(uint8_t type);  //transmit/receive
      trDir_t  getTrDir(void);

      uint16_t getData(uint8_t index);
      bool     getData(uint16_t *data, uint8_t size);
      //String   getData(void);
      String   getString(uint8_t offset = 0);

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
      bool     isBusy(void);


   public:
      //bool     done      = false; // transaction complete
      bool     locking   = false;
      bool     newMail   = false; // new mail that has not been sent yet or picked up yet

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

      bool isEven(int val);

};


// Bus Controller.
// This class supports the basic packet operations of a 1553 bus controller.
// It sets up the control word and handles the acknowledge
// This requires the use of one FlexIO_1553TX and one
// or two FlexIO_1553RX instances
// Though this class will manage a basic 1553 data transaction, it could
// still use a lot of work. Since I do not plan to use a bus controller in
// my project, I probably will not take this any farther.

class MIL_1553_BC
{
   public:
      #define RT_TO_BC  1
      #define BC_TO_RT  0
      #define TRANSMIT  RT_TO_BC
      #define RECEIVE   BC_TO_RT
      //#define BUS_A     0
      //#define BUS_B     1

      //MIL_1553_BC(uint8_t tx1_pin, uint8_t rx1_pin, uint8_t tx2_pin, uint8_t rx2_pin);
      MIL_1553_BC(FlexIO_1553TX *tx0, FlexIO_1553RX *rx0, FlexIO_1553RX *rx1 = NULL);
      //MIL_1553_BC(FlexIO_1553TX *tx0, FlexIO_1553RX *rx0);

      bool begin(void);
      bool send(MIL_1553_packet *packet, int8_t bus);
      bool request(MIL_1553_packet *packet, int8_t bus);

      // Number of data words that have been sent from current packet
      inline int  wordsSent(void) {return gWordsSent;}
      inline int  wordsReceived(void) {if(poTxBus == FLEX1553_BUS_A) return gWordsReceivedOnRX0; else return gWordsReceivedOnRX1;}
      inline int  getDebug(void) {return gDebug;}


   protected:
      //FlexIO_1553TX *poFlexIO_tx0 = NULL;     // pointers to physical layer classes
      //FlexIO_1553RX *poFlexIO_rx0 = NULL;
      //FlexIO_1553RX *poFlexIO_rx1 = NULL;
      //MIL_1553_packet *poTxPacket = NULL;     // pointer to active packet
      uint8_t poTxBus = FLEX1553_BUS_A;        // active bus channel

      static bool     beginOk;
      static bool     txActiveFlag;
      static uint8_t  gWordsSent;
      static uint8_t  gWordsReceivedOnRX0;
      static uint8_t  gWordsReceivedOnRX1;
      static uint8_t  gWordsToSend;
      static uint8_t  gWordsToGetRX0;
      static uint8_t  gWordsToGetRX1;
      static int gDebug;
      static FlexIO_1553TX  *gFlexIO_tx0;  // pointers with static linkage
      static FlexIO_1553RX  *gFlexIO_rx0;
      static FlexIO_1553RX  *gFlexIO_rx1;
      static MIL_1553_packet  *gPacket;
      static MIL_1553_packet  *gRxPacket;


   private:
      static void isrCallbackTx0(void);
      static void isrCallbackRx0(void);
      static void isrCallbackRx1(void);
};



class MIL_1553_RT
{
   public:
     // #define OUTGOING   1
     // #define INCOMING   0
     // #define BUS_A     1
     // #define BUS_B     2
     // #define BUS_BOTH  4

      int errUnhandledIntrpt_rx_sync = 0;
      int errUnhandledIntrpt_rx_full = 0;
      int errUnhandledIntrpt_rx_eor  = 0;
      int errUnhandledIntrpt_tx_emty = 0;
      int errUnhandledIntrpt_tx_eot  = 0;

      int errBitCnt = 0;    //   Counts the number of transition errors detected (for debug)
      int errParityCnt = 0;
      int nfuCount = 0;     // not really an error, just not for us


      MIL_1553_RT(FlexIO_1553TX *tx, FlexIO_1553RX *rx, int8_t bus = FLEX1553_BUS_A);
      bool begin(uint8_t rta);
      //bool openMailbox(uint8_t subAddress, bool outgoing, MIL_1553_packet *packet, bool lock = false);
      bool openMailbox(uint8_t subAddress, uint8_t wordCount, bool outgoing, MIL_1553_packet *packet, bool lock = false);
      bool openMailbox(MIL_1553_packet *packet);
      bool closeMailbox(uint8_t subAddress, bool outgoing);
      bool mailAvailable(uint8_t subAddress);
      MIL_1553_packet*  mailAvailable(void);  // checks all mailboxes for new incoming mail
      bool mailSent(uint8_t subAddress);
      MIL_1553_packet*  mailSent(void);
      MIL_1553_packet*  getPacket(uint8_t subAddress, bool outgoing);
      bool useBus(uint8_t busForTx, uint8_t busForRx);
      bool unlock(uint8_t subAddress, bool outgoing);
      void dumpInternalState(void);  // for debug
      void dumpMailboxAssigments(void); // for debug
      bool getLedRcvFlag(void);
      bool errorAvailable(void);
      uint8_t getErrCode(void);
      void printErrReport(void);
      void dumpPacket(int sa);


   protected:
      enum intrpt_t {INT_UNHANDLED_INT, INT_RX_SYNC, INT_RX_FULL, INT_RX_EOR, INT_TX_EMTY, INT_TX_EOT};
      enum state_t  {ST_IDLE, ST_GET_CMD, ST_RX_GET_DATA, ST_RX_SEND_STS, ST_TX_SEND_STS, ST_TX_SEND_DATA, ST_RX_END, ST_TX_END };
      bool     beginOk;
      uint8_t  phyErrFlag = 0;      // Invalid - this error will prevent any reaponse
      uint8_t  msgErrFlag = 0;      // Illegal - error bit returned by status word
      uint8_t  stateErr =   0;      // more error codes
      bool     nfuFlag = false;     // NotForUs - indicates RTA did not match
      state_t  state = ST_IDLE;
      uint8_t  myRta = 0;
      int8_t   myBus;
      uint8_t  wordsSentOnTX;
      uint8_t  wordsReceivedOnRX;
      //uint8_t  msgWordCount;
      bool     ledRcvFlag = false;

      // these are for debugging message problems
      bool     goodExitFlag = true;
      uint8_t  msgRta;
      uint8_t  msgSA;
      uint8_t  msgWC;
      bool     msgTrans;


      MIL_1553_packet *mailboxOutPacket[32];  // pointers to packets assigned to mailboxes, indexed by subaddress
      MIL_1553_packet *mailboxInPacket[32];
      MIL_1553_packet *pPacket = NULL;

      // there is only one TX instance, and it always needes to know which bus
      // is active. That is how the TX interrupt knows which MIL_1553_RT instance
      // to access. This is essentially a flag shared between the two instances.
      static int8_t activeTxBus;

      // these are pointers to this class, that can be called by static functions
      static MIL_1553_RT* gRTBusA;
      static MIL_1553_RT* gRTBusB;

      // these are pointers to the 1553 FlexIO modules
      FlexIO_1553RX *pFlexIO_rx;  // local non-static version
      static FlexIO_1553TX *gFlexIO_tx;  // static pointers for ISR
      static FlexIO_1553RX *gFlexIO_rxA;
      static FlexIO_1553RX *gFlexIO_rxB;

   private:
      static void isrCallbackRxA(void);  // used for bus A instance of MIL_1553_RT
      static void isrCallbackRxB(void);  // used for bus B instance
      static void isrCallbackTx(void);   // both instances must share the TX ISR
      void isrRtStateMachine(intrpt_t intrpt);
      void clearInterrupt(intrpt_t intrpt);
      void logUnhandledInterrupt(intrpt_t intrpt);


};


/* Bus Controller (BC)
   BC to RT
      begin()
         setup TX isr
         setup RX isr
         leave interrupts disabled

      load packet data
      send(packet)
         pass packet pointer to ISR
         initialize counters
         set packet direction bit to RECEIVE
         set SYNC type to STATUS
         enable TX and RX interrupts

      TX ISR
         check interrupt flags
         if(transmitter empty)
            if first word sent
               write(COMMAND word) to TX FlexIO
               set txActiveFlag flag
            else
               write(data word) from packet to TX FlexIO
            if no data left to send
               disable "transmitter empty" interrupt
         if(end of transmit)
            disable "end of transmit" interrupt
            clear txActiveFlag flag

      RX ISR
         check interrupt flags
         if "recevier full" interrupt
            read data and faults from RX FlexIO
            if(txActiveFlag == true)
               ignore data, this is just an echo from the transmitter
            if first word received
               save data in packet as STATUS
            else
               save data in packet as DATA
            if all data received
            check for parity and transition faults
         if "end of receive" interrupt
            turn off RX interrupts
         if "sync" interrupt
            change SYNC type to DATA

      wait for status word to be received
         on RX interrupt, capture STATUS word
         check status word for error bits
         or timeout if not recived
*/


/* STATUS word
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
      Busy Bit             normally 0, set to 1 for a locked mailbox
      Subsystem Flag Bit   not supported - set to 0
      Dynamic Bus Control  not supported - set to 0
      Terminal Flag Bit    not supported - set to 0
      Parity bit           calculated parity of status word


      Valid words:
         (a) The word begins with a valid sync field
         (b) The bits are in a valid Manchester II code
         (c) The information field has 16 bits plus parity
         (d) The word parity is odd
         (e) For a command, RTA must match the receiving RT, or be a broadcast 31
         A remote terminal shall not respond to an invalid command word
         Any invalid data word(s) or an error in the data word count, shall cause the remote
            terminal to set the message error bit in the status word suppress the transmission of the status word.

      Illegal command:
          valid command, but...
          unsupported conbination of subaddress, word count, RCV/TRANS bit
        If an RT detects an illegal command
          AND and the proper number of contiguous valid data words as specified by the illegal command word
          it shall respond with a status word only, setting the message error bit, and not use the information received
*/



/* Remote Terminal (RT)
   states
      IDLE           waiting for command
      GET_CMD        command sync received
      RX_GET_DATA    geting incoming data
      RX_SEND_STS    send status word
      TX_SEND_STS    sending status
      TX_SEND_DATA   sending data out
   flag
      nfuFlag        Not For Us - RTA did not match. Continue following transaction, but do not send or save anything
      phyErrFlag     Parity error, transition error, or other error which indicates data corruption
      msgErrFlag     Undefined subaddress, word count mismatch, or other error which indicates bad input
      pPacket->isBusy()  Indicates that the data is locked until packet is accessed on our end
      stateErr       This tracks error conditons thru the state machine for error reporting
      goodExitFlag   Indicates whether the state machine had a "good" exit

   Init, done in begin()
      set SYNC = command
      setup all 1553 interrupt callbacks to point to
      enable RX interrupts
*/

/*
States
   Idle
      This state is waiting for a command
      CMD_SYNC interrupt
         change sync to DATA_SYNC, in antisipation of a RECV command
         initialize transaction flags and counters
         state = GetCmd
      else
         any other interrupt will be assumed to be NFU bus traffic and ignored
         and stay in Idle state

   GetCmd
      We have detected a CMD_SYNC and are expecting a command word
      RX_FULL interrupt
         Capture command
         If physical errors
            log error
            ignore command and go to Idle state
               set sync back to CMD_SYNC
         else if RT address is not for us
            set nfuFlag
            continue to follow transaction, but take no action
         get subaddress, word count and T/R from command
         match this with a mailbox/packet class
         if RECV
            state = GetRxData
         else must be TRANS
            sync = CMD_SYNC   receiver is not used in TRANSmit, but I dont want to completely
                              disable it.  CMD_SYNC will minimize transmitter echo
            state = SendTxSts
         if mailbox does not exist, set msgErr flag
         if data word count does not match the expected count, set msgErr flag
      else
         any other interrupt should log a bus error
         state = Idle

   RX **********************
   GetRxData
      A RECV Command has been received, expecting data
      if RX_FULL interrupt
         Capture command
         If physical errors
            set phyErrFlag
            continue to follow transaction, but take no action
         if !busy & no errors
            copy data into buffer
            increment word pointer
      if EOR interrpt
         check for wrong word count
         generate status word
         if no problems, send status word
            set proper BUS
            enable TX EOT interrupt
            state = SendRxSts
            send status word
         else
            state = Idle
      if DATA_SYNC interrupt
         ignore, nothing to do
      else
         log unexpected interrupt

   SendRxSts
      Data has been received, now we are sending a status back
      if RX_FULL
         throw away TX echo
      if EOT interrupt
         disable transmitter interrupts
         if no errors
            set pPacket->newMail flag
         sync = CMD_SYNC
         state = Idle
      else
         log unexpected interrupt

   TX ************************
   SendTxSts
      A TRANS Command has been received, now we are sending a status back
      if EOR
         if physical error?
            ?? these should have been caught in GetCmd state
         check for command errors
         generate status word
         if no physical problems, send status word
            set proper BUS
            enable TX interrupts
            state = SendTxData
            send status word
         else
            if we report an error, we will send no data
            set sync back to CMD_SYNC
            state = Idle
      else
         log unexpected interrupt

   SendTxData
      Status has been sent, now we send the data
      if TX_EMTY
         send data from buffer
         incrment pointers
         if last word
            disable TX_EMTY interrupt
      if TX_EOT interrupt
         disable transmitter interrupts
         if no errors
            set pPacket->newMail flag
         sync = CMD_SYNC
         state = Idle
      if RX_FULL
         ignore transmitter echo
      else
         log unexpected interrupt

*/