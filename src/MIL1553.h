#pragma once

#include <Flex1553.h>


class packet_1553
{
   public:
      #define UNKNOWN   0xff;

      //bool send(void);

      // write data to packet one word at a time, using 'index' to specify word
      bool setData(uint8_t index, uint16_t data);

      // write data to packet as a buffer
      bool setData(uint16_t *data, uint8_t wc);

      // write a string to packet
      bool setData(String *str);

      // if writing data one word at a time, the word count must be set here
      bool setWordCount(uint8_t wc);

      // if we are Bus Controller, this is the target RTA
      // if we are Remote Teminal, this is our RTA
      bool setRta(uint8_t rta);

      //
      bool setSubAddress(uint8_t sa);
      //bool setPacketType(uint8_t type);  //transmit/receive

      uint16_t getData(uint8_t index);
      bool getData(uint16_t *data, uint8_t size);
      String  getData(void);
      uint8_t getWordCount(void);
      uint8_t getRta(void);
      uint8_t getSubAddress(void);


   protected:
      //uint8_t type  = RECEIVE;
      uint8_t  rta = UNKNOWN;
      uint8_t  subAddress = UNKNOWN;
      uint8_t  wordCount = 1;
      uint16_t data[32];

};



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
      MIL_1553_BC(FlexIO_1553TX *tx0, FlexIO_1553RX *rx0, FlexIO_1553RX *rx1);

      bool begin(void);
      bool send(packet_1553 *packet, int8_t bus);

   protected:
      FlexIO_1553TX *flexIO_tx0 = NULL;
      FlexIO_1553RX *flexIO_rx0 = NULL;
      FlexIO_1553RX *flexIO_rx1 = NULL;
      uint8_t tx_bus = FLEX1553_CH_A;

};

