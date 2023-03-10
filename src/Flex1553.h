// Flex1553
// A MIL-STD-1553 library for Teensy 4
// Copyright (c) 2022 Bill Sundahl
// MIT License

#pragma once

#include <Arduino.h>
#include <FlexIOBase.h>

// All FlexIO configuration and access is done here.
// These classes perform the low level reads and writes to the 1553 bus.
// The SYNC and parity bit are setup in software, Manchester endoding,
// bit shifting and SYNC detection are done by FlexIO hardware.
// This class has no knowledge of the packet format or CONTROL/STATUS word
// content.


#define FLEX1553_STATUS_WORD    2
#define FLEX1553_COMMAND_WORD   1
#define FLEX1553_DATA_WORD      0

#define FLEX1553_COMMAND_SYNC_PATTERN  0x8001ff00U   // upper 16 bits is the mask, lower 16 bits is the trigger pattern
#define FLEX1553_DATA_SYNC_PATTERN     0x800100ffU
#define FLEX1553_PARITY_ERR_BIT     0x40000000
#define FLEX1553_FAULT_ERR_BIT      0x20000000

#define FLEX1553_BUS_A      1
#define FLEX1553_BUS_B      2
#define FLEX1553_BUS_ALL    4

#define FLEX1553_PINPAIR_1 0
#define FLEX1553_PINPAIR_2 1
#define FLEX1553_PINPAIR_3 2
#define FLEX1553_PINPAIR_4 3

#define FLEX1553TX_TRANSMITTER_EMPTY_INTERRUPT          1      // FLEXIO_SHIFTERS, 1
#define FLEX1553TX_END_OF_TRANSMIT_INTERRUPT            66     // FLEXIO_TIMERS, 2
#define FLEX1553TX_END_OF_TRANSMIT_DELAYED_INTERRUPT    70     // FLEXIO_TIMERS, 6
#define FLEX1553RX_RECEIVER_FULL_INTERRUPT              1      // FLEXIO_SHIFTERS, 1
//#define FLEX1553RX_SYNC_FOUND_INTERRUPT                 3      // FLEXIO_SHIFTERS, 3
#define FLEX1553RX_SYNC_FOUND_INTERRUPT                 71     // FLEXIO_TIMERS, 7
#define FLEX1553RX_END_OF_RECEIVE_INTERRUPT             66     // FLEXIO_TIMERS, 2
#define FLEX1553_ALL_INTERRUPTS                         255


typedef struct {
   bool   allowed;   // true = allow use of this pin pair
   //bool   enabled;   // true = this pin pair is being used
   int8_t f_posPin;  // flexIO pin number
   int8_t f_negPin;
   int8_t t_posPin;  // teensy pin number
   int8_t t_negPin;
}pair_t;



/***************************************************************************************
*    Start of 1553 TX Class
***************************************************************************************/

class FlexIO_1553TX: public FlexIO_Base
{
   protected:
      pair_t   m_pair[4];
      //uint32_t m_baud_div;
      int8_t   m_chan;  // active channel
      int8_t   m_chanToPinPair[3] = {-1, -1}; // 1=chan A, 2=chan B, pin pair=0 to 3
      int8_t   m_altFlex;
      int8_t   m_altGpio;
      bool     m_just_configured;

      uint8_t flexio_d_shift1_out;
      uint8_t flexio_d_shift2_out;
      uint8_t flexio_d_tim0_out;
      uint8_t flexio_d_tim2_out;
      uint8_t flexio_d_tim3_out;
      uint8_t flexio_d_tim5_out;
      uint8_t flexio_d_tim7_out;

      bool config_flex( void );
      bool config_io_pins( void );


   public:
      //   Class Constructor
      // Pins are brought out as differential pairs (order is positive, negative)
      // which are intended to drive transistors for transformer coupling.
      // Due to the way that FlexIO uses pins in State Machine mode, these are the
      // only combinations of pins avaliable for TX.
      // Teensy 4.1
      //    FlexIO_1          FlexIO_2           FlexIO_3           define
      //     pair1: n/a        pair1: 10,12       pair1: 19,18       FLEX1553_PINPAIR_1
      //     pair2: n/a        pair2: 11,13       pair2: 14,15       FLEX1553_PINPAIR_2
      //     pair3: 2,3        pair3: n/a         pair3: 40,41       FLEX1553_PINPAIR_3
      //     pair4: 4,33       pair4: n/a         pair4: 17,16       FLEX1553_PINPAIR_4
      //
      // @param flex_num       [1 to 3]  specifies which of the on-chip FlexIO modules is to be used.
      // @param pinPairA       [0 to 3]  pin pair to use as Channel A output
      // @param pinPairB       [0 to 3]  pin pair to use as Channel B output, or -1 for none
      //FlexIO_1553TX(uint8_t flex_num, bool pair1 = true, bool pair2 = false, bool pair3 = false, bool pair4 = false);
      FlexIO_1553TX(uint8_t flex_num, int8_t pinPairA, int8_t pinPairB = -1);


      // Configure the FlexIO hardware and check for configuration errors
      bool begin( void );

      // clear the FlexIO configuration
      void end( void );

      // this enables the flex module
      // in most cases, this should be overridden to resume operation from a disable()
      bool enable( void );

      // this disables the flex module
      // in most cases, this should be overridden to stop or pause operation of the flex circuit
      bool disable( void );

      // Use the blocking functions if TX interrupts are NOT being used
      // The function will not return until the word has been sent, or a timeout occurs
      int write_blocking( uint8_t sync, uint16_t data );
      int write_command_blocking( byte rtaddress, byte subaddress, byte wordcount, byte trDir );
      int write_status_blocking( uint8_t sync, uint16_t data );
      int write_data_blocking( uint16_t data );

      bool write( uint8_t sync, uint16_t data );

      uint8_t parity( uint32_t data );

      // Controls the pin MUX to enable or disable the 1553 outputs.
      int set_channel( int8_t ch );
      int8_t get_channel(void);

      int transmitter_busy( void );

      unsigned long get_status( void );
};



/***************************************************************************************
*    Start of 1553 RX Class
***************************************************************************************/

class FlexIO_1553RX: public FlexIO_Base
{
   protected:
      //int8_t   m_f_Pin;  // FXIO_Dxx signal
      int8_t   m_t_Pin;  // Teensy pin
      int8_t   m_altFlex;
      int8_t   m_altGpio;
      int8_t   m_rxDataRdPtr = 0;

      uint8_t flexio_d_data_in;
      uint8_t flexio_d_shift1_in;
      uint8_t flexio_d_shift2_in;
      uint8_t flexio_d_tim0_out;
      uint8_t flexio_d_tim1_out;
      uint8_t flexio_d_tim2_out;
      uint8_t flexio_d_tim4_out;
      uint8_t flexio_d_tim5_out;
      uint8_t flexio_d_tim7_out;

      bool config_flex( void );
      bool config_io_pins( void );

      //void isr1553Rx(void);
      //static void isrFlex3_1553Rx(void);


   public:

      //   Class Constructor
      // These are the pins which may be used for the data receive line
      // Teensy 4.1
      //    FlexIO_1          FlexIO_2           FlexIO_3
      //     2,3,4,5,33        6,9,11,13      14,15,16,17,20,21,22,23,40,41
      //
      // @param flex_num       [1 to 3]  specifies which of the on-chip FlexIO modules is to be used.
      // @param rxPin          one of the Teensy pin number from the list above
      FlexIO_1553RX(uint8_t flex_num, uint8_t rxPin);

      // Configure the FlexIO hardware and check for configuration errors
      bool begin( void );

      // disables the FlexIO interrupt, which effectively disables the receiver
      void disable(void);

      // enables the FlexIO interrupt and flushes buffer to get ready for new RX data
      void enable(void);

      // Clear buffer and get ready for new RX packet
      void flush(void);

      // Number of words left in the buffer to read
      uint8_t available(void);

      // Total Number of words which have been received into the buffer
      uint8_t word_count(void);

      uint8_t getSyncType(void);

      // Read one word from RX buffer
      int32_t read(void);

      uint8_t parity( uint32_t data );

      // Write the sync pattern to the FlexIO hardware
      void set_sync( uint8_t sync_type );

      // for debug only - may have side effects
      int set_trigger( unsigned int pattern, unsigned int mask );
      unsigned long read_data( void );   // Read directly from FlexIO shifter
      unsigned long read_faults( void ); // Read directly from FlexIO shifter
      unsigned long get_status( void );  // Read FlexIO status registers

};

