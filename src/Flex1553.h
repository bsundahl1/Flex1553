#include <Arduino.h>
#include <FlexIOBase.h>

// 1553 experiment
#define FLEX1553_COMMAND_WORD   1
#define FLEX1553_DATA_WORD      0
#define FLEX1553_COMMAND_SYNC_PATTERN  0x8001ff00U   // upper 16 bits is the mask, lower 16 bits is the trigger pattern
#define FLEX1553_DATA_SYNC_PATTERN     0x800100ffU
#define FLEX1553_CH_A      1
#define FLEX1553_CH_B      2
#define FLEX1553_CH_ALL    4


typedef struct {
   bool   allowed;   // true = allow use of this pin pair
   bool   enabled;   // true = this pin pair is being used
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
      int8_t   m_chan;
      int8_t   m_altFlex;
      int8_t   m_altGpio;
      bool     m_just_configured;

      bool config_flex( void );
      bool config_io_pins( void );
      uint8_t parity( uint32_t data );

      uint8_t flexio_d_shift1_out;
      uint8_t flexio_d_shift2_out;
      uint8_t flexio_d_tim0_out;
      uint8_t flexio_d_tim2_out;
      uint8_t flexio_d_tim3_out;
      uint8_t flexio_d_tim5_out;
      uint8_t flexio_d_tim7_out;


   public:
      //   Class Constructor
      // due to the way that FlexIO uses pins in State Machine mode, these are the only
      // combinations of pins avaliable for TX (order is positive, negative):
      // Teensy 4.1
      //    FlexIO_1          FlexIO_2           FlexIO_3
      //     pair1: n/a        pair1: 10,12       pair1: 19,18
      //     pair2: n/a        pair2: 11,13       pair2: 14,15
      //     pair3: 2,3        pair3: n/a         pair3: 40,41
      //     pair4: 4,33       pair4: n/a         pair4: 17,16
      //
      // @param flex_num       [1 to 3]  specifies which of the on-chip FlexIO modules is to be used.
      // @param pairN          true  = output pair will be configured for use
      //                       false = pins will be avialble for other purposes
      FlexIO_1553TX(uint8_t flex_num, bool pair1 = true, bool pair2 = false, bool pair3 = false, bool pair4 = false);


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

      int send( uint8_t type, uint16_t data );

      int send_command( byte rtaddress, byte subaddress, byte wordcount );
      int send_status( uint8_t type, uint16_t data );
      int send_data( uint16_t data );

      // Controls the pin MUX to enable or disable the 1553 outputs.
      int set_channel( int ch );

      int transmitter_busy( void );

      unsigned long get_status( void );
};



/***************************************************************************************
*    Start of 1553 RX Class
***************************************************************************************/

class FlexIO_1553RX: public FlexIO_Base
{
   protected:
      int8_t   m_f_Pin;  // FXIO_Dxx signal
      int8_t   m_t_Pin;  // Teensy pin
      int8_t   m_altFlex;
      int8_t   m_altGpio;

      bool config_flex( void );
      bool config_io_pins( void );
      uint8_t parity( uint32_t data );
      //void isr1553Rx(void);


   public:
      FlexIO_1553RX(uint8_t flex_num, uint8_t rxPin);

      // Configure the FlexIO hardware and check for configuration errors
      bool begin( void );

      // Read RX data from FlexIO
      unsigned long read_data( void );

      // Read RX bit faults from FlexIO
      unsigned long read_faults( void );

      // Read FlexIO status registers
      unsigned long get_status( void );

      // Write the sync pattern to the FlexIO hardware
      void set_sync( uint8_t sync_type );

      // for debug
      int set_trigger( unsigned int pattern, unsigned int mask );

};


//int Flex1553TX_config(void);
//int Flex1553TX_send( uint8_t type, uint16_t data );
//int Flex1553TX_send_command( byte rtaddress, byte subaddress, byte wordcount );
//int Flex1553TX_send_status( uint8_t type, uint16_t data );
//int Flex1553TX_send_data( uint16_t data );
//int Flex1553TX_transmitter_busy( void );
//unsigned long Flex1553TX_get_status( void );
//int Flex1553TX_set_channel( int ch );

int Flex2_1553TX_config(void);
int Flex2_1553TX_send( uint8_t type, uint16_t data );
int Flex2_1553TX_transmitter_busy( void );
int Flex2_1553TX_get_status( void );

//int Flex3_1553RX_config(void);
//int Flex1553RX_trigger( unsigned int trigger, unsigned int pattern );

//unsigned long Flex3_1553RX_get_status( void );
//unsigned long Flex3_1553RX_read_data( void );
//unsigned long Flex3_1553RX_read_faults( void );

int Flex1_1553Sync_config1(void);
int Flex1_1553Sync_config2(void);
int Flex1_1553Sync_config3(void);
int Flex1_1553Sync_config4(void);
int Flex1_1553Sync_config5(void);
//int Flex3_1553Sync_config(void);
int Flex1_writeShifter3( unsigned long config );
uint32_t Flex1_readShifter3(void);



// There are 4 separate code modules here: two for TX and two for RX
//
// Transmit:
// TX can be implemented in FlexIO1 or FlexIO2 or both.
// Calling Flex1553TX_config() will configure FlexIO1, and
// Flex2_1553TX_config() will configure FlexIO2.
// These will operate completely independent of each other.
// Each has its own set of function calls.
// Flex1 is written so that it can use two sets of output pins
// (channel A and channel B). They both use the same transmitter
// but pretend to be two different channels. The intent here is
// to privide two transmittes and two receivers in three FlexIO
// modules. The receivers need oppreate simultaniously, but the
// transmitters do not. If this feature is not needed, comment
// out "#define FLEX01_TX_CHB" near the top of the Flex1553.c file.
// Flex1 Channel A will output on Teensy 4.1 pins 2 and 3
// Flex1 Channel B will output on Teensy 4.1 pins 4 and 33
// Flex2 will output on Teensy 4.1 pins 10 and 12
//
// Receiver:
// RX can be implemented in FlexIO2 or FlexIO3(if it exists) or both.
// Calling Flex2_1553RX_config() will configure FlexIO2, and
// Flex3_1553RX_config() will configure FlexIO3.
// These will operate completely independent of each other.
// Each has its own set of function calls.
// Flex2 will input on Teensy 4.1 pin ??TBD
// Flex3 will input on Teensy 4.1 pin 40
//


