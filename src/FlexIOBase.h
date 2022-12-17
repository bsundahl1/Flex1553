// This is a simple base class to use with a single FlexIO module

// Also check out KurtE's FlexIO library. harder to understand, but more sophisticated
// https://github.com/KurtE/FlexIO_t4/tree/master/src/FlexIO_t4.cpp

// for a crash course in FlexIO, see miciwan's excellent 'getting started' writeup
// https://forum.pjrc.com/threads/66201-Teensy-4-1-How-to-start-using-FlexIO

#pragma once

#include <Arduino.h>

#define FLEXIO1   1
#define FLEXIO2   2
#define FLEXIO3   3

#define FLEXIO_SHIFTERS    0
#define FLEXIO_TIMERS      1
#define FLEXIO_PINS        2
#define FLEXIO_TRIGGERS    3

#define ALL_FLEXIO_INTERRUPTS  255

#define FLEXIO_REVERSE_LOOKUP    true
#define FLEXIO_TEENSY_PIN_TO_FLEXIO_D  true
#define FLEXIO_FLEXIO_D_TO_TEENSY_PIN  false

#define FLEXIO1_DEBUG_INT_PIN  32
#define FLEXIO2_DEBUG_INT_PIN  9
#define FLEXIO3_DEBUG_INT_PIN  39

/*
*  Valid pin numbers for FlexIO
*
*  Teensy4.1
*     FlexIO1     Pins: 2,3,4,5,33,49,50,52,54
*     FlexIO2     Pins: 6,7,8,9,10,11,12,13,32,33,35,36,37
*     FlexIO3     Pins: 7,8,14,15,16,17,18,19,20,21,22,23,26,27,34,35,36,37,38,39,40,41
*/
/*
*  Pin mapping -  FXIO_Dxx lines that wire to Teensy pins
*                 *******  Teensy 4.1  ********
*     FXIO_Dxx    FlexIO1    FlexIO2    FlexIO3
*         0           -         10         19
*         1           -         12         18
*         2           -         11         14
*         3           -         13         15
*         4           2          -         40
*         5           3          -         41
*         6           4          -         17
*         7          33          -         16
*         8           5          -         22
*         9           -          -         23
*         10          -          6         20
*         11          -          9         21
*         12         52         32         38
*         13         49          -         39
*         14         50          -         26
*         15         54          -         27
*         16                     8          8
*         17                     7          7
*         18                    36         36
*         19                    37         37
*         20                     -         10
*         21                     -         11
*         22                     -          -
*         23                     -          -
*         24                     -          -
*         25                     -          -
*         26                     -          -
*         27                     -          -
*         28                    35         35
*         29                    34         34
*         30                     -          -
*         31                     -          -
*/


class FlexIO_Base
{
   protected:
      IMXRT_FLEXIO_t *m_flex;    // pointer to the FlexIO hardware
      uint8_t  m_flex_num;       // FlexIO module being used [1 to 3]
      uint8_t  m_pll_divider;    // Sets divider for FLexIO Clock from 480MHz PLL (range: 4 to 64)


      //bool config_clock( void );  // configures PLL module
      //bool configFlex( void );   // configures FlexIO module

      // general utility to setup the pin mux
      // refer to table for avilable PIN and ALT values
      // https://github.com/KurtE/TeensyDocuments/blob/master/Teensy4.1%20Pins%20Mux.pdf
      bool setPinMux( uint8_t teensyPin, uint8_t alt);

      // simplified version that sets the pin mux to FlexIO
      bool setPinMux( uint8_t teensyPin );

      // get the internal Teensy pin number for specified FlexIO pin
      // @param pin:             pin number to look up
      // @param reverseLookup:   false = FlexIO_Dxx to Teensy pin
      //                         true  = Teensy pin to FlexIO_Dxx
      // returns -1 if error
      int8_t getTeensyPin( int8_t pin, bool reverseLookup = false );

      // Calculate the "best" dividers to use in the CCM_CDCDR register
      // Note that flex2 and flex3 share the same clock
      // @param divider    desired total divider (range: 4 to 64)
      // @param p_prediv   pointer to calculated pre-divider (range: 1 to 8)
      // @param p_postdiv  pointer to calculated post-divider (range: 1 to 8)
      // @param retrun     final divider ratio
      int calc_pll_clock_div( uint8_t divider, uint8_t *p_prediv, uint8_t *p_postdiv );

      // returns the interrupt number for the flexIO module being used (m_flex_num)
      int get_irq_num(void);


  public:


     /**
      * Class Constructor -
      * Instantiate a new instance of the class.
      *
      * @param flex_num       [1 to 3]  specifies which of the on-chip FlexIO modules is to be used.
      * @param flex_clock [120 to 7.5]  specifies the FlexIO clock frequency in MHz. default is 30 MHz.
      *                                 This is derived from a 480MHz PLL, and divided thru two dividers,
      *                                 each with a 1 - 8 divider range. If an exact frequency can not
      *                                 be achieved, the closest possible will be used.
      */
      //FlexIO_Base(uint8_t flex_num, uint8_t m_pll_divider = 16);
      FlexIO_Base(uint8_t flex_num, float flex_clock_freq = 30);

      //static void isrFlex1_1553(void);
      //static void isrFlex2_1553(void);
      //static void isrFlex3_1553(void);

      // configure the FlexIO hardware and check for configuration errors
      bool begin( void );

      // clear the FlexIO configuration
      void end( void );

      // this enables the flex module
      // in most cases, this should be overridden to resume operation from a disable()
      bool enable( void );

      // this disables the flex module
      // in most cases, this should be overridden to stop or pause operation of the flex circuit
      bool disable( void );

      // check if the flex clock is running
      bool clock_running( void );

      // get the PARAM register from the FLEXIO module
      // @param   param_type:  0=SHIFTERs, 1=TIMERs, 2=PINs, 3=TRIGGERs
      uint8_t get_params(uint8_t param_type);

      // returns the PIN register, which is the current state of all the FlexIO pins FXIO_Dxx
      uint32_t get_pin_states(void);

      // returns the clock dividers used to generate the flex clock from PLL3 (480MHz)
      // used for debugging clock issues
      // @param target_div  target divider, calculated in the FlexIO_Base constructor
      //                        from flex_clock frequency
      // @param pre_div     pre-divider used in the CCM clock module
      // @param post_div    post-divider used in the CCM clock module
      void get_clock_divider( uint8_t *target_div, uint8_t *pre_div, uint8_t *post_div );

      // Set the flex clock dividers
      // Use this ONLY if you need to override the divider ratios calculated from flex_clock_freq
      // Note that flex2 and flex3 share the same clock
      // Default Flex clock is 30MHz
      // @param p_prediv   pre-divider (range: 1 to 8)
      // @param p_postdiv  post-divider (range: 1 to 8)
      // @param retrun     true if no errors
      bool config_clock_div( uint8_t prediv, uint8_t postdiv );

      void attachInterrupt(void (*isr)(void));
      void attachInterrupt(void (*isr)(void), uint8_t prio);
      void detachInterrupt(void);
      void clearInterrupt(uint8_t source, uint8_t flag_num);
      void clearInterrupt(uint8_t key);
      void enableInterruptSource(uint8_t source, uint8_t flag);
      void enableInterruptSource(uint8_t key);
      void disableInterruptSource(uint8_t source, uint8_t flag_num);
      void disableInterruptSource(uint8_t key);
      uint32_t readInterruptFlags(uint8_t source);
      bool readInterruptFlag(uint8_t source, uint8_t flag_num);
      bool readInterruptFlag(uint8_t key);


      bool attachInterruptCallback(void (*callback)(void));
      void detachInterruptCallback(void);
      //void enableCallback(void);
      //void disableCallback(void);


};

