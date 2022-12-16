// Flex1553
// A MIL-STD-1553 library for Teensy 4
// Copyright (c) 2022 Bill Sundahl
// MIT License

#include <Arduino.h>
#include <Flex1553.h>

// This class implements a 1553 receiver in one FlexIO module of the
// NXP i.MXRT1062 processor. This is a physical layer only, it does not
// know the meaning of a packet, or any of the control bits other than
// parity. There is no synchronization with the transmit module.
// For information on FlexIO, refer to the NXP reference manual (IMXRT1060RM)
// Chapter 50.


// 7/12/21  change to a 48MHz clock so we can derive 6MHz from timers
// 9/10/21  reconfigured code to use C++ class instead of standard C
// 9/20/21  created base class for reusable FlexIO functions
// 10/30/21 changed clock speed back to 40MHz
// 1/02/22  Added changes for "Match Continuous mode", RX working
// 2/13/22  Added interrupt routine to capture RX data
// 2/20/22  Changed 5MHz source from Timer2 to Timer5
// 7/10/22  Configure Timer2 to trigger EOR interrupt


// Bug list
// finish enable() and disable()
// use DMA controller to store data from FlexIO receiver


#define REVERSE_LOOKUP   true

// Optional build flags:
#define FLEX_PRINT_MESSAGES // uncomment to print startup messages to serial port.
                              // note that to see these messages, you must wait for the serial
                              // port to connect in setup() :
                              //    Serial.begin(115200);
                              //    while(!Serial);

//#define FLEX_DEBUG          // uncomment to bring out debug pins.
                              // You will normally want to turn this off so that you can use
                              // these IO pins for other purposes.

#ifndef __IMXRT1062__
   #error "This code requires the use of FlexIO hardware, found on the i.MXRT 106x processors"
#endif

/***************************************************************************************
*    Start of 1553 RX Class
***************************************************************************************/

// FXIO_Dxx signals used by this module
// The following FXIO signals can not be changed without consequences.
// These are hard coded into the state machine. They are the same for all FlexIO modules.
#define FLEX_1553RX_PIN_SHFT1_IN   0    // data bits from state machine   (not accessable on teensy pin)
#define FLEX_1553RX_PIN_SHFT2_IN   1    // fault bits from state machine  (not accessable on teensy pin)

// The following FXIO signals may be changed as needed
// For debug, the recomended input pin is in the range of FXIO_[2:5] to avoid conflicts
// Notes: FXIO_D[1:0] are reserved for state machine outputs
//        State Machine (SM) inputs include 3 signals:
//          the specified input FXIO_Dx (which maps to rxPin)
//          the 1MHz clock FXIO_Dx + 1
//          unused but required input FXIO_Dx + 2
//        These three SM inputs move with rxPin, as a set, in order by FXIO_D number.
//        1553RX_PIN_TIM4_OUT & 1553RX_PIN_TIM5_OUT are always enabled and must
//          not conflict with SM input or output signals.
//        The other three signals listed here must not conflict IF FLEX_DEBUG is defined
//          otherwise, these signals are disabled.
#define FLEX1_1553RX_PIN_TIM0_OUT  12    // 2 MHz state machine clock - for debug only
#define FLEX1_1553RX_PIN_TIM2_OUT  13    // End of Transmission  - for debug only
#define FLEX1_1553RX_PIN_TIM4_OUT  14    // Reset pulse to Timer3
#define FLEX1_1553RX_PIN_TIM5_OUT  15    // 5 MHz clock
#define FLEX1_1553RX_PIN_TIM7_OUT   8    // SYNC pulse - for debug only

#define FLEX2_1553RX_PIN_TIM0_OUT  10    // 2 MHz state machine clock
#define FLEX2_1553RX_PIN_TIM2_OUT  11    // End of Transmission
#define FLEX2_1553RX_PIN_TIM4_OUT  14    // Reset pulse to Timer3
#define FLEX2_1553RX_PIN_TIM5_OUT  15    // 5 MHz clock
#define FLEX2_1553RX_PIN_TIM7_OUT  17    // SYNC pulse - for debug only

#define FLEX3_1553RX_PIN_TIM0_OUT  10    // 2 MHz state machine clock
#define FLEX3_1553RX_PIN_TIM2_OUT  11    // End of Transmission
#define FLEX3_1553RX_PIN_TIM4_OUT  14    // Reset pulse to Timer3
#define FLEX3_1553RX_PIN_TIM5_OUT  15    // 5 MHz clock
#define FLEX3_1553RX_PIN_TIM7_OUT   9    // SYNC pulse - for debug only

#ifdef DEBUG
   #define PIN_ENA  3   // enable timer output enable only if DEBUG flag is set
#else
   #define PIN_ENA  0
#endif


////////////////////////////////////////////////////////////////
/////////////// Start of Interrupt Routines ////////////////////
////////////////////////////////////////////////////////////////

// It turns out that an interrupt rountine can not be imbedded inside a C++ class.
// There are techniques that allow an interrupt routine to access the variables
// within a class, from outside the class, however they are incredibly complex
// and difficult to undestand.
// In this application, there are only 3 possible interrupts (one for each FlexIO module)
// and only one of those can be used by an instance of the class. And the routines are
// relatively simple. So below, you will find three copies of the ISR, each customized
// for its specific FlexIO module. All variables required by the ISR are also defined
// outside the class, and there are three copies of each variable. This allows for up to
// three instances of the class (one for each FlexIO module), each using its own ISR and
// own set of static variables.

static volatile uint8_t  g_syncType[4] = {FLEX1553_COMMAND_WORD, FLEX1553_COMMAND_WORD, FLEX1553_COMMAND_WORD, FLEX1553_COMMAND_WORD};
static volatile uint8_t  g_rxDataCount[4] = {0};  // keep track of how many words are in the buffer
static volatile uint32_t g_rxData[4][33];  // RX data buffer, 33 words (including command) is the longest valid packet
static volatile uint32_t g_rxFault[4][33];

// these interrupt routines are defaults, which will move captured receive
// data to a memory buffer. data is read from the buffer with the read() function.
// these ISR's can be overridden with attachInterrupt() for use with a higher
// code layer.
static void isrFlex1_1553Rx(void)
{
   int flags =  FLEXIO1_SHIFTSTAT; // read FlexIO shifter flags

   // found Sync pattern
   if(flags & 0x08) { // bit 3 is from the Shifter3 status flag
      // This indicates a match in the sync trigger pattern

      //flex1553RX.clearInterrupt(FLEXIO_SHIFTERS, 1);  // not needed in Match Continuous mode?
      if(g_syncType[1] == FLEX1553_COMMAND_WORD) { // change to data sync
         // A valid packet will always start with a COMMAND word, and be followed by
         // one or more DATA words. We only have the ability to watch for a single
         // sync pattern, so the pattern must initially be set to COMMAND, and here we
         // change it to DATA.
         FLEXIO1_SHIFTBUFBIS3 = FLEX1553_DATA_SYNC_PATTERN;
         g_syncType[1] = FLEX1553_DATA_WORD;
      }
      //Serial.println("found sync");
   }

   // received one word
   if(flags & 0x02) { // bit 1 is from the Shifter1 status flag
      // This indicates that a new word has been captured by the receiver

      // save the data and fault shifters into a buffer
      uint8_t offset = g_rxDataCount[1];
      if(offset < 32) {
         g_rxFault[1][offset] = FLEXIO1_SHIFTBUFBIS2;
         g_rxData[1][offset]  = FLEXIO1_SHIFTBUFBIS1; // reading the shifter also clears the interrupt flag
      }
      g_rxDataCount[1]++;

      //Serial.print("received word: ");
      //Serial.println(rx_data >> 1, HEX);
   }
}

static void isrFlex2_1553Rx(void)
{
   int flags =  FLEXIO2_SHIFTSTAT; // read FlexIO shifter flags

   // found Sync pattern
   if(flags & 0x08) { // bit 3 is from the Shifter3 status flag
      // This indicates a match in the sync trigger pattern

      //flex1553RX.clearInterrupt(FLEXIO_SHIFTERS, 2);  // not needed in Match Continuous mode?
      if(g_syncType[2] == FLEX1553_COMMAND_WORD) { // change to data sync
         // A valid packet will always start with a COMMAND word, and be followed by
         // one or more DATA words. We only have the ability to watch for a single
         // sync pattern, so the pattern must initially be set to COMMAND, and here we
         // change it to DATA.
         FLEXIO2_SHIFTBUFBIS3 = FLEX1553_DATA_SYNC_PATTERN;
         g_syncType[2] = FLEX1553_DATA_WORD;
      }
      //Serial.println("found sync");
   }

   // received one word
   if(flags & 0x02) { // bit 1 is from the Shifter1 status flag
      // This indicates that a new word has been captured by the receiver

      // save the data and fault shifters into a buffer
      uint8_t offset = g_rxDataCount[2];
      if(offset < 32) {
         g_rxFault[2][offset] = FLEXIO2_SHIFTBUFBIS2;
         g_rxData[2][offset]  = FLEXIO2_SHIFTBUFBIS1; // reading the shifter also clears the interrupt flag
      }
      g_rxDataCount[2]++;

      //Serial.print("received word: ");
      //Serial.println(rx_data >> 1, HEX);
   }
}

static void isrFlex3_1553Rx(void)
{
   int flags =  FLEXIO3_SHIFTSTAT; // read FlexIO shifter flags

   // found Sync pattern
   if(flags & 0x08) { // bit 3 is from the Shifter3 status flag
      // This indicates a match in the sync trigger pattern

      //flex1553RX.clearInterrupt(FLEXIO_SHIFTERS, 3);  // not needed in Match Continuous mode?
      if(g_syncType[3] == FLEX1553_COMMAND_WORD) { // change to data sync
         // A valid packet will always start with a COMMAND word, and be followed by
         // one or more DATA words. We only have the ability to watch for a single
         // sync pattern, so the pattern must initially be set to COMMAND, and here we
         // change it to DATA.
         FLEXIO3_SHIFTBUFBIS3 = FLEX1553_DATA_SYNC_PATTERN;
         g_syncType[3] = FLEX1553_DATA_WORD;
      }
      //Serial.println("found sync");
   }

   // received one word
   if(flags & 0x02) { // bit 1 is from the Shifter1 status flag
      // This indicates that a new word has been captured by the receiver

      // save the data and fault shifters into a buffer
      uint8_t offset = g_rxDataCount[3];
      if(offset < 32) {
         g_rxFault[3][offset] = FLEXIO3_SHIFTBUFBIS2;
         g_rxData[3][offset]  = FLEXIO3_SHIFTBUFBIS1; // reading the shifter also clears the interrupt flag
      }
      g_rxDataCount[3]++;

      //Serial.print("received word: ");
      //Serial.println(rx_data >> 1, HEX);
   }
}



////////////////////////////////////////////////////////////////
////////////////// Start of 1553 RX Class //////////////////////
////////////////////////////////////////////////////////////////

FlexIO_1553RX::FlexIO_1553RX(uint8_t flex_num, uint8_t rxPin)
   :FlexIO_Base(flex_num, 40.0)
{
   m_t_Pin    = rxPin;
   //m_f_Pin    = getTeensyPin(rxPin, REVERSE_LOOKUP);
   flexio_d_data_in  = getTeensyPin(rxPin, REVERSE_LOOKUP);  // first input to state machine
   flexio_d_tim1_out = flexio_d_data_in + 1;                 // second input to state machine
   m_altFlex  = (m_flex_num == 3)? 9 : 4; // FlexIO3 uses ALT9, FLEXIO1 & 2 use ALT4
   m_altGpio  = 5;  // gpio always uses Alt5

   // assign FlexIO_Dxx lines based on which FlexIO is used
   // most of these are just used for debug, so it is important
   // that they come out to usable Teensy pins.
   switch(m_flex_num) {
      case FLEXIO1:
         flexio_d_tim0_out   = FLEX1_1553RX_PIN_TIM0_OUT;
         flexio_d_tim2_out   = FLEX1_1553RX_PIN_TIM2_OUT;
         flexio_d_tim4_out   = FLEX1_1553RX_PIN_TIM4_OUT;
         flexio_d_tim5_out   = FLEX1_1553RX_PIN_TIM5_OUT;
         flexio_d_tim7_out   = FLEX1_1553RX_PIN_TIM7_OUT;
         break;
      case FLEXIO2:
         flexio_d_tim0_out   = FLEX2_1553RX_PIN_TIM0_OUT;
         flexio_d_tim2_out   = FLEX2_1553RX_PIN_TIM2_OUT;
         flexio_d_tim4_out   = FLEX2_1553RX_PIN_TIM4_OUT;
         flexio_d_tim5_out   = FLEX2_1553RX_PIN_TIM5_OUT;
         flexio_d_tim7_out   = FLEX2_1553RX_PIN_TIM7_OUT;
         break;
      case FLEXIO3:
         flexio_d_tim0_out   = FLEX3_1553RX_PIN_TIM0_OUT;
         flexio_d_tim2_out   = FLEX3_1553RX_PIN_TIM2_OUT;
         flexio_d_tim4_out   = FLEX3_1553RX_PIN_TIM4_OUT;
         flexio_d_tim5_out   = FLEX3_1553RX_PIN_TIM5_OUT;
         flexio_d_tim7_out   = FLEX3_1553RX_PIN_TIM7_OUT;
         break;
      default:
         m_flex_num = 0; // invalid value
   }
}



bool FlexIO_1553RX::begin( void )
{
   if(m_flex_num == 0) {
      #ifdef FLEX_PRINT_MESSAGES
         Serial.println( "Error: invalid flexIO module number" );
      #endif
      return false;
   }

   #ifdef FLEX_PRINT_MESSAGES
      Serial.print( "Configuring 1553RX on FlexIO" );
      Serial.print( m_flex_num );
      Serial.println();
   #endif

   if(flexio_d_data_in < 2 || flexio_d_data_in > 11) {
      // The Teensy pin number passed to the constructor (rxPin) is expected to
      // translate to a FlexIO data line in the range of FXIO[11:2].
      #ifdef FLEX_PRINT_MESSAGES
         Serial.println( "Error: invalid pin number used for rxPin" );
      #endif
      return false;
   }

   if( FlexIO_Base::begin() == false ) { // configures pll divider
      #ifdef FLEX_PRINT_MESSAGES
         Serial.println( "Error: FlexIO_Base::begin() failed" );
      #endif
      return false;
   }

   if( config_flex() ==  false ) {
      #ifdef FLEX_PRINT_MESSAGES
         Serial.println( "Error: config_flex() failed" );
      #endif
      return false;
   }

   if( config_io_pins() == false ) {
      #ifdef FLEX_PRINT_MESSAGES
         Serial.println( "Error: config_io_pins() failed" );
      #endif
      return false;
   }

   // Enable RX interrupt routine
   switch(m_flex_num) {
      case FLEXIO1:
         //FlexIO_Base::attachInterrupt(isrFlex1_1553Rx);
         FlexIO_Base::attachInterruptCallback(isrFlex1_1553Rx);
         break;
      case FLEXIO2:
         //FlexIO_Base::attachInterrupt(isrFlex2_1553Rx);
         FlexIO_Base::attachInterruptCallback(isrFlex2_1553Rx);
         //FlexIO_Base::enableCallback();
         break;
      case FLEXIO3:
         //FlexIO_Base::attachInterrupt(isrFlex3_1553Rx);
         FlexIO_Base::attachInterruptCallback(isrFlex3_1553Rx);
         break;
   }
   FlexIO_Base::enableInterruptSource(FLEXIO_SHIFTERS, 1);
   FlexIO_Base::enableInterruptSource(FLEXIO_SHIFTERS, 3);


   #ifdef FLEX_PRINT_MESSAGES  // print out pins used
      // Receiver Input pin
      Serial.print( "  RX_IN    = FXIO_D" );
      Serial.print( flexio_d_data_in );
      Serial.print( " = Teensy pin " );
      Serial.print( m_t_Pin );
      Serial.println();

      // print out the internal FXIO connections used
      int pin = -1;
      for( int i=0; i<8; i++ ) {
         switch(i) {
            case 0:
               Serial.print( "  SHFT1_IN  = FXIO_D" );
               pin = FLEX_1553RX_PIN_SHFT1_IN;
               break;
            case 1:
               Serial.print( "  SHFT2_IN  = FXIO_D" );
               pin = FLEX_1553RX_PIN_SHFT2_IN;
               break;
            case 2:
               Serial.print( "  TIM0_OUT  = FXIO_D" );
               pin = flexio_d_tim0_out;
               break;
            case 3:
               Serial.print( "  TIM1_OUT  = FXIO_D" );
               pin = flexio_d_tim1_out;
               break;
            case 4:
               Serial.print( "  TIM2_OUT  = FXIO_D" );
               pin = flexio_d_tim2_out;
               break;
            case 5:
               Serial.print( "  TIM4_OUT  = FXIO_D" );
               pin = flexio_d_tim4_out;
               break;
            case 6:
               Serial.print( "  TIM5_OUT  = FXIO_D" );
               pin = flexio_d_tim5_out;
               break;
            case 7:
               Serial.print( "  TIM7_OUT  = FXIO_D" );
               pin = flexio_d_tim7_out;
               break;
         }
         Serial.print( pin );

         // print the associated Teensy pin if it is being used for DEBUG
         #ifdef FLEX_DEBUG
             Serial.print( " = Teensy pin " );
             Serial.print( getTeensyPin(pin) );
         #endif
         Serial.println();
      }
   #endif

   return true;
}


// route IO pins
bool FlexIO_1553RX::config_io_pins(void)
{
   // 1553 input data pin
   setPinMux( m_t_Pin );

   #ifdef FLEX_DEBUG
      // if debug flag is set, route all output signals to pins.
      // otherwise, these signals are used internally by FlexIO,
      // but do not connect to Teensy pins

    //setPinMux( getTeensyPin(FLEX_1553RX_PIN_SHFT1_IN) );
    //setPinMux( getTeensyPin(FLEX_1553RX_PIN_SHFT2_IN) );
    //setPinMux( getTeensyPin(FLEX_1553RX_PIN_TIM0_OUT) );
    //setPinMux( getTeensyPin(FLEX_1553RX_PIN_TIM1_OUT) );
    //setPinMux( getTeensyPin(FLEX_1553RX_PIN_TIM5_OUT) );
    //setPinMux( getTeensyPin(FLEX_1553RX_PIN_TIM4_OUT) );
    //setPinMux( getTeensyPin(FLEX_1553RX_PIN_TIM7_OUT) );

      setPinMux( getTeensyPin(flexio_d_shift1_in) );
      setPinMux( getTeensyPin(flexio_d_shift2_in) );
      setPinMux( getTeensyPin(flexio_d_tim0_out) );
      setPinMux( getTeensyPin(flexio_d_tim1_out) );
      setPinMux( getTeensyPin(flexio_d_tim2_out) );
      setPinMux( getTeensyPin(flexio_d_tim4_out) );
      setPinMux( getTeensyPin(flexio_d_tim5_out) );
      setPinMux( getTeensyPin(flexio_d_tim7_out) );
   #endif
   return true;
}


uint8_t FlexIO_1553RX::parity( uint32_t data )
{
   uint32_t parity = 0;

   while(data > 0) {         // if it is 0 there are no more 1's to count
      if(data & 0x01) {       // see if LSB is 1
         parity++;             // why yes it is
      }
      data = data >> 1; //shift to next bit
   }

   return (~parity & 0x0001U);  // only need the low bit to determine odd / even
}



// configure 1553 receiver on FlexIO 3
bool FlexIO_1553RX::config_flex(void)
{
  // setup flex clock
  // note: FlexIO2 and FlexIO3 share the same clock
 // CCM_CS1CDR &= ~( CCM_CS1CDR_FLEXIO2_CLK_PODF( 7 ) ); // clear flex clock bits
 // CCM_CS1CDR |= CCM_CS1CDR_FLEXIO2_CLK_PODF( 4 );   // set flex clock = 40MHz
 //                                                   // clock speed = 480MHz/(N+1)
 // CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);      // enable clock


   // Check that the Flex clock is enabled
   // The flex clock should already be configured at this point.
   // The Flex clock MUST BE CONFIGURED before accessing any Flex register
   // or else the code will hang.
   if( !clock_running() )
      return(false);  // abort

   // if the Flex module gets hung up, reconfiguring will not fix it, you will
   // need to reset it. Flex module should be disabled during configuration or
   // else you will likely get "random" output transitions during config.
   // Reset and disable FLEXIO3 (clock MUST be enabled or this will hang)
   m_flex->CTRL |= 2;    // reset Flex module
   m_flex->CTRL &= 0xfffffffc;  // release reset and leave Flex disabled

   delayMicroseconds(1); // The first register assignment sometimes fails if there is
                         // no delay here. Hot sure why, might still be performing reset?
                         // The minimum delay required might be related to the Flex Clock speed

   // setup flex timer 0 *****************************************************
   // this is a 2MHz clock to step the state machine
   // it is clocked from from the FLEXIO clock
   // it is enabled by Shifter 3 status flag (TBD)
   m_flex->TIMCTL[0]    =
           FLEXIO_TIMCTL_TRGSEL( 13 )      |       // triggered by Shifter3 status flag (N*4) +1
           //FLEXIO_TIMCTL_TRGSEL( 8 )      |        // Input Pin 4 => (N * 2) = 8
           //FLEXIO_TIMCTL_TRGSEL( m_f_Pin * 2) |    // Input Pin 4 => (N * 2) = 8
           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG(PIN_ENA)  |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL( flexio_d_tim0_out )     |        // timer pin 10 (for debug only)
           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
           FLEXIO_TIMCTL_TIMOD( 1 );               // dual counter/baud mode

   m_flex->TIMCFG[0]   =
           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
           FLEXIO_TIMCFG_TIMDIS( 2 )      |        // disable timer on timer compare
           FLEXIO_TIMCFG_TIMENA( 6 )      |        // enable timer on trigger rising
           FLEXIO_TIMCFG_TSTOP(  0 )       ;       // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled

   //  //40 clocks,  divide clock by 20     ((n*2-1)<<8) | (baudrate_divider/2-1))
   //  34 clocks,  divide clock by 20     ((n*2-1)<<8) | (baudrate_divider/2-1))
   //                                     (34*2-1)<<8  | (20/2-1)
   //                                         (67)<<8  | (9)
   //                                          0x4300  | 0x09
   //m_flex->TIMCMP[0]   =   0x4F09;
   m_flex->TIMCMP[0]   =   0x4309;


   // setup flex timer 1 *****************************************************
   // this is a 1MHz clock to step both shifters
   // it is clocked from from the FLEXIO clock
   // it is enabled by TBD
   m_flex->TIMCTL[1]    =
           FLEXIO_TIMCTL_TRGSEL( 13 )     |       // Shifter3 status flag =(3 * 4) + 1
           //FLEXIO_TIMCTL_TRGSEL(m_f_Pin * 2)      |        // Input Pin 4 =(2 * 4) + 0)
           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL( flexio_d_tim1_out )   |   // timer pin 11 (for debug only)
           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
           FLEXIO_TIMCTL_TIMOD( 1 );               // dual counter/baud mode

   m_flex->TIMCFG[1]    =
           FLEXIO_TIMCFG_TIMOUT( 1 )      |        // timer output = logic low when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
           FLEXIO_TIMCFG_TIMDIS( 2 )      |        // disable timer on timer compare
           FLEXIO_TIMCFG_TIMENA( 6 )      |        // enable timer on trigger rising
           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled

   // 20 shifts are needed to clock in the data, and one more shift is needed
   // to STORE the data into the Shift Buffer. If only 20 clocks are provided,
   // the last bit (parity) will not be captured.
   // It seems that just a half clock is needed to store the data. This extra
   // half clock is not visible in the timer output line, but does capture all
   // 20 bits.
   //
   // //20-1/2 shifts,  divide clock by 40  ((n*2-1)<<8) | (baudrate_divider/2-1))
   // 17-1/2 shifts,  divide clock by 40  ((n*2-1)<<8) | (baudrate_divider/2-1))
   //                                   (17.5*2-1)<<8  | (40/2-1)
   //                                         (34)<<8  | (19)
   //                                          0x2200  | 0x13
   //m_flex->TIMCMP[1]  =  0x2813;
   m_flex->TIMCMP[1]  =  0x2213;


  // setup flex timer 2 *****************************************************
  // this timer watches for the end of receive (end of transmission from the other end).
  // if we dont see a new SYNC in 21us, it is safe to assume that the transmission has ended.
  // this timer will be reset by Shifter3 every time a new word is received (every 20 us),
  // so this timer will not timeout until after transmission has ended.
  // it is clocked from the FlexIO clock
  // it is enabled by Timer1
  // it is reset by Shifte3 flag (sync pulse)
   m_flex->TIMCTL[2]    =
           FLEXIO_TIMCTL_TRGSEL( 13 )     |        // trigger on Shifter3 status flag (N * 4) + 1
           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG(PIN_ENA)  |        // timer pin is an output
           FLEXIO_TIMCTL_PINSEL(flexio_d_tim2_out) |      // timer pin (for debug only)
           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit timer mode

   m_flex->TIMCFG[2]    =
           FLEXIO_TIMCFG_TIMOUT( 1 )      |        // timer output = logic low when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
           FLEXIO_TIMCFG_TIMRST( 6 )      |        // reset timer on trigger rising edge
           FLEXIO_TIMCFG_TIMDIS( 2 )      |        // disable timer on timer compare
           FLEXIO_TIMCFG_TIMENA( 1 )      |        // timer is enabled on Timer N-1 enable
           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled

   // Count to 21us (17 bits + 4us minimum response time) see figure 8 of MIL-STD-1553b spec.
   // We really cant make it any faster than this because this is bearly past the point where
   // we would detect the next sync pattern.
   //   21us * 40count/us = 840 counts
   //                     (n-1)
   //                   (840-1)
   //                     (839)
   m_flex->TIMCMP[2]   =   839;


  // setup flex timer 3 *****************************************************
  // this is the shift counter for Shifter3
  // the shift clock is passed thru from Timer2
  // it is clocked from from Timer2
  // it is always enabled
  // it is reset by Timer4
   m_flex->TIMCTL[3]    =
           FLEXIO_TIMCTL_TRGSEL( 23 )     |        // trigger on Timer5 out (N * 4)+3
           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 0 )      |        // timer pin is an input
           FLEXIO_TIMCTL_PINSEL(flexio_d_tim4_out) |      // timer pin (used as Reset input)
           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit timer mode

   m_flex->TIMCFG[3]    =
           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 3 )      |        // decrement on Trig, shift clock = Trig
           FLEXIO_TIMCFG_TIMRST( 4 )      |        // reset timer on pin rising edge
           FLEXIO_TIMCFG_TIMDIS( 0 )      |        // timer never disables
           FLEXIO_TIMCFG_TIMENA( 0 )      |        // timer is always enabled
           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled

   //  102 shifts                           (n*2-1)
   //                                     (102*2-1)
   //                                         (204)
   m_flex->TIMCMP[3]    =    203;
   // this would normally timeout and stop after 102 shifts, however it will be reset by Timer4
   // after 100 shifts, so the timeout never happens


   // setup flex timer 4 *****************************************************
   // this is an an extra timer that produces a reset to Timer3
   // it is clocked from from FlexIO
   // it is always enabled
   //
   // This is a workaround for a bug (in my opinion) in "Match continuous mode" which, as it
   // turns out, does not match continuously! When the controlling timer (Timer3 in this case)
   // wraps around, it causes the associated shifter (Shifter 3 here) to clear. This makes sense
   // for transmit and receive modes, but not "match continuous". This timer is being used to
   // reset Timer3 back to its initial count, before it reaches zero. So Timer3 will never wrap
   // and Shifter3 will never clear, allowing it really match continuously.

   m_flex->TIMCTL[4]    =
           FLEXIO_TIMCTL_TRGSEL( 0 )      |        // trigger not used
           //FLEXIO_TIMCTL_TRGPOL         |        // trigger not used
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL(flexio_d_tim4_out) |      // timer pin (resets Timer3)
           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit counter mode

   m_flex->TIMCFG[4]    =
           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
           FLEXIO_TIMCFG_TIMDIS( 0 )      |        // never disable
           FLEXIO_TIMCFG_TIMENA( 0 )      |        // timer is always enabled
           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled

   //  40 shifts * 8 flex clocks             (n-1)
   //                                      (320-1)
   //                                        (319)
   //                                       0x013F
   m_flex->TIMCMP[4]    =  0x13F;
   // output toggles after 50 shifts, but reset is on rising edge only, so reset is every 100 shifts


   // setup flex timer 5 *****************************************************
   // this is a 5MHz clock for shifter 3
   // it is clocked from from the FLEXIO clock
   // it is always enabled
   m_flex->TIMCTL[5]    =
            FLEXIO_TIMCTL_TRGSEL( 0 )      |        // trigger not used
            //FLEXIO_TIMCTL_TRGPOL         |        // trigger not used
            FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
            FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
            FLEXIO_TIMCTL_PINSEL( flexio_d_tim5_out )      |        // timer pin 8 (for debug only)
            // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
            FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit timer mode
            //FLEXIO_TIMCTL_TIMOD( 1 );               // dual counter/baud mode

   m_flex->TIMCFG[5]    =
            FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
            FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
            FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
            FLEXIO_TIMCFG_TIMDIS( 0 )      |        // never disable
            FLEXIO_TIMCFG_TIMENA( 0 )      |        // timer is always enabled
            FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
            // FLEXIO_TIMCFG_TSTART                 // start bit disabled

   //  Flex clock = 40MHz, we want 5MHz, so divide by 8
   //  TIMCMP = divider/2-1 = 8/2-1 = 3
   m_flex->TIMCMP[5]    =    0x0003U;


   // Setup flex timer 7 *****************************************************
   // For debug, this passes the sync pulse (from Shifter3) thru to an IO pin for debug
   // It is also used as the sync interrupt source. The sync detector (Shifter3) can
   // produce an interrupt, but in Match Continuous Mode, it does not latch. This means
   // that if the system is busy with another interrupt, this interrupt can be missed.
   // By passing the sync thru a timer, and using the timer interrupt (which does latch)
   // is solves this problem.
   // it is always enabled
   m_flex->TIMCTL[7]    =
           FLEXIO_TIMCTL_TRGSEL( 13 )     |        // Shifter3 status flag =(3 * 4) + 1
           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG(PIN_ENA)  |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL(flexio_d_tim7_out)      |        // timer pin 9 (for debug only)
           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit timer mode

   m_flex->TIMCFG[7]    =
           FLEXIO_TIMCFG_TIMOUT( 1 )      |        // timer output = logic low when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 3 )      |        // decrement on Trigger input (both edges), Shift clock equals Trigger input.
           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
           FLEXIO_TIMCFG_TIMDIS( 0 )      |        // never disable
           FLEXIO_TIMCFG_TIMENA( 0 )      |        // timer is always enabled
           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled


   // setup data shifter 1 **************************************************
   // This shifter is configured in receive Mode
   // It captures the data bits from the state machine decoder
   m_flex->SHIFTCTL[1]  =
           FLEXIO_SHIFTCTL_TIMSEL( 1 )    |        // clocked from timer 1
           FLEXIO_SHIFTCTL_TIMPOL         |        // on falling edge
           FLEXIO_SHIFTCTL_PINCFG( 0 )    |        // pin output disabled
           FLEXIO_SHIFTCTL_PINSEL( FLEX_1553RX_PIN_SHFT1_IN )    |        // FLEXIO pin 0
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 1 );              // receive mode

   m_flex->SHIFTCFG[1]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled


   // setup data shifter 2 **************************************************
   // This shifter is configured in receive Mode
   // It captures the fault bits from the state machine decoder
   m_flex->SHIFTCTL[2]  =
           FLEXIO_SHIFTCTL_TIMSEL( 1 )    |        // clocked from timer 1
           FLEXIO_SHIFTCTL_TIMPOL         |        // on falling edge
           FLEXIO_SHIFTCTL_PINCFG( 0 )    |        // pin output disabled
           FLEXIO_SHIFTCTL_PINSEL( FLEX_1553RX_PIN_SHFT2_IN )    |        // FLEXIO pin 1
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 1 );              // receive mode

   m_flex->SHIFTCFG[2]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled


   // setup data shifter 3 **************************************************
   // This shifter is configured in Match Continuous Mode
   // It watches for the sync pattern at the start of the 1553 transmission
   // and when found, triggers the data capture
   m_flex->SHIFTCTL[3]  =
           FLEXIO_SHIFTCTL_TIMSEL( 3 )    |        // clocked from timer 3
           // FLEXIO_SHIFTCTL_TIMPOL      |        // on positive edge
           FLEXIO_SHIFTCTL_PINCFG( 0 )    |        // pin output disabled
           FLEXIO_SHIFTCTL_PINSEL(flexio_d_data_in)  |    // FLEXIO pin 40     (Input data stream)
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 5 );              // match continuous mode

   m_flex->SHIFTCFG[3]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled

   // The trigger pattern is 1.5 ms of zeros, followed by 1.5 ms of 1's
   // for a total 3 ms pattern @5MHz = 15 bits.
   // We are using 5MHz because this is the fastest that we can sample
   // and still have the pattern fit in 16 bits.
   // Reduce this to 14 bits (7 high, 7 low) to be sure we dont capture
   // anything outside the trigger pattern.
   // mask = 1100 0000 0000 0000   pattern = xx11 1111 1000 0000
   //      = 0xC000                        = 0xff80
   //m_flex->SHIFTBUFBIS[3] =  0xC000ff80U;
   //m_flex->SHIFTBUFBIS[3] =  0x8001ff00U;
   m_flex->SHIFTBUFBIS[3] =  FLEX1553_COMMAND_SYNC_PATTERN;




   // State Machine **************************************************
   // This uses five shifters as a five-state state machine to decode the
   // Manchester encoded data
   //  state 0 = fault - no transiton during bit time
   //  state 5 = first  half bit time, with input = 1
   //  state 7 = second half bit time, with input transitioning to 0
   //  state 6 = first  half bit time, with input = 0
   //  state 4 = second half bit time, with input transitioning to 1
   // all 5 shifters are set the same, only the state tables are different

   // setup state 0
   m_flex->SHIFTCTL[0]  =
           FLEXIO_SHIFTCTL_TIMSEL( 0 )    |        // controlled from timer 0
           FLEXIO_SHIFTCTL_TIMPOL         |        // on negative edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // enable output
           FLEXIO_SHIFTCTL_PINSEL( flexio_d_data_in )   |   // sets data in pins for state machine
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 6 );              // state mode

   m_flex->SHIFTCFG[0]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0xF )  |        // disable FXIO_D[7:4] outputs
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP(  3 )    |        // disable FXIO_D[3:2] outputs
           FLEXIO_SHIFTCFG_SSTART( 0 );            // enable FXIO_D[1:0] outputs
                                                   //    Teensy pins 19, 18
 // setup state 4
   m_flex->SHIFTCTL[4]  =
           FLEXIO_SHIFTCTL_TIMSEL( 0 )    |        // controlled from timer 0
           FLEXIO_SHIFTCTL_TIMPOL         |        // on negative edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // enable output
           FLEXIO_SHIFTCTL_PINSEL( flexio_d_data_in )   |   // sets data in pins for state machine
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 6 );              // state mode

   m_flex->SHIFTCFG[4]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0xF )  |        // disable FXIO_D[7:4] outputs
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP(  3 )    |        // disable FXIO_D[3:2] outputs
           FLEXIO_SHIFTCFG_SSTART( 0 );            // enable FXIO_D[1:0] outputs
                                                   //    Teensy pins 2, 3, 4, 33

 // setup state 5
   m_flex->SHIFTCTL[5]  =
           FLEXIO_SHIFTCTL_TIMSEL( 0 )    |        // controlled from timer 0
           FLEXIO_SHIFTCTL_TIMPOL         |        // on negative edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // enable output
           FLEXIO_SHIFTCTL_PINSEL( flexio_d_data_in )   |   // sets data in pins for state machine
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 6 );              // state mode

   m_flex->SHIFTCFG[5]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0xF )  |        // disable FXIO_D[7:4] outputs
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP(  3 )    |        // disable FXIO_D[3:2] outputs
           FLEXIO_SHIFTCFG_SSTART( 0 );            // enable FXIO_D[1:0] outputs
                                                   //    Teensy pins 2, 3, 4, 33

 // setup state 6
   m_flex->SHIFTCTL[6]  =
           FLEXIO_SHIFTCTL_TIMSEL( 0 )    |        // controlled from timer 0
           FLEXIO_SHIFTCTL_TIMPOL         |        // on negative edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // enable output
           FLEXIO_SHIFTCTL_PINSEL( flexio_d_data_in )   |  // sets data in pins for state machine
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 6 );              // state mode

   m_flex->SHIFTCFG[6]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0xF )  |        // disable FXIO_D[7:4] outputs
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP(  3 )    |        // disable FXIO_D[3:2] outputs
           FLEXIO_SHIFTCFG_SSTART( 0 );            // enable FXIO_D[1:0] outputs
                                                   //    Teensy pins 2, 3, 4, 33

 // setup state 7
   m_flex->SHIFTCTL[7]  =
           FLEXIO_SHIFTCTL_TIMSEL( 0 )    |        // controlled from timer 0
           FLEXIO_SHIFTCTL_TIMPOL         |        // on negative edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // enable output
           FLEXIO_SHIFTCTL_PINSEL( flexio_d_data_in )   |   // sets data in pins for state machine
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 6 );              // state mode

   m_flex->SHIFTCFG[7]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0xF )  |        // disable FXIO_D[7:4] outputs
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP(  3 )    |        // disable FXIO_D[3:2] outputs
           FLEXIO_SHIFTCFG_SSTART( 0 );            // enable FXIO_D[1:0] outputs
                                                   //    Teensy pins 2, 3, 4, 33
   // Two output bits from the state machine, FXIO_D[1:0], represent data bits
   // (on flex bit 0) and faults (no transition, on flex bit 1). These two bits
   //  will be captured by shifters.

   // load state lookup table               output  : next state
   m_flex->SHIFTBUF[0] =  0x0202E02EU;  //  0000 0010 : 000 000 101 110 000 000 101 110b
   m_flex->SHIFTBUF[5] =  0x001C01C0U;  //  0000 0000 : 000 111 000 000 000 111 000 000b
   m_flex->SHIFTBUF[7] =  0x0102E02EU;  //  0000 0001 : 000 000 101 110 000 000 101 110b
   m_flex->SHIFTBUF[6] =  0x00800800U;  //  0000 0000 : 100 000 000 000 100 000 000 000b
   m_flex->SHIFTBUF[4] =  0x0002E02EU;  //  0000 0000 : 000 000 101 110 000 000 101 110b



   // enable FLEXIO
   m_flex->CTRL |= 1;    // enable FLEXIO3 module


   //m_flex->TIMCTL[0] = 0x8430B01;
   //Serial.print("RX Timer0: ");
   //Serial.print(m_flex->TIMCTL[0], HEX);
   //Serial.print(" : ");
   //Serial.print(m_flex->TIMCFG[0], HEX);
   //Serial.print(" : ");
   //Serial.print(m_flex->TIMCMP[0], HEX);
   //Serial.println();
   //
   //Serial.print("RX Timer1: ");
   //Serial.print(m_flex->TIMCTL[1], HEX);
   //Serial.print(" : ");
   //Serial.print(m_flex->TIMCFG[1], HEX);
   //Serial.print(" : ");
   //Serial.print(m_flex->TIMCMP[1], HEX);
   //Serial.println();
   //
   //Serial.println( (uint64_t)(&m_flex->TIMCTL[0]), HEX);
   //Serial.println( (uint64_t)(&m_flex->TIMCTL[1]), HEX);

   return( true );
}




// get status flags for 8 timers and 8 shifters in FlexIO3
// @return  8 timer flags : 8 shifter flags (one byte for each)
unsigned long FlexIO_1553RX::get_status( void )
{
   unsigned long flags;

   // make sure Flex clock is enabled
    if( !clock_running() )
       return( 0 );  // will hang if Flex clock not enabled

   flags = (m_flex->SHIFTERR << 16) | (m_flex->SHIFTSTAT << 8) | m_flex->TIMSTAT;
   m_flex->SHIFTERR  = 0xff;  // clear flags
   m_flex->SHIFTSTAT = 0xff;
   m_flex->TIMSTAT   = 0xff;
   return( flags );
}



// reads directly from the FlexIO data register
unsigned long FlexIO_1553RX::read_data( void )
{
  // make sure Flex clock is enabled
  if( !clock_running() )
     return( 0 );  // abort
     // Todo: need a way to return an error

  return( m_flex->SHIFTBUFBIS[1] );
}



unsigned long FlexIO_1553RX::read_faults( void )
{
  // make sure Flex clock is enabled
  if( !clock_running() )
     return( 0 );  // abort
     // Todo: need a way to return an error

  return( m_flex->SHIFTBUFBIS[2] );
}


void FlexIO_1553RX::set_sync( uint8_t sync_type )
{
   switch(sync_type) {
      case FLEX1553_COMMAND_WORD:
      case FLEX1553_STATUS_WORD:
         // Shifter3 is in Match Continuous mode, and is constantly serarching for the sync pattern
         //noInterrupts();
            m_flex->SHIFTBUFBIS[3] = FLEX1553_COMMAND_SYNC_PATTERN;
            g_syncType[m_flex_num] = sync_type;
         //interrupts();
         break;
      case FLEX1553_DATA_WORD:
         //noInterrupts();
            m_flex->SHIFTBUFBIS[3] = FLEX1553_DATA_SYNC_PATTERN;
            g_syncType[m_flex_num] = sync_type;
         //interrupts();
         break;
   }
}



// set trigger pattern for RX data capture. For debug only
int FlexIO_1553RX::set_trigger( unsigned int pattern, unsigned int mask )
{
   // the upper 16 bits are the mask bits
   m_flex->SHIFTBUFBIS[3] = ((unsigned long)mask << 16) | (unsigned long)pattern;

   return( 0 );
}



// disables the FlexIO interrupt, which effectively disables the receiver
void FlexIO_1553RX::disable(void)
{
   // disable all interrupt sources
   m_flex->SHIFTSIEN  = 0;
   m_flex->TIMIEN     = 0;
}

// enables the FlexIO RX interrupts
void FlexIO_1553RX::enable(void)
{
   FlexIO_Base::enableInterruptSource(FLEXIO_SHIFTERS, 1);   // RX data
   FlexIO_Base::enableInterruptSource(FLEXIO_SHIFTERS, 3);   // SYNC pulse
}


// Clear buffer and get ready for new RX packet
void FlexIO_1553RX::flush(void)
{
   noInterrupts();
      g_rxDataCount[m_flex_num] = 0;  // clear data buffer write pointer
      read_data(); // make sure receiver is empty

      // set sync for new packet
      //m_flex->SHIFTBUFBIS[3] = FLEX1553_COMMAND_SYNC_PATTERN;
      //g_syncType[m_flex_num] = FLEX1553_COMMAND_WORD;
   interrupts();
   m_rxDataRdPtr = 0; // clear data buffer read pointer
}


// Number of words left in the buffer to read
uint8_t FlexIO_1553RX::available(void)
{
   return(g_rxDataCount[m_flex_num] - m_rxDataRdPtr);
}


// Total Number of words which have been received into the buffer
uint8_t FlexIO_1553RX::word_count(void)
{
   return(g_rxDataCount[m_flex_num]);
}


uint8_t FlexIO_1553RX::getSyncType(void)
{
   return g_syncType[m_flex_num];
}


// Read RX one word from buffer
// lower 16-bits are the data bits [15:0]
// bit 30 is high if there is a parity error
// bit 29 is high if transtion faults were detected
// -1 is returned if no data available in buffer
int32_t FlexIO_1553RX::read(void)
{
   // data contained in the buffer is raw. It must still be checked
   // for proper parity and for transition faults (transition not detected)

   // first check if there is any data in the buffer
   if(m_rxDataRdPtr >= g_rxDataCount[m_flex_num])
      return(-1);

   int32_t data  = g_rxData[m_flex_num][m_rxDataRdPtr];
   int32_t fault = g_rxFault[m_flex_num][m_rxDataRdPtr++];
   uint8_t parity_bit = data & 0x01;   // lowest bit is parity bit
   data = (data >> 1) & 0xffff;     // the next 16-bits is the actual data

   // check the parity
   bool parity_err = false;
   if(parity(data) != parity_bit)
      parity_err = true;

   // check for faults
   // the FlexIO statemachine checks for bit transitions while receiving data.
   // There should a transition on every bit. If there is no transtion detected
   // then a one is written to the fault shifter (Shifter2). We are expecting to
   // see 17 zeros in the fault buffer, indicating that a transition was detected
   // on every bit, including the parity bit.
   bool fault_err = (fault != 0);

   if(parity_err)
      data = data | 0x40000000; // set bit 30
   if(fault_err)
      data = data | 0x20000000; // set bit 29

   return(data);
}
