#include <Arduino.h>
#include <Flex1553.h>

// 7/12/21  change to a 48MHz clock so we can derive 6MHz from timers
// 9/10/21  reconfigured code to use C++ class instead of standard C
// 9/20/21  created base class for reusable FlexIO functions
// 10/30/21 changed clock speed back to 40MHz
// 1/??/22  Added changes for "Match Continuous mode", RX working
// 2/13/22  Added interrupt routine to capture RX data

// Bug list
//

#define REVERSE_LOOKUP   true

// Optional build flags:
#define FLEX_PRINT_MESSAGES   // uncomment to print startup messages to serial port
#define FLEX_DEBUG            // uncomment to bring out debug pins
//#define FLEX02_DEBUG  // uncomment to bring out debug pins
//#define FLEX03_DEBUG  // uncomment to bring out debug pins
//#define FLEX01_TX_CHB // enable transmit on both A and B channels
   // if not defined, only channel A output pins will be used




/***************************************************************************************
*    Start of 1553 RX Class
***************************************************************************************/

// pins used by this module
//#define FLEX1_1553RX_PIN_DATA_IN    4    // data input pin
#define FLEX1_1553RX_PIN_TIM0_OUT  10    // 2 MHz state machine clock
#define FLEX1_1553RX_PIN_TIM1_OUT   5    // 1 MHz shift clock
#define FLEX1_1553RX_PIN_TIM2_OUT   8    // 5 MHz clock
#define FLEX1_1553RX_PIN_TIM4_OUT  12    // Reset pulse to Timer3
#define FLEX1_1553RX_PIN_TIM7_OUT   9    // for debug only
#define FLEX1_1553RX_PIN_SHFT1_IN   0    // data bits from state machine
#define FLEX1_1553RX_PIN_SHFT2_IN   1    // fault bits from state machine

//#define FLEX2_1553RX_PIN_DATA_IN    4    // data input pin
#define FLEX2_1553RX_PIN_TIM0_OUT  10    // 2 MHz state machine clock
#define FLEX2_1553RX_PIN_TIM1_OUT   5    // 1 MHz shift clock
#define FLEX2_1553RX_PIN_TIM2_OUT   8    // 5 MHz clock
#define FLEX2_1553RX_PIN_TIM4_OUT  12
#define FLEX2_1553RX_PIN_TIM7_OUT   9    // for debug only
#define FLEX2_1553RX_PIN_SHFT1_IN   0    // data bits from state machine
#define FLEX2_1553RX_PIN_SHFT2_IN   1    // fault bits from state machine

//#define FLEX3_1553RX_PIN_DATA_IN    4    // data input pin
#define FLEX3_1553RX_PIN_TIM0_OUT  11    // 2 MHz state machine clock
#define FLEX3_1553RX_PIN_TIM1_OUT   5    // 1 MHz shift clock
#define FLEX3_1553RX_PIN_TIM2_OUT   8    // 5 MHz clock
#define FLEX3_1553RX_PIN_TIM4_OUT  12
#define FLEX3_1553RX_PIN_TIM7_OUT   9    // for debug only
#define FLEX3_1553RX_PIN_SHFT1_IN   0    // data bits from state machine
#define FLEX3_1553RX_PIN_SHFT2_IN   1    // fault bits from state machine

// #define FLEX3_1553RX_PIN_SHFT3_IN   4    // 1553 data in
#define FLEX_1553RX_PIN_SHFT1_IN    FLEX3_1553RX_PIN_SHFT1_IN
#define FLEX_1553RX_PIN_SHFT2_IN    FLEX3_1553RX_PIN_SHFT2_IN
#define FLEX_1553RX_PIN_TIM0_OUT    FLEX3_1553RX_PIN_TIM0_OUT
#define FLEX_1553RX_PIN_TIM1_OUT    FLEX3_1553RX_PIN_TIM1_OUT
#define FLEX_1553RX_PIN_TIM2_OUT    FLEX3_1553RX_PIN_TIM2_OUT
#define FLEX_1553RX_PIN_TIM4_OUT    FLEX3_1553RX_PIN_TIM4_OUT
#define FLEX_1553RX_PIN_TIM7_OUT    FLEX3_1553RX_PIN_TIM7_OUT

// big ugly macro that decides which of the FLEX pin definitions to use
//#define FLEX_1553RX_PIN_SHFT2_IN  ((int8_t)((m_flex_num == 1)? FLEX1_1553RX_PIN_SHFT2_IN : ((m_flex_num == 2)? FLEX2_1553RX_PIN_SHFT2_IN : FLEX3_1553RX_PIN_SHFT2_IN)))
//#define FLEX_1553RX_PIN_DATA_IN   ((int8_t)((m_flex_num == 1)? FLEX1_1553RX_PIN_DATA_IN : ((m_flex_num == 2)? FLEX2_1553RX_PIN_DATA_IN : FLEX3_1553RX_PIN_DATA_IN)))


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

static volatile uint32_t g_syncType[4] = {FLEX1553_COMMAND_WORD, FLEX1553_COMMAND_WORD, FLEX1553_COMMAND_WORD, FLEX1553_COMMAND_WORD};
static volatile uint8_t  g_rxDataCount[4] = {0};  // keep track of how many words are in the buffer
static volatile uint32_t g_rxData[4][33];  // RX data buffer, 33 words (including command) is the longest valid packet
static volatile uint32_t g_rxFault[4][33];

static void isrFlex1_1553Rx(void)
{
}

static void isrFlex2_1553Rx(void)
{
}

static void isrFlex3_1553Rx(void)
{
   int flags =  FLEXIO3_SHIFTSTAT;

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
         g_rxData[3][offset]  = FLEXIO3_SHIFTBUFBIS1; // reading the shifter also clears the interrupt flag
         g_rxFault[3][offset] = FLEXIO3_SHIFTBUFBIS2;
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
   m_f_Pin    = getTeensyPin(rxPin, REVERSE_LOOKUP);
   m_altFlex  = (m_flex_num == 3)? 9 : 4; // FlexIO3 uses ALT9, FLEXIO1 & 2 use ALT4
   m_altGpio  = 5;  // gpio always uses Alt5
}



bool FlexIO_1553RX::begin( void )
{
   #ifdef FLEX_PRINT_MESSAGES
      if(m_flex_num == 0)
         Serial.println( "Error: invalid flexIO module number" );
      else {
         Serial.print( "Configuring 1553RX on FlexIO" );
         Serial.print( m_flex_num );
         Serial.println();
      }
   #endif

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
         FlexIO_Base::attachInterrupt(isrFlex1_1553Rx);
         break;
      case FLEXIO2:
         FlexIO_Base::attachInterrupt(isrFlex2_1553Rx);
         break;
      case FLEXIO3:
         FlexIO_Base::attachInterrupt(isrFlex3_1553Rx);
         break;
   }
   //FlexIO_Base::attachInterrupt(isrFlex3_1553Rx);
   FlexIO_Base::enableInterruptSource(FLEXIO_SHIFTERS, 1);
   FlexIO_Base::enableInterruptSource(FLEXIO_SHIFTERS, 3);
   //flex1553RX.enableInterruptSource(FLEXIO_SHIFTERS, 1);
   //flex1553RX.enableInterruptSource(FLEXIO_SHIFTERS, 3);


   #ifdef FLEX_PRINT_MESSAGES  // print out pins used
      // Receiver Input pin
      Serial.print( "  RX_IN = FXIO_D" );
      Serial.print( m_f_Pin );
      Serial.print( " = Teensy pin " );
      Serial.print( m_t_Pin );
      Serial.println();

      // print out the internal FXIO connections used
      int pin = -1;
      for( int i=0; i<7; i++ ) {
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
               pin = FLEX_1553RX_PIN_TIM0_OUT;
               break;
            case 3:
               Serial.print( "  TIM1_OUT  = FXIO_D" );
               pin = FLEX_1553RX_PIN_TIM1_OUT;
               break;
            case 4:
               Serial.print( "  TIM2_OUT  = FXIO_D" );
               pin = FLEX_1553RX_PIN_TIM2_OUT;
               break;
            case 5:
               Serial.print( "  TIM4_OUT  = FXIO_D" );
               pin = FLEX_1553RX_PIN_TIM4_OUT;
               break;
            case 6:
               Serial.print( "  TIM7_OUT  = FXIO_D" );
               pin = FLEX_1553RX_PIN_TIM7_OUT;
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


bool FlexIO_1553RX::config_io_pins(void)
{
   // route IO pins to FlexIO 3
    // 1553 input bit stream
   //IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04 = 9;      // FLEXIO pin4    Teensy pin 40
   //int8_t fPin = 4;
   setPinMux( m_t_Pin );
#ifdef FLEX03_DEBUG  // bring out internal signals
    // Timer 0,1,2 outputs  - for debug only
   IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10 = 9;      // FLEXIO pin10   Teensy pin 20
   IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = 9;      // FLEXIO pin5    Teensy pin 41
   IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08 = 9;      // FLEXIO pin8    Teensy pin 22
     // Compare output      - for debug only
   IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 = 9;      // FLEXIO pin9    Teensy pin 23
     // State machine out / shifter in - for debug only
   IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_00 = 9;      // FLEXIO pin0    Teensy pin 19
   IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_01 = 9;      // FLEXIO pin1    Teensy pin 18
#endif
   #ifdef FLEX_DEBUG
      setPinMux( getTeensyPin(FLEX_1553RX_PIN_SHFT1_IN) );
      setPinMux( getTeensyPin(FLEX_1553RX_PIN_SHFT2_IN) );
      setPinMux( getTeensyPin(FLEX_1553RX_PIN_TIM0_OUT) );
      setPinMux( getTeensyPin(FLEX_1553RX_PIN_TIM1_OUT) );
      setPinMux( getTeensyPin(FLEX_1553RX_PIN_TIM2_OUT) );
      setPinMux( getTeensyPin(FLEX_1553RX_PIN_TIM4_OUT) );
      setPinMux( getTeensyPin(FLEX_1553RX_PIN_TIM7_OUT) );
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
           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL( FLEX_1553RX_PIN_TIM0_OUT )     |        // timer pin 10 (for debug only)
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
           FLEXIO_TIMCTL_PINSEL( FLEX_1553RX_PIN_TIM1_OUT )   |   // timer pin 11 (for debug only)
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
   // this is a 5MHz clock for shifter 3
   // it is clocked from from the FLEXIO clock
   // it is always enabled
   m_flex->TIMCTL[2]    =
            FLEXIO_TIMCTL_TRGSEL( 0 )      |        // trigger not used
            //FLEXIO_TIMCTL_TRGPOL         |        // trigger not used
            FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
            FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
            FLEXIO_TIMCTL_PINSEL( FLEX_1553RX_PIN_TIM2_OUT )      |        // timer pin 8 (for debug only)
            // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
            FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit timer mode
            //FLEXIO_TIMCTL_TIMOD( 1 );               // dual counter/baud mode

   m_flex->TIMCFG[2]    =
            FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
            FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
            FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
            FLEXIO_TIMCFG_TIMDIS( 0 )      |        // never disable
            FLEXIO_TIMCFG_TIMENA( 0 )      |        // timer is always enabled
            FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
            // FLEXIO_TIMCFG_TSTART                 // start bit disabled

   //  Flex clock = 40MHz, we want 5MHz, so divide by 8
   //  TIMCMP = divider/2-1 = 8/2-1 = 3
   m_flex->TIMCMP[2]    =    0x0003U;


  // setup flex timer 3 *****************************************************
  // this is the shift counter for Shifter3
  // the shift clock is passed thru from Timer2
  // it is clocked from from Timer2
  // it is always enabled
  // it is reset by Timer4
   m_flex->TIMCTL[3]    =
           FLEXIO_TIMCTL_TRGSEL( 11 )     |        // trigger on Timer2 out (2 * 4)+3
           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 0 )      |        // timer pin is an input
           FLEXIO_TIMCTL_PINSEL(FLEX_1553RX_PIN_TIM4_OUT) |      // timer pin (used as Reset input)
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
           FLEXIO_TIMCTL_PINSEL(FLEX_1553RX_PIN_TIM4_OUT) |      // timer pin (resets Timer3)
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


   // setup flex timer 7 *****************************************************
   // for debug only
   // this just passes the trigger thru to an IO pin for debug
   // it is always enabled
   m_flex->TIMCTL[7]    =
           FLEXIO_TIMCTL_TRGSEL( 13 )     |        // Shifter3 status flag =(3 * 4) + 1
           //FLEXIO_TIMCTL_TRGSEL( 3 )     |        // Timer 0 output =(N * 4) + 3
           //FLEXIO_TIMCTL_TRGSEL(m_f_Pin * 2)      |        // Input Pin 4 =(N * 2)
           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL( FLEX_1553RX_PIN_TIM7_OUT )      |        // timer pin 9 (for debug only)
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
           FLEXIO_SHIFTCTL_PINSEL(m_f_Pin)  |    // FLEXIO pin 40     (Input data stream)
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
           FLEXIO_SHIFTCTL_PINSEL( m_f_Pin )   |   // input bit used
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
           FLEXIO_SHIFTCTL_PINSEL( m_f_Pin )   |   // only 1 input bit used              // FLEXIO_SHIFTCTL_PINPOL      |        // active high
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
           FLEXIO_SHIFTCTL_PINSEL( m_f_Pin )   |   // only 1 input bit used              // FLEXIO_SHIFTCTL_PINPOL      |        // active high
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
           FLEXIO_SHIFTCTL_PINSEL( m_f_Pin )   |   // only 1 input bit used              // FLEXIO_SHIFTCTL_PINPOL      |        // active high
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
           FLEXIO_SHIFTCTL_PINSEL( m_f_Pin )   |   // only 1 input bit used              // FLEXIO_SHIFTCTL_PINPOL      |        // active high
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
         // Shifter3 is in Match Continuous mode, and is constantly serarching for the sync pattern
         noInterrupts();
            m_flex->SHIFTBUFBIS[3] = FLEX1553_COMMAND_SYNC_PATTERN;
            g_syncType[m_flex_num] = FLEX1553_COMMAND_WORD;
         interrupts();
         break;
      case FLEX1553_DATA_WORD:
         noInterrupts();
            m_flex->SHIFTBUFBIS[3] = FLEX1553_DATA_SYNC_PATTERN;
            g_syncType[m_flex_num] = FLEX1553_DATA_WORD;
         interrupts();
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







// ********************************************************
// This is for testing the "Match Continuous Mode"
// Use a square <30kHz wave as input
// this adds another timer (Timer4) to config2, to reset Timer2 before it can timeout
// this should be the same as Config4, but using 16-bit counters
// this appears to work!!
// timers are not in any logical order, this is just how it was configured when I got it to work
int Flex1_1553Sync_config5(void)
{
//  // setup flex clock for i.MXRT1062
//  // note that the clock setup is device specific
//  CCM_CCGR5 |= CCM_CCGR5_FLEXIO1(CCM_CCGR_OFF);      // disable clock
//  CCM_CDCDR &= ~( CCM_CDCDR_FLEXIO1_CLK_PODF( 7 ) ); // clear flex clock bits
//  CCM_CDCDR |= CCM_CDCDR_FLEXIO1_CLK_PODF( 5 );     // set flex clock = 40MHz
//                                                    // clock speed = 480MHz/2/(N+1)
//  CCM_CCGR5 |= CCM_CCGR5_FLEXIO1(CCM_CCGR_ON);      // enable clock
//
//  // if the Flex module gets hung up, reconfiguring will not fix it, you will
//  // need to reset it. Flex module should be disabled during configuration or
//  // else you will likely get "random" output transitions during config.
//  // Reset and disable FLEXIO1 (clock MUST be enabled or this will hang)
//  FLEXIO1_CTRL |= 2;    // reset Flex module
//  FLEXIO1_CTRL &= 0xfffffffc;  // release reset and leave Flex disabled
//
//  // route IO pins to FlexIO 1
//    // input bit stream
//  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 = 4;     // FLEXIO pin4    Teensy pin 2
//    // Timer 0,1,2 outputs
//  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05 = 4;     // FLEXIO pin5    Teensy pin 3
//  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06 = 4;     // FLEXIO pin6    Teensy pin 4
//  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_07 = 4;     // FLEXIO pin7    Teensy pin 33
//  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_08 = 4;     // FLEXIO pin8    Teensy pin 5
//
//  #define SHIFTER3_IN   4     // Teensy pin 2    match input
//  #define TIMER1_OUT    6     // Teensy pin 4    for debug only
//  #define TIMER2_OUT    8     // Teensy pin 5    for debug only
//  #define TIMER4_OUT    5     // Teensy pin 3    reset for Timer3
//  #define TIMER7_OUT    7     // Teensy pin 33   for debug only
//
//
//  // setup flex timer 1 *****************************************************
//  // this is here so that we can see something happen if we get a trigger from Shifter3
//  // produces a 1MHz clock
//  // it is clocked from from the FLEXIO clock
//  // it is enabled by an output from Shifter3 status flag
//  FLEXIO1_TIMCTL1    =
//           FLEXIO_TIMCTL_TRGSEL( 13 )     |        // Shifter3 status flag =(3 * 4) + 1
//           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
//           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
//           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
//           FLEXIO_TIMCTL_PINSEL(TIMER1_OUT) |      // timer pin 6 (for debug only)
//           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
//           FLEXIO_TIMCTL_TIMOD( 1 );               // dual counter/baud mode
//
//  FLEXIO1_TIMCFG1    =
//           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
//           FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
//           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
//           FLEXIO_TIMCFG_TIMDIS( 2 )      |        // disable timer on timer compare
//           FLEXIO_TIMCFG_TIMENA( 6 )      |        // enable timer on trigger rising
//           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
//           // FLEXIO_TIMCFG_TSTART                 // start bit disabled
//
//  //  TBD
//  //  22 shifts,  divide clock by 40     ((n*2-1)<<8) | (baudrate_divider/2-1))
//  //                                     (22*2-1)<<8  | (40/2-1)
//  //                                         (43)<<8  | (19)
//  //                                          0x2b00  | 0x13
//  FLEXIO1_TIMCMP1    =    0x2b13;
//
//
//  // setup flex timer 2 *****************************************************
//  // this is a 5MHz shift clock for shifter 3
//  // it is clocked from from the FLEXIO clock
//  // it is always enabled
//  FLEXIO1_TIMCTL2    =
//           FLEXIO_TIMCTL_TRGSEL( 0 )      |        // trigger not used
//           //FLEXIO_TIMCTL_TRGPOL         |        // trigger not used
//           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
//           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
//           FLEXIO_TIMCTL_PINSEL(TIMER2_OUT) |      // timer pin 8 (for debug only)
//           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
//           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit timer mode
//
//  FLEXIO1_TIMCFG2    =
//           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
//           FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
//           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
//           FLEXIO_TIMCFG_TIMDIS( 0 )      |        // never disable
//           FLEXIO_TIMCFG_TIMENA( 0 )      |        // timer is always enabled
//           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
//           // FLEXIO_TIMCFG_TSTART                 // start bit disabled
//
//  //  Flex clock = 40MHz, we want 5MHz, so divide by 8
//  //  TIMCMP = divider/2-1 = 8/2-1 = 3
//  FLEXIO1_TIMCMP2    =    0x0003U;
//
//
//  // setup flex timer 3 *****************************************************
//  // this is the shift counter for Shifter3
//  // the shift clock is passed thru from Timer2
//  // it is clocked from from Timer2
//  // it is always enabled
//  // it is reset by Timer4
//  FLEXIO1_TIMCTL3    =
//           FLEXIO_TIMCTL_TRGSEL( 11 )     |        // trigger on Timer2 out (2 * 4)+3
//           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
//           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
//           FLEXIO_TIMCTL_PINCFG( 0 )      |        // timer pin output disabled
//           FLEXIO_TIMCTL_PINSEL(TIMER4_OUT) |      // timer pin 5 (used as Reset input)
//           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
//           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit timer mode
//
//  FLEXIO1_TIMCFG3    =
//           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
//           FLEXIO_TIMCFG_TIMDEC( 3 )      |        // decrement on Trig, shift clock = Trig
//           FLEXIO_TIMCFG_TIMRST( 4 )      |        // reset timer on pin rising edge
//           FLEXIO_TIMCFG_TIMDIS( 0 )      |        // timer never disables
//           FLEXIO_TIMCFG_TIMENA( 0 )      |        // timer is always enabled
//           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
//           // FLEXIO_TIMCFG_TSTART                 // start bit disabled
//
//
//  //  102 shifts                           (n*2-1)
//  //                                     (102*2-1)
//  //                                         (204)
//  FLEXIO1_TIMCMP3    =    203;
//  // this would normally timeout and stop after 102 shifts, however it will be reset by Timer4
//  // after 100 shifts, so the timeout never happens
//
//
//  // setup flex timer 4 *****************************************************
//  // this is an an extra timer that produces a reset to Timer3
//  // it is clocked from from FlexIO
//  // it is always enabled
//  FLEXIO1_TIMCTL4    =
//           FLEXIO_TIMCTL_TRGSEL( 0 )      |        // trigger not used
//           //FLEXIO_TIMCTL_TRGPOL         |        // trigger not used
//           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
//           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
//           FLEXIO_TIMCTL_PINSEL(TIMER4_OUT) |      // timer pin 5 (resets Timer3)
//           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
//           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit counter mode
//
//  FLEXIO1_TIMCFG4    =
//           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
//           FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
//           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
//           FLEXIO_TIMCFG_TIMDIS( 0 )      |        // never disable
//           FLEXIO_TIMCFG_TIMENA( 0 )      |        // timer is always enabled
//           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
//
//  //  50 shifts * 8 flex clocks             (n-1)
//  //                                      (400-1)
//  //                                        (399)
//  //                                       0x018F
//  FLEXIO1_TIMCMP4    =  0x18F;
//  // output toggles after 50 shifts, but reset is on rising edge only, so reset is every 100 shifts
//
//
//  // setup flex timer 7 *****************************************************
//  // for debug only
//  // this basically passes the trigger thru to an IO pin for debug
//  // it is always enabled
//  FLEXIO1_TIMCTL7    =
//           FLEXIO_TIMCTL_TRGSEL( 13 )     |        // Shifter3 status flag =(3 * 4) + 1
//           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
//           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
//           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
//           FLEXIO_TIMCTL_PINSEL(TIMER7_OUT) |      // timer pin 7 (for debug only)
//           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
//           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit timer mode
//
//  FLEXIO1_TIMCFG7    =
//           FLEXIO_TIMCFG_TIMOUT( 1 )      |        // timer output = logic low when enabled, not affcted by reset
//           FLEXIO_TIMCFG_TIMDEC( 3 )      |        // decrement on Trigger input (both edges), Shift clock equals Trigger input.
//           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
//           FLEXIO_TIMCFG_TIMDIS( 0 )      |        // never disable
//           FLEXIO_TIMCFG_TIMENA( 0 )      |        // timer is always enabled
//           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
//           // FLEXIO_TIMCFG_TSTART                 // start bit disabled
//
//  FLEXIO1_TIMCMP7    =  0;    // times out on first clock after trigger
//
//
//  // setup data shifter 3 **************************************************
//  // This shifter is configured in Match Continuous Mode
//  // It watches for the sync pattern at the start of the 1553 transmission
//  // and when found, triggers the data capture
//  FLEXIO1_SHIFTCTL3  =
//           FLEXIO_SHIFTCTL_TIMSEL( 3 )    |        // clocked from timer 3
//           FLEXIO_SHIFTCTL_TIMPOL         |        // shift on neg edge
//           FLEXIO_SHIFTCTL_PINCFG( 0 )    |        // pin output disabled
//           FLEXIO_SHIFTCTL_PINSEL(SHIFTER3_IN) |   // FLEXIO pin 4    Teensy pin 2  (Input data stream)
//           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
//           FLEXIO_SHIFTCTL_SMOD( 5 );              // match continuous mode
//
//  FLEXIO1_SHIFTCFG3  =
//           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
//           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
//           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
//           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled
//
//  FLEXIO1_SHIFTBUF3 =  0xff000000U;    // pattern triggers on a glitch free rising edge
//  //FLEXIO1_SHIFTBUF3 =  0x00ff0000U;    // pattern triggers on a glitch free falling edge
//
//
//
//  // enable FLEXIO3
//  FLEXIO1_CTRL |= 1;    // enable FLEXIO3 module
//
  return( 0 );
}


// disables the FlexIO interrupt, which effectively disables the receiver
void disable(void);

// enables the FlexIO interrupt and flushes buffer to get ready for new RX data
void enable(void);


// Clear buffer and get ready for new RX packet
void FlexIO_1553RX::flush(void)
{
   noInterrupts();
      g_rxDataCount[m_flex_num] = 0;  // clear data buffer write pointer

      // set sync for new packet
      m_flex->SHIFTBUFBIS[3] = FLEX1553_COMMAND_SYNC_PATTERN;
      g_syncType[m_flex_num] = FLEX1553_COMMAND_WORD;
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
   data = (data >> 1) & 0xffff;     // the next 16-bits it the actual data

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
