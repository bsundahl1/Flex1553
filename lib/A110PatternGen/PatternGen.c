#include <Arduino.h>
//#include <Flex1553.h>
#include <PatternGen.h>

#define FLEX01_DEBUG    // brings out optional debug pins
#define USE_8_SHIFTERS  // else use 4
// Flex clock = 480MHz / FLEX01_PREDIVIDER / FLEX01_POSTDIVIDER
#define FLEX01_PREDIVIDER   5   // 1 to 8
#define FLEX01_POSTDIVIDER  6   // 1 to 8
// Shift rate = Flex clock / FLEX01_BAUDDIVIDER
#define FLEX01_BAUDDIVIDER  16    // 2 to 32766, even numbers only

uint32_t patternWord0 = 0;  // needed to re-start shifters


// Configures FlexIO1 as a pattern generator
// @return  always returns zero
int Flex1_PatGen_config( void )
{
  // output pins used by this module
  #define FLEX1_PIN_SHFT0_OUT  4    // Teensy pin 2
  #define FLEX1_PIN_TIM0_OUT   5    // Teensy pin 3
  #define FLEX1_PIN_TIM1_OUT   6    // Teensy pin 4

  // setup flex clock
  // the Flex clock MUST BE CONFIGURED FIRST. Accessing and Flex register without
  // a clock, will hang the code.
  // The default clock speed is 30MHz and I need a multiple of 1.024
  // I can get this by switching to PLL4, which as an output of 786.43MHz
  // and divide that by 48 to get 16.384MHz

  // Note: the clock setup used here is specifically for an i.MXRT106x
  // other devices may be different

  // set clock MUX to route PLL4 to FlexIO1
  CCM_CCGR5 |= CCM_CCGR5_FLEXIO1(CCM_CCGR_OFF);     // disable clock
  //CCM_CDCDR &= ~( CCM_CDCDR_FLEXIO1_CLK_SEL( 3 ) ); // select PLL4

  // set flexIO1 pre-divider to �8
  CCM_CDCDR &= ~( CCM_CDCDR_FLEXIO1_CLK_PRED( 7 ) ); // clear flex clock bits
  CCM_CDCDR |= CCM_CDCDR_FLEXIO1_CLK_PRED( FLEX01_PREDIVIDER-1 );   // set divider

  // set flexIO1 post-divider to �6
  CCM_CDCDR &= ~( CCM_CDCDR_FLEXIO1_CLK_PODF( 7 ) ); // clear flex clock bits
  CCM_CDCDR |= CCM_CDCDR_FLEXIO1_CLK_PODF( FLEX01_POSTDIVIDER-1 );  // set divider

  CCM_CCGR5 |= CCM_CCGR5_FLEXIO1(CCM_CCGR_ON);      // enable clock


  // if the Flex module gets hung up, reconfiguring will not fix it, you will
  // need to reset it. Flex module should be disabled during configuration or
  // else you will likely get "random" output transitions during config.
  // Reset and disable FLEXIO1 (clock MUST be enabled or this will hang)
  FLEXIO1_CTRL |= 2;    // reset Flex module
  FLEXIO1_CTRL &= 0xfffffffc;  // release reset and leave Flex disabled


  // setup pin mux to route IO pins to FlexIO 1
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 = 4;     // FLEXIO pin4    Teensy pin 2
#ifdef FLEX01_DEBUG
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05 = 4;     // FLEXIO pin5    Teensy pin 3
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06 = 4;     // FLEXIO pin6    Teensy pin 4
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_07 = 4;     // FLEXIO pin7    Teensy pin 33
#endif

  // setup flex timer 0 *****************************************************
  // this is a 1.024MHz clock for the shifters
  // it is clocked from from the FLEXIO clock
  // it is always enabled
  FLEXIO1_TIMCTL0    =
           FLEXIO_TIMCTL_TRGSEL( 1 )      |        // shifter 0 status flag =(1 * 4) + 1
           FLEXIO_TIMCTL_TRGPOL           |        // trigger active low
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL( FLEX1_PIN_TIM0_OUT )     |        // timer pin 13, Teensy pin 49 (for debug only)
           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit counter mode

  FLEXIO1_TIMCFG0    =
           FLEXIO_TIMCFG_TIMOUT( 1 )      |        // timer output = logic low when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
           FLEXIO_TIMCFG_TIMDIS( 0 )      |        // never disable
           FLEXIO_TIMCFG_TIMENA( 6 )      |        // enable on trigger rising edge
           FLEXIO_TIMCFG_TSTOP(  0 )       ;       // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled


  //  Flex clock = 16.384MHz, we want 1.024MHz, so divide by 16
  //  TIMCMP = divider/2-1 = 16/2-1 = 7
  FLEXIO1_TIMCMP0    = (FLEX01_BAUDDIVIDER / 2) - 1;   //0x0007;


  // setup flex timer 1 *****************************************************
  // this is a bit counter for the shifters
  // it is clocked from from Timer0
  // it is enabled by Shifter 0 status flag
  // the shifter clock is passed thru from the trigger input
  FLEXIO1_TIMCTL1    =
           FLEXIO_TIMCTL_TRGSEL( FLEX1_PIN_TIM0_OUT * 2 )     |     // trigger on Timer0 (pin 5) =(N *2)
           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL( FLEX1_PIN_TIM1_OUT )     |        // timer pin 13, Teensy pin 49 (for debug only)
           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit counter mode

  FLEXIO1_TIMCFG1    =
           FLEXIO_TIMCFG_TIMOUT( 1 )      |        // timer output = logic low when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 3 )      |        // decrement on trigger, shift clock = trigger
           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
           FLEXIO_TIMCFG_TIMDIS( 1 )      |        // disable on Timer N-1 disable
           FLEXIO_TIMCFG_TIMENA( 1 )      |        // enable on Timer N-1 enable
           FLEXIO_TIMCFG_TSTOP(  0 )       ;       // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled


  //  16 shifts,  divide clock by 16     ( n*2-1)
  //                                     (16*2-1)
  //                                        31
  FLEXIO1_TIMCMP1    =    31;


  // setup data shifter 0 **************************************************
  // note: falling edge polarity was needed to prevent a -half bit at the start
  // and +half bit end of the pattern. The data load appears to always occur on
  // the falling edge of the clock, but it is not clear to me why. Timers decrement
  // on BOTH edges.
  FLEXIO1_SHIFTCTL0  =
           FLEXIO_SHIFTCTL_TIMSEL( 1 )    |        // controlled from timer 1
           FLEXIO_SHIFTCTL_TIMPOL         |        // on falling edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // pin output enabled
           FLEXIO_SHIFTCTL_PINSEL( FLEX1_PIN_SHFT0_OUT )   |        // FLEXIO pin 10    Teensy pin n/a
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 2 );              // transmit mode

  FLEXIO1_SHIFTCFG0  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           FLEXIO_SHIFTCFG_INSRC          |        // from Shifter N+1 output
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled

  // setup data shifter 1 **************************************************
  FLEXIO1_SHIFTCTL1  =
           FLEXIO_SHIFTCTL_TIMSEL( 1 )    |        // controlled from timer 1
           FLEXIO_SHIFTCTL_TIMPOL         |        // on falling edge
           FLEXIO_SHIFTCTL_PINCFG( 0 )    |        // pin output disabled
           FLEXIO_SHIFTCTL_PINSEL( 0 )   |         // pin not used
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 2 );              // transmit mode

  FLEXIO1_SHIFTCFG1  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           FLEXIO_SHIFTCFG_INSRC          |        // from Shifter N+1 output
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled

  // setup data shifter 2 **************************************************
  FLEXIO1_SHIFTCTL2  =
           FLEXIO_SHIFTCTL_TIMSEL( 1 )    |        // controlled from timer 1
           FLEXIO_SHIFTCTL_TIMPOL         |        // on falling edge
           FLEXIO_SHIFTCTL_PINCFG( 0 )    |        // pin output disabled
           FLEXIO_SHIFTCTL_PINSEL( 0 )   |         // pin not used
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 2 );              // transmit mode

  FLEXIO1_SHIFTCFG2  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           FLEXIO_SHIFTCFG_INSRC          |        // from Shifter N+1 output
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled

  // setup data shifter 3 **************************************************
  FLEXIO1_SHIFTCTL3  =
           FLEXIO_SHIFTCTL_TIMSEL( 1 )    |        // controlled from timer 1
           FLEXIO_SHIFTCTL_TIMPOL         |        // on falling edge
           FLEXIO_SHIFTCTL_PINCFG( 0 )    |        // pin output disabled
           FLEXIO_SHIFTCTL_PINSEL( 0 )   |         // pin not used
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 2 );              // transmit mode

  FLEXIO1_SHIFTCFG3  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           FLEXIO_SHIFTCFG_INSRC          |        // from Shifter N+1 output
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled

#ifdef USE_8_SHIFTERS
  // setup data shifter 4 **************************************************
  FLEXIO1_SHIFTCTL4  =
           FLEXIO_SHIFTCTL_TIMSEL( 1 )    |        // controlled from timer 1
           FLEXIO_SHIFTCTL_TIMPOL         |        // on falling edge
           FLEXIO_SHIFTCTL_PINCFG( 0 )    |        // pin output disabled
           FLEXIO_SHIFTCTL_PINSEL( 0 )   |         // pin not used
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 2 );              // transmit mode

  FLEXIO1_SHIFTCFG4  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           FLEXIO_SHIFTCFG_INSRC          |        // from Shifter N+1 output
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled

  // setup data shifter 5 **************************************************
  FLEXIO1_SHIFTCTL5  =
           FLEXIO_SHIFTCTL_TIMSEL( 1 )    |        // controlled from timer 1
           FLEXIO_SHIFTCTL_TIMPOL         |        // on falling edge
           FLEXIO_SHIFTCTL_PINCFG( 0 )    |        // pin output disabled
           FLEXIO_SHIFTCTL_PINSEL( 0 )   |         // pin not used
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 2 );              // transmit mode

  FLEXIO1_SHIFTCFG5  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           FLEXIO_SHIFTCFG_INSRC          |        // from Shifter N+1 output
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled

  // setup data shifter 6 **************************************************
  FLEXIO1_SHIFTCTL6  =
           FLEXIO_SHIFTCTL_TIMSEL( 1 )    |        // controlled from timer 1
           FLEXIO_SHIFTCTL_TIMPOL         |        // on falling edge
           FLEXIO_SHIFTCTL_PINCFG( 0 )    |        // pin output disabled
           FLEXIO_SHIFTCTL_PINSEL( 0 )   |         // pin not used
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 2 );              // transmit mode

  FLEXIO1_SHIFTCFG6  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           FLEXIO_SHIFTCFG_INSRC          |        // from Shifter N+1 output
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled

  // setup data shifter 7 **************************************************
  FLEXIO1_SHIFTCTL7  =
           FLEXIO_SHIFTCTL_TIMSEL( 1 )    |        // controlled from timer 1
           FLEXIO_SHIFTCTL_TIMPOL         |        // on falling edge
           FLEXIO_SHIFTCTL_PINCFG( 0 )    |        // pin output disabled
           FLEXIO_SHIFTCTL_PINSEL( 0 )   |         // pin not used
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 2 );              // transmit mode

  FLEXIO1_SHIFTCFG7  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           FLEXIO_SHIFTCFG_INSRC          |        // from Shifter N+1 output
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled
#endif


  // enable FLEXIO1
  FLEXIO1_CTRL |= 1;    // enable FLEXIO1 module
  //just_configured1 = true;

  return( 0 );
}



int Flex1_PatGen_start( void )
{
  // make sure Flex1 clock is enabled
  if( (CCM_CCGR5 & 0xC) == 0 )
    return( -1 );  // can not write registers if FlexIO1 clock is disabled

  // clear Timer0 disable, so it will run forever
  FLEXIO1_TIMCFG0 = FLEXIO1_TIMCFG0 & ~FLEXIO_TIMCFG_TIMDIS( 7 );

  FLEXIO1_TIMSTAT   = 3;          // reset timer status bits

  // writing to Shifter0 triggers all the shifters
  FLEXIO1_SHIFTBUFBIS0 = patternWord0;

  return( 0 );
}



int Flex1_PatGen_stop( void )
{
  // this just abruptly stops at the end of the curent bit time
  // might be nice if it ran to the end of the pattern
  FLEXIO1_TIMCFG0 = (FLEXIO1_TIMCFG0 & ~FLEXIO_TIMCFG_TIMDIS( 7 )) | FLEXIO_TIMCFG_TIMDIS( 2 );

  return 0;
}



// Calculate the "best" dividers to use in the CCM_CDCDR register
// @param divider    desired total divider (range: 4 to 64)
// @param p_prediv   pointer to calculated pre-divider (range: 1 to 8)
// @param p_postdiv  pointer to calculated post-divider (range: 1 to 8)
int Flex1_PatGen_clock( uint8_t divider, uint8_t *p_prediv, uint8_t *p_postdiv )
{
   int prediv  = 0;
   int postdiv = 0;
   int i, d, r, r2;
   int minr = 100;

   // start from the highest divisor and work down
   for( i=8; i>=1; i-- ) {
      d = divider / i;
      r = divider % i;  // positive remainder
      // because an interger division never gives a high result, we test the next
      // higher divisor here to see if it is closer.
      r2 = abs( divider - ((d+1) * i) );  // negative remainder

      // is it closer?
      if( r > r2 ) {
         r = r2;
         d = d+1;
      }

      // if this divides perfectly, we are done
      if( (d <= 8) && (r == 0) ) {
         prediv  = i;
         postdiv = d;
         break;
      }
      else {  // search for the minimum remainder
         if( (d <= 8) && (r < minr) ) {
            minr = r;
            prediv  = i;
            postdiv = d;
         }
      }
   }

   // if no result was found, set to max divider
   if( prediv == 0 ) {
      prediv  = 8;
      postdiv = 8;
   }

   if( p_prediv  != NULL ) *p_prediv  = prediv;
   if( p_postdiv != NULL ) *p_postdiv = postdiv;
   return( prediv * postdiv );
}



// sets the number of bits that will be shifted out
int Flex1_PatGen_set_bits( int bitCount )
{
  int counter = 0;

  counter = (bitCount * 2) - 1;
  FLEXIO1_TIMCMP1 = counter;
  return( 0 );
}



// Load the shift pattern, in long words
// @param data   pointer to array of long words
// @param size   number of long words in array
int Flex1_PatGen_load(  uint32_t *data, uint8_t size )
{
   if( data == NULL )
      return( -1 );   // null pointer

   if( size < 1 )
      return( -2 );   // no data!

   patternWord0 =  data[0];      // save for re-starts
   if( size > 1 )  FLEXIO1_SHIFTBUFBIS1 = data[1];
   if( size > 2 )  FLEXIO1_SHIFTBUFBIS2 = data[2];
   if( size > 3 )  FLEXIO1_SHIFTBUFBIS3 = data[3];
   if( size > 4 )  FLEXIO1_SHIFTBUFBIS4 = data[4];
   if( size > 5 )  FLEXIO1_SHIFTBUFBIS5 = data[5];
   if( size > 6 )  FLEXIO1_SHIFTBUFBIS6 = data[6];
   if( size > 7 )  FLEXIO1_SHIFTBUFBIS7 = data[7];

   // writing to Shifter0 triggers all the shifters
   FLEXIO1_SHIFTBUFBIS0 = patternWord0;
   return( 0 );
}

