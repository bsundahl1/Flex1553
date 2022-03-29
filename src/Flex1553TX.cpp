#include <Arduino.h>
#include <Flex1553.h>

// 7/12/21  change to a 48MHz clock so we can derive 6MHz from timers
// 9/10/21  reconfigured code to use C++ class instead of standard C
// 9/20/21  created base class for reusable FlexIO functions
// 10/30/21 changed clock speed back to 40MHz
//

// Bug list
//  resolved: startup messages not printing - fixed - need to wait for serial port to connect
//  resolved: FLEX01 pair2 always seems to be enabled - initial conditions not set right
//  resolved: need to finish config_io_pins
//  resolved: fixed bug in state machine table causing output pair 2 to always be low


// Optional build flags:
#define FLEX_PRINT_MESSAGES // uncomment to print startup messages to serial port.
                              // note that to see these messages, you must wait for the serial
                              // port to connect in setup() :
                              //    Serial.begin(115200);
                              //    while(!Serial);

//#define FLEX_DEBUG          // uncomment to bring out debug pins.
                              // You will normally want to turn this off so that you can use
                              // these IO pins for other purposes.



// assign flex data line resources to outputs. some of the are used internally
// others are just brought out for debug. Note that flex FXIO_D[7:0] are reserved
// for the state machine outputs.
#define FLEX1_1553TX_D_SHFT1_OUT 10    // routes shifter to state machine  (not accessable on teensy pin)
#define FLEX1_1553TX_D_SHFT2_OUT 11    // routes shifter to state machine  (not accessable on teensy pin)
#define FLEX1_1553TX_D_TIM0_OUT  8     // for debug only
#define FLEX1_1553TX_D_TIM2_OUT  12    // for debug only
#define FLEX1_1553TX_D_TIM3_OUT  13    // 1MHz clock routed between timers
#define FLEX1_1553TX_D_TIM5_OUT  14    // for debug only
#define FLEX1_1553TX_D_TIM7_OUT  15    // for debug only

#define FLEX2_1553TX_D_SHFT1_OUT 10    // routes shifter to state machine
#define FLEX2_1553TX_D_SHFT2_OUT 11    // routes shifter to state machine
#define FLEX2_1553TX_D_TIM0_OUT  28    // for debug only
#define FLEX2_1553TX_D_TIM2_OUT  12    // for debug only
#define FLEX2_1553TX_D_TIM3_OUT  16    // 1MHz clock routed between timers
#define FLEX2_1553TX_D_TIM5_OUT  17    // for debug only
#define FLEX2_1553TX_D_TIM7_OUT  18    // for debug only

#define FLEX3_1553TX_D_SHFT1_OUT 10    // routes shifter to state machine
#define FLEX3_1553TX_D_SHFT2_OUT 11    // routes shifter to state machine
#define FLEX3_1553TX_D_TIM0_OUT  8     // for debug only
#define FLEX3_1553TX_D_TIM2_OUT  12    // for debug only
#define FLEX3_1553TX_D_TIM3_OUT  13    // 1MHz clock routed between timers
#define FLEX3_1553TX_D_TIM5_OUT  14    // for debug only
#define FLEX3_1553TX_D_TIM7_OUT  15    // for debug only



/***************************************************************************************
*    Start of 1553 TX Class
***************************************************************************************/

// Class constructor
//FlexIO_1553TX::FlexIO_1553TX(uint8_t flex_num, bool pair1, bool pair2, bool pair3, bool pair4)
FlexIO_1553TX::FlexIO_1553TX(uint8_t flex_num, int8_t pinPairA, int8_t pinPairB)
   :FlexIO_Base(flex_num, 40.0)
{
   m_chanToPinPair[FLEX1553_CH_A] = -1;
   m_chanToPinPair[FLEX1553_CH_B] = -1;

   // initialize pin pair structures
   for( int i=0; i<4; i++ ) {
      //m_pair[i].enabled  = false;
      m_pair[i].allowed  = false;
      m_pair[i].f_posPin =  i * 2;     // flex pin number are in order from 0 to 7
      m_pair[i].f_negPin = (i * 2) + 1;
      m_pair[i].t_posPin = getTeensyPin(  i * 2 );    // teensy pin numbers
      m_pair[i].t_negPin = getTeensyPin( (i * 2) + 1 );
   }

   // if a valid pin pair was specified, assign
   if(pinPairA >= FLEX1553_PINPAIR_1 && pinPairA <= FLEX1553_PINPAIR_4) {
      m_pair[pinPairA].allowed = true;
      m_chanToPinPair[FLEX1553_CH_A] = pinPairA;
   }
   if(pinPairB >= FLEX1553_PINPAIR_1 && pinPairB <= FLEX1553_PINPAIR_4) {
      m_pair[pinPairB].allowed = true;
      m_chanToPinPair[FLEX1553_CH_B] = pinPairB;
   }

   // assign FlexIO_Dxx lines based on which FlexIO is used
   // most of these are just used for debug, so it is important
   // that they come out to usable Teensy pins.
   switch(flex_num) {
      case FLEXIO1:
         flexio_d_shift1_out = FLEX1_1553TX_D_SHFT1_OUT;
         flexio_d_shift2_out = FLEX1_1553TX_D_SHFT2_OUT;
         flexio_d_tim0_out   = FLEX1_1553TX_D_TIM0_OUT;
         flexio_d_tim2_out   = FLEX1_1553TX_D_TIM2_OUT;
         flexio_d_tim3_out   = FLEX1_1553TX_D_TIM3_OUT;
         flexio_d_tim5_out   = FLEX1_1553TX_D_TIM5_OUT;
         flexio_d_tim7_out   = FLEX1_1553TX_D_TIM7_OUT;
         m_pair[0].allowed = false;  // not available on flexIO1
         m_pair[1].allowed = false;
         break;

      case FLEXIO2:
         flexio_d_shift1_out = FLEX2_1553TX_D_SHFT1_OUT;
         flexio_d_shift2_out = FLEX2_1553TX_D_SHFT2_OUT;
         flexio_d_tim0_out   = FLEX2_1553TX_D_TIM0_OUT;
         flexio_d_tim2_out   = FLEX2_1553TX_D_TIM2_OUT;
         flexio_d_tim3_out   = FLEX2_1553TX_D_TIM3_OUT;
         flexio_d_tim5_out   = FLEX2_1553TX_D_TIM5_OUT;
         flexio_d_tim7_out   = FLEX2_1553TX_D_TIM7_OUT;
         m_pair[2].allowed = false;  // not available on flexIO2
         m_pair[3].allowed = false;
         break;

      case FLEXIO3:
         flexio_d_shift1_out = FLEX3_1553TX_D_SHFT1_OUT;
         flexio_d_shift2_out = FLEX3_1553TX_D_SHFT2_OUT;
         flexio_d_tim0_out   = FLEX3_1553TX_D_TIM0_OUT;
         flexio_d_tim2_out   = FLEX3_1553TX_D_TIM2_OUT;
         flexio_d_tim3_out   = FLEX3_1553TX_D_TIM3_OUT;
         flexio_d_tim5_out   = FLEX3_1553TX_D_TIM5_OUT;
         flexio_d_tim7_out   = FLEX3_1553TX_D_TIM7_OUT;
         break;
   }

   m_altFlex  = (m_flex_num == 3)? 9 : 4; // FlexIO3 uses ALT9, FLEXIO1 & 2 use ALT4
   m_altGpio  = 5;  // gpio always uses Alt5
   //m_baud_div = 30; // divide from 30MHz to 1MHz
   m_chan = FLEX1553_CH_A;
   m_just_configured = false;
}


bool FlexIO_1553TX::begin( void )
{
   #ifdef FLEX_PRINT_MESSAGES
      if(m_flex_num == 0)
         Serial.println( "Error: invalid flexIO module number" );
      else {
         Serial.print( "Configuring 1553TX on FlexIO" );
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

   #ifdef FLEX_PRINT_MESSAGES  // print out pin pairs used
      for( int i=0; i<4; i++ ) {
         if(m_pair[i].allowed == true) {
            Serial.print( "  Pin pair " );
            Serial.print( i+1 );  // index 0 is pair 1
            Serial.print( ": FXIO_D[" );
            Serial.print( m_pair[i].f_posPin );
            Serial.print( ":" );
            Serial.print( m_pair[i].f_negPin );
            Serial.print( "] = Teensy pins " );
            Serial.print( m_pair[i].t_posPin );
            Serial.print( ", " );
            Serial.print( m_pair[i].t_negPin );
            Serial.println();
         }
      }

      // print out the internal FXIO connections used
      int pin = -1;
      for( int i=0; i<7; i++ ) {
         switch(i) {
            case 0:
               Serial.print( "  SHFT1_OUT = FXIO_D" );
               pin = flexio_d_shift1_out;
               break;
            case 1:
               Serial.print( "  SHFT2_OUT = FXIO_D" );
               pin = flexio_d_shift2_out;
               break;
            case 2:
               Serial.print( "  TIM0_OUT  = FXIO_D" );
               pin = flexio_d_tim0_out;
               break;
            case 3:
               Serial.print( "  TIM2_OUT  = FXIO_D" );
               pin = flexio_d_tim2_out;
               break;
            case 4:
               Serial.print( "  TIM3_OUT  = FXIO_D" );
               pin = flexio_d_tim3_out;
               break;
            case 5:
               Serial.print( "  TIM5_OUT  = FXIO_D" );
               pin = flexio_d_tim5_out;
               break;
            case 6:
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



bool FlexIO_1553TX::config_io_pins(void)
{
   // setup pin mux to route FlexIO to Teensy pins
   // this is the primary output from the 1553 module
   for( int i=0; i<4; i++ ) {
      if(m_pair[i].allowed == true) {
         // For each output pin, set the GPIO pins low, but then set
         // the MUX to FlexIO. We can then disable this channel by changing
         // the MUX back to GPIO
         digitalWrite(m_pair[i].t_posPin, false);  // set the GPIO pins low
         digitalWrite(m_pair[i].t_negPin, false);
         pinMode(m_pair[i].t_posPin, OUTPUT);      // and configure as outputs
         pinMode(m_pair[i].t_negPin, OUTPUT);
         //if(i == m_chanToPinPair[FLEX1553_CH_A]) { // if this is channel A
         //   setPinMux(m_pair[i].t_posPin);         // set the PinMux to FlexIO
         //   setPinMux(m_pair[i].t_negPin);
         //}
      }
      set_channel( FLEX1553_CH_A );  // enable channel A as output
   }

   // The rest of this code routs optional outputs to IO pins, primarily for debug.
   // You will normally want to turn this off so that you can use these IO pins
   // for other purposes. Note that the flexio_d_ variables refer to FlexIO data
   // lines (FXIO_Dxx), and are translated to Teensy pins.
   #ifdef FLEX_DEBUG
      setPinMux( getTeensyPin(flexio_d_shift1_out) );
      setPinMux( getTeensyPin(flexio_d_shift2_out) );
      setPinMux( getTeensyPin(flexio_d_tim0_out)  );
      setPinMux( getTeensyPin(flexio_d_tim2_out)  );
      setPinMux( getTeensyPin(flexio_d_tim3_out)  );
      setPinMux( getTeensyPin(flexio_d_tim5_out)  );
      setPinMux( getTeensyPin(flexio_d_tim7_out)  );
   #endif
   return(true);
}



// Configures FlexIO as a 1553 transmitter
// @return   returns false on error
bool FlexIO_1553TX::config_flex( void )
{
   // Check that the Flex clock is enabled
   // The flex clock should already be configured at this point.
   // The Flex clock MUST BE CONFIGURED before accessing any Flex register
   // or else the code will hang.
   if( !clock_running() )
      return(false);  // abort

   // if the Flex module gets hung up, reconfiguring will not fix it, you will
   // need to reset it. Flex module should be disabled during configuration or
   // else you will likely get "random" output transitions during config.
   // Reset and disable FLEXIO (clock MUST be enabled or this will hang)
   m_flex->CTRL |= 2;    // reset Flex module
   m_flex->CTRL &= 0xfffffffc;  // release reset and leave Flex disabled

   delayMicroseconds(1); // The first register assignment sometimes fails if there is
                         // no delay here. Hot sure why, might still be performing reset?
                         // The minimum delay required might be related to the Flex Clock speed

   // State Machine **************************************************
   // This uses five shifters as a five-state state machine to produce Manchester II
   // biphase encoded data bits
   //  state 0 = outputs disabled
   //  state 4 = first  half of a "1" state
   //  state 6 = second half of a "1" state
   //  state 7 = first  half of a "0" state
   //  state 5 = second half of a "0" state
   // all 5 shifters are set the same, only the state tables are different

   // setup state 0
   m_flex->SHIFTCTL[0]  =
           FLEXIO_SHIFTCTL_TIMSEL( 0 )    |        // controlled from timer 0
           //FLEXIO_SHIFTCTL_TIMPOL       |        // on positive edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // enable output
           FLEXIO_SHIFTCTL_PINSEL( flexio_d_shift1_out )   |
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 6 );              // state mode

   m_flex->SHIFTCFG[0]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // enable FXIO_D[7:4] outputs
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP(  0 )    |        // enable FXIO_D[3:2] outputs
           FLEXIO_SHIFTCFG_SSTART( 0 );            // enable FXIO_D[1:0] outputs

  // setup state 4
   m_flex->SHIFTCTL[4]  =
           FLEXIO_SHIFTCTL_TIMSEL( 0 )    |        // controlled from timer 0
           //FLEXIO_SHIFTCTL_TIMPOL       |        // on positive edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // enable output
           FLEXIO_SHIFTCTL_PINSEL( flexio_d_shift1_out )   |
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 6 );              // state mode

   m_flex->SHIFTCFG[4]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // enable FXIO_D[7:4] outputs
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP(  0 )    |        // enable FXIO_D[3:2] outputs
           FLEXIO_SHIFTCFG_SSTART( 0 );            // enable FXIO_D[1:0] outputs
                                                   //    Teensy pins 2, 3, 4, 33

   // setup state 5
   m_flex->SHIFTCTL[5]  =
           FLEXIO_SHIFTCTL_TIMSEL( 0 )    |        // controlled from timer 0
           //FLEXIO_SHIFTCTL_TIMPOL       |        // on positive edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // enable output
           FLEXIO_SHIFTCTL_PINSEL( flexio_d_shift1_out )   |
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 6 );              // state mode

  m_flex->SHIFTCFG[5]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // enable FXIO_D[7:4] outputs
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP(  0 )    |        // enable FXIO_D[3:2] outputs
           FLEXIO_SHIFTCFG_SSTART( 0 );            // enable FXIO_D[1:0] outputs
                                                   //    Teensy pins 2, 3, 4, 33

   // setup state 6
   m_flex->SHIFTCTL[6]  =
           FLEXIO_SHIFTCTL_TIMSEL( 0 )    |        // controlled from timer 0
           //FLEXIO_SHIFTCTL_TIMPOL       |        // on positive edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // enable output
           FLEXIO_SHIFTCTL_PINSEL( flexio_d_shift1_out )   |
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 6 );              // state mode

   m_flex->SHIFTCFG[6]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // enable FXIO_D[7:4] outputs
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP(  0 )    |        // enable FXIO_D[3:2] outputs
           FLEXIO_SHIFTCFG_SSTART( 0 );            // enable FXIO_D[1:0] outputs
                                                   //    Teensy pins 2, 3, 4, 33

   // setup state 7
   m_flex->SHIFTCTL[7]  =
           FLEXIO_SHIFTCTL_TIMSEL( 0 )    |        // controlled from timer 0
           //FLEXIO_SHIFTCTL_TIMPOL       |        // on positive edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // enable output
           FLEXIO_SHIFTCTL_PINSEL( flexio_d_shift1_out )   |
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 6 );              // state mode

   m_flex->SHIFTCFG[7]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // enable FXIO_D[7:4] outputs
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP(  0 )    |        // enable FXIO_D[3:2] outputs
           FLEXIO_SHIFTCFG_SSTART( 0 );            // enable FXIO_D[1:0] outputs
                                                   //    Teensy pins 2, 3, 4, 33
   // Two output bits from the state machine (a pin pair), are used to pulse
   // two FETs to drive an isolation transformer. The outputs are differential,
   // causing a reversal of the current in the transformer primary for each bit.
   // At the end of the transmission, the state machine goes to the "0" idle
   // state which turns off both transistors.

   // load state lookup table               output  : next state
   m_flex->SHIFTBUF[0] =  0x009E7000U;  //  0000 0000 : 100 111 100 111 000 000 000 000b
   m_flex->SHIFTBUF[4] =  0x55DADDADU;  //  0101 0101 : 110 110 101 101 110 110 101 101b
   m_flex->SHIFTBUF[7] =  0xAAB76B76U;  //  1010 1010 : 101 101 110 110 101 101 110 110b
   m_flex->SHIFTBUF[5] =  0x559E7000U;  //  0101 0101 : 100 111 100 111 000 000 000 000b
   m_flex->SHIFTBUF[6] =  0xAA9E7000U;  //  1010 1010 : 100 111 100 111 000 000 000 000b

   // setup data shifter 1 **************************************************
   // this is the data shifter. writing data to SHIFTBUF1 will trigger the transmit
   // 21 bits are used. The first 3 (LSB) for the Sync, 16 bits of data, one parity,
   // and the last bit (MSB) is used to turn off the transmitter
   m_flex->SHIFTCTL[1]  =
           FLEXIO_SHIFTCTL_TIMSEL( 1 )    |        // controlled from timer 1
           // FLEXIO_SHIFTCTL_TIMPOL      |        // on positive edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // pin output enabled
           FLEXIO_SHIFTCTL_PINSEL( flexio_d_shift1_out )   |        // FLEXIO pin 10    Teensy pin n/a
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 2 );              // transmit mode

   m_flex->SHIFTCFG[1]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled

   // setup data shifter 2 **************************************************
   // This shfiter is used for control bits, which will modify the data
   // Both shifters shift data out at the same time
   // A zero in this shifter causes the same bit postion in the data shifter
   // to be output without transition. A one will cause the data bit to
   // transition: 0 = 0 to 1, 1 = 1 to 0
   m_flex->SHIFTCTL[2]  =
           FLEXIO_SHIFTCTL_TIMSEL( 1 )    |        // controlled from timer 1
           // FLEXIO_SHIFTCTL_TIMPOL      |        // on positive edge
           FLEXIO_SHIFTCTL_PINCFG( 3 )    |        // pin output enabled
           FLEXIO_SHIFTCTL_PINSEL( flexio_d_shift2_out )   |        // FLEXIO pin 11    Teensy pin n/a
           // FLEXIO_SHIFTCTL_PINPOL      |        // active high
           FLEXIO_SHIFTCTL_SMOD( 2 );              // transmit mode

   m_flex->SHIFTCFG[2]  =
           FLEXIO_SHIFTCFG_PWIDTH( 0 )    |        // single bit width
           // FLEXIO_SHIFTCFG_INSRC       |        // from pin
           FLEXIO_SHIFTCFG_SSTOP( 0 )     |        // stop bit disabled
           FLEXIO_SHIFTCFG_SSTART( 0 );            // start bit disabled

   // the control bits are always the same. remember LSB first
   // the shifter will reload the same data each time TIMER1 is triggered
   m_flex->SHIFTBUF[2] =  0x0ffffaU;  //   1111 1111 1111 1111 1010b;
   //m_flex->SHIFTBUF[2] =  0x0000000U;   // for debug: SM should output raw data from shifter 1

   // setup flex timer 0 *****************************************************
   // this is a 2MHz clock to step the state machine
   // it is clocked from from the FLEXIO clock
   // it is always running
   m_flex->TIMCTL[0]    =
           FLEXIO_TIMCTL_TRGSEL( 5 )      |        // not used
           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active low
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL( flexio_d_tim0_out )     |        // timer pin (for debug only)
           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit mode

   m_flex->TIMCFG[0]    =
           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
           FLEXIO_TIMCFG_TIMDIS( 0 )      |        // never disable
           FLEXIO_TIMCFG_TIMENA( 0 )      |        // always enabled
           FLEXIO_TIMCFG_TSTOP(  0 )       ;       // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled


   //   divide clock by 20               (baudrate_divider/2-1))
   //                                         (20/2-1)
   //                                            (9)
   m_flex->TIMCMP[0]  = 9;

   // setup flex timer 1 *****************************************************
   // this is a counter to track the bit count for both shifters
   // it is clocked from Timer3 (1MHz)
   // it is enabled by Shifter 1 status flag via Timer4 and Timer5
   m_flex->TIMCTL[1]    =
           FLEXIO_TIMCTL_TRGSEL( 19 )     |        // Timer4 out (N * 4) + 3
           //FLEXIO_TIMCTL_TRGSEL( 23 )     |        // Timer5 out (N * 4) + 3
           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 0 )      |        // timer pin output disabled
           FLEXIO_TIMCTL_PINSEL( flexio_d_tim3_out )   |   // used as the clock input
           FLEXIO_TIMCTL_PINPOL        |           // timer pin active low
           FLEXIO_TIMCTL_TIMOD( 1 );               // dual counter/baud mode

   m_flex->TIMCFG[1]    =
           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 2 )      |        // decrement on pin input, shift clock = pin input
           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
           FLEXIO_TIMCFG_TIMDIS( 2 )      |        // disable timer on timer compare
           FLEXIO_TIMCFG_TIMENA( 2 )      |        // enable timer on (inverted) trigger high
                  // this is needed to prevent a timing synchronization fault that
                  // occurs occasionally if the trigger occurs too close to the clock edge
           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled

   //  We are shifting 20 bits of data
   //  20 shifts,  divide clock by 1      ((n*2-1)<<8) | (baudrate_divider/2-1))
   //                                     (20*2-1)<<8  | (1/2-1)
   //                                         (39)<<8  | (0)
   //                                          0x2700  | 0x0
   m_flex->TIMCMP[1]    =    0x2700;

   // setup flex timer 2 *****************************************************
   // This produced the Enable input to the state machine which
   // will disable the output at the end of the transmission.
   // This timer produces a .525us delay, which is reset after
   // each edge of Timer1 output (500us edges). Thus, the timer
   // output will stay high until .525us after Timer1 stops.
   // clocked from flex clock
   // enabled by Timer1 enable
   // reset by Timer 1 output
   // disabled by compare (timeout)
   m_flex->TIMCTL[2]    =
           FLEXIO_TIMCTL_TRGSEL( 7 )      |        // triggered by Timer 1 output =(1 * 4) + 3
           FLEXIO_TIMCTL_TRGPOL           |        // trigger active low
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL( flexio_d_tim2_out )     |        // timer pin 12 (wired to state machine)
           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit timer

   m_flex->TIMCFG[2]    =
           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 0 )      |        // Decrement on Flex clock
           FLEXIO_TIMCFG_TIMRST( 7 )      |        // reset on trigger, both edges
           FLEXIO_TIMCFG_TIMDIS( 2 )      |        // Timer disabled on timer compare
           FLEXIO_TIMCFG_TIMENA( 1 )      |        // Timer enabled on Timer N-1 enable
           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled

   // divide Flex clock to get 0.50us
   // time = (21 + 1)/40  = .550us
   m_flex->TIMCMP[2]    =    21;


   // setup flex timer 3 *****************************************************
   // 1MHz clock for the shifters
   // This is free running and is always running
   // it is clocked from from the Flex clock
   // and is synchronized to Timer0 on the rising edge, simply because
   // both clocks are started at the same time (at the end of config)
   m_flex->TIMCTL[3]    =
           FLEXIO_TIMCTL_TRGSEL( 3 )      |        // Triggered by Timer0 =(0 * 4) + 3
           FLEXIO_TIMCTL_TRGPOL           |        // trigger active low
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL( flexio_d_tim3_out )     |        // timer pin (for debug only)
           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit mode

   m_flex->TIMCFG[3]    =
           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on flex clock
           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
           FLEXIO_TIMCFG_TIMDIS( 0 )      |        // never disable
           FLEXIO_TIMCFG_TIMENA( 0 )      |        // always enabled
           FLEXIO_TIMCFG_TSTOP(  0 )       ;       // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled


   //   divide clock by 40               (baudrate_divider/2-1))
   //                                         (40/2-1)
   //                                            (19)
   m_flex->TIMCMP[3]    =    19;


   // setup flex timer 4 *****************************************************
   // This is a hack to try to sync the shifter flag to the clocks
   // This design relies on Timer0 and Timer3 to always be running, keeping the data bits on the same
   // bit clock from one word to the next. But a transmit from software is asychronous to this clock,
   // and could be triggerd at any time, on any edge of the 48MHz FlexIO clock. If the trigger occurs
   // at a critical time just before the data shifts, the timers and shifters can get out of sync
   // with each other, and everything falls apart.
   // This timer synchronizes the trigger to the falling edge of Timer3 (1MHz clock), an edge that
   // should be safe for triggering.
   // When SHIFTER1 flag goes high (caused by software writing data to the shifer) this timer will
   // enable Timer4 on the next falling edge of TIMER3 (1MHz clock). It disables as soon as the SHIFTER1
   // flag goes low, which is caused by the data being loaded into the shifter.
   m_flex->TIMCTL[4]    =
           FLEXIO_TIMCTL_TRGSEL( 5 )      |        // shifter 1 status flag =(1 * 4) + 1
           FLEXIO_TIMCTL_TRGPOL           |        // trigger active low
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 0 )      |        // timer pin is an input
           FLEXIO_TIMCTL_PINSEL( flexio_d_tim3_out )     |        // pin input from 1MHz clock
           FLEXIO_TIMCTL_PINPOL           |        // timer pin active low
           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit mode

   m_flex->TIMCFG[4]    =
           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
           //FLEXIO_TIMCFG_TIMDEC( 2 )      |        // decrement on pin input, shift clock = pin input
           //FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
           FLEXIO_TIMCFG_TIMDEC( 1 )      |        // decrement on trigger, shift clock = timer output
           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
           FLEXIO_TIMCFG_TIMDIS( 6 )      |        // disable timer on trigger falling edge
           //FLEXIO_TIMCFG_TIMENA( 3 )      |        // enable timer on (inverted) trigger high & pin high
           FLEXIO_TIMCFG_TIMENA( 5 )      |        // enable timer on pin rising edge AND trigger high
           FLEXIO_TIMCFG_TSTOP(  0 )       ;       // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled

   //m_flex->TIMCMP[4]    =   0x1;
   m_flex->TIMCMP[4]    =   0xffff;


   // setup flex timer 5 *****************************************************
   // this is part of a hack to try to sync the shifter flag to the clocks
   // It is no longer needed - a better fix was found
   // This basically transfers the Enable from TIMER4 to a pin output
   // It can still be used to make the TIMER4 output visible on a Teensy pin for debug
   m_flex->TIMCTL[5]    =
           //FLEXIO_TIMCTL_TRGSEL( 36 )     |        // Pin 18 (2MHz clock) =(pin * 2)
           FLEXIO_TIMCTL_TRGSEL( 0 )      |        // not used
           FLEXIO_TIMCTL_TRGPOL           |        // trigger active low
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL( flexio_d_tim5_out )     |        // timer pin 19 (for debug only)
           FLEXIO_TIMCTL_PINPOL           |        // timer pin active low
           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit mode

   m_flex->TIMCFG[5]    =
           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic low when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 1 )      |        // decrement on trigger, shift clock = timer output
           //FLEXIO_TIMCFG_TIMDEC( 0 )      |        // decrement on FlexIO clock, shift clock = timer output
           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
           FLEXIO_TIMCFG_TIMDIS( 1 )      |        // disable timer on timer N-1 disable
           //FLEXIO_TIMCFG_TIMENA( 3 )      |        // enable timer on (inverted) trigger high & pin high
           FLEXIO_TIMCFG_TIMENA( 1 )      |        // enable timer on timer N-1 enable
           FLEXIO_TIMCFG_TSTOP(  0 )       ;       // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled

   m_flex->TIMCMP[5]    =   0xffff;


     // setup flex timer 7 *****************************************************
   // for debug only
   // this just passes the trigger thru to an IO pin for debug
   // it is always enabled
   m_flex->TIMCTL[7]    =
           FLEXIO_TIMCTL_TRGSEL( 5 )     |        // Shifter1 status flag =(1 * 4) + 1
           //FLEXIO_TIMCTL_TRGSEL( 19 )     |        // Timer4 output =(4 * 4) + 3
           //FLEXIO_TIMCTL_TRGPOL         |        // trigger active high
           FLEXIO_TIMCTL_TRGSRC           |        // internal trigger
           FLEXIO_TIMCTL_PINCFG( 3 )      |        // timer pin output enabled
           FLEXIO_TIMCTL_PINSEL( flexio_d_tim7_out )      |        // timer pin 9 (for debug only)
           // FLEXIO_TIMCTL_PINPOL        |        // timer pin active high
           FLEXIO_TIMCTL_TIMOD( 3 );               // 16-bit timer mode

   m_flex->TIMCFG[7]    =
           FLEXIO_TIMCFG_TIMOUT( 0 )      |        // timer output = logic high when enabled, not affcted by reset
           FLEXIO_TIMCFG_TIMDEC( 3 )      |        // decrement on Trigger input (both edges), Shift clock equals Trigger input.
           FLEXIO_TIMCFG_TIMRST( 0 )      |        // dont reset timer
           FLEXIO_TIMCFG_TIMDIS( 0 )      |        // never disable
           FLEXIO_TIMCFG_TIMENA( 0 )      |        // timer is always enabled
           FLEXIO_TIMCFG_TSTOP(  0 )    ;          // stop bit disabled
           // FLEXIO_TIMCFG_TSTART                 // start bit disabled


   // enable FLEXIO1
   m_flex->CTRL |= 1;    // enable FLEXIO1 module
   m_just_configured = true;

   return( true );
}





// Sends one command word on bus, using FlexIO1
// If transmitter is busy, this will wait for it to become avilable.
// @param  rdaddress:   address of the device to send to
// @param  subaddress:  register number to send data to
// @param  wordcount:   number of data bytes to follow (1 to 32)
// @return error code:  0=success, -1=Flex clock disabled, -2=timeout
int FlexIO_1553TX::send_command( byte rtaddress, byte subaddress, byte wordcount )
{
   uint16_t data;

   data = (rtaddress & 0x1f) << 11 | (subaddress & 0x1f) << 5 | (wordcount & 0x1f);
   return( send( FLEX1553_COMMAND_WORD, data) );
}



// Send one data word on bus, using FlexIO1
// If transmitter is busy, this will wait for it to become avilable.
// @param  data:   16-bit data
// @return error code: 0=success, -1=Flex clock disabled, -2=timeout
int FlexIO_1553TX::send_data( uint16_t data )
{
   return( send( FLEX1553_DATA_WORD, data) );
}



// Generic send command
// Sends one word on bus, using FlexIO1
// If transmitter is busy, this will wait for it to become avilable.
// @param  sync:   0=data sync, 1=command or status sync
// @param  data:   16-bit data
// @return error code: 0=success, -1=Flex clock disabled, -2=timeout
int FlexIO_1553TX::send( uint8_t sync, uint16_t data )
{
   uint32_t shiftData;
   uint32_t time;
   int status = 0;

   // make sure Flex1 clock is enabled
    if( !clock_running() )
       return( -1 );  // code will hang if Flex clock not enabled

   // write output data to shifter
   // The first three bits form a SYNC pulse, which behave differently than the
   // data bits. The FlexIO state machine will treat these bit positions special
   // due to the control bits loaded into SHIFTBUF2, do not use any other codes here.
   if( sync == FLEX1553_COMMAND_WORD )
      shiftData =  0xC0000000U; // 110b
   else
      shiftData =  0x20000000U; // 001b

   shiftData = shiftData | ((uint32_t)data << 13) | ((uint32_t)parity(data) << 12);

   // wait for transmitter avaliable
   time = micros();
   while(1) {
      if( transmitter_busy() == 0 )
         break;
      if( (micros() - time) > 100 ) { // or timeout
         status = -2;  // timeout
         break;
      }
   }

   //shiftData = FLEXIO3_SHIFTBUF3; // this is a dummy read to clear the RX shifter error flag

   //  If all is well, send the data
   if( status == 0 ) {
      // the shifter sends LSB first, using BIS function reverses the bit
      // order of the data, so effectively, we are sending MSB first.
      m_flex->SHIFTBUFBIS[1] = shiftData; // start transimision
      m_flex->TIMSTAT = 7;    // reset timer status bits, these are used to determine when
                              // transmission is complete
      m_just_configured = false;
   }

   return( status );
}



int FlexIO_1553TX::transmitter_busy( void )
{
   int flags;
   int status;

   // the usual "tranmitter empty" flag does not seem to work with FlexIO
   // writing the second word before the transmitter is done will cause it
   // to stop working. The code does not hang, but the Flex module will
   // no longer trigger, until it has been reset.
   // There seem to be two valid flag states with this specific setup
   // where flags = (FLEXIO2_TIMSTAT << 8) | FLEXIO2_SHIFTSTAT
   //    each bit is a flag for one timer or shifter
   // 0x0003 = immediatly after configuration, shifters 0 and 1 will show empty
   //          shifter 2 was loaded by the configuration, so it shows full
   //          timers have not been triggered, so their status is 0
   //          BUT, this condition can also occur on subsiquent transmissions
   //          durring the shift, tricking the code into thinking it is done.
   // 0x0707 = after data has been sent and transmission is complete
   //          shifters 0, 1 and 2 all show empty
   //          timers 0, 1, and 2 show a timeout
   //          BUT, the processor is so crazy fast, it can check this status again
   //          and load a second word into the transmitter, before the HARDWARE
   //          has time to clear the status flags! So the timer flags must be
   //          cleared by software after writing the FLEXIO2_SHIFTBUF1 reg
   // Any other status indicates that the FlexIO transmit is not done

   // make sure Flex1 clock is enabled
    if( !clock_running() )
       return( -1 );  // will hang if Flex clock not enabled

   //flags = (FLEXIO1_TIMSTAT << 8) | FLEXIO1_SHIFTSTAT;
   flags = (m_flex->TIMSTAT << 8) | m_flex->SHIFTSTAT;
   //if( flags == 0x0707 ) // normal flags
   if( flags & 2 ) // data shifter empty
      status = 0;
   else if( (flags == 3) && m_just_configured ) // only used by the first write
      status = 0;
   else
      status = 1;  // not ready

   return( status );
}


// get status flags for 8 timers and 8 shifters in FlexIO1
// @return  8 timer flags : 8 shifter flags (one byte for each)
unsigned long FlexIO_1553TX::get_status( void )
{
   unsigned long flags;

   // make sure Flex1 clock is enabled
    if( !clock_running() )
       return( 0 );  // will hang if Flex clock not enabled

   //flags = (FLEXIO1_TIMSTAT << 8) | FLEXIO1_SHIFTSTAT; // will hang if Flex clock not enabled
   flags = (m_flex->SHIFTERR << 16) | (m_flex->SHIFTSTAT << 8) | m_flex->TIMSTAT;
   m_flex->SHIFTERR  = 0xff;  // clear flags
   m_flex->SHIFTSTAT = 0xff;
   m_flex->TIMSTAT   = 0xff;
   return( flags );
}



// Controls the pin MUX to enable or disable the 1553 outputs.
// If transmitter is busy, this will wait for it to empty before changing the outputs.
// @param  ch:    0=all channels off
//                1=transmit on channel A
//                2=transmit on channel B
//                4=transimt on both channels
//                any other value will disable all channels
// @return error code: 0=success, -1=Flex clock disabled, -2=timeout, -3=illegal configuration
int FlexIO_1553TX::set_channel( int8_t ch )
{
   uint32_t time;
   bool    chA_on, chB_on;
   int8_t  chPairA = m_chanToPinPair[FLEX1553_CH_A];  // this is an index into the m_pair[] array of structures
   int8_t  chPairB = m_chanToPinPair[FLEX1553_CH_B];

   // this function changes the pin MUX to enable or disable the
   // outputs from the 1553 state machine. When enabled, these two
   // differential outputs have identical information on them, and
   // when disabled, they have zeros from the GPIO drivers.
   // Their only purpose is to switch between two different TX
   // hardware circuits.


   //if( ch == m_chan )
   //   return( 0 );   // no change needed

   switch(ch)
   {
      case 0:
         chA_on = false;
         chB_on = false;
         break;
      case FLEX1553_CH_A:
         chA_on = true;
         chB_on = false;
         break;
      case FLEX1553_CH_B:
         chA_on = false;
         chB_on = true;
         if(chPairB == -1)
            return -3; // channel B has not been configured
         break;
      case FLEX1553_CH_ALL:
         chA_on = true;
         chB_on = true;
         break;
      default:
         return -3;  // invalid input
   }

   // sanity check
   if(chPairA == -1)
      return( -3 );  // this should never happen
   if(chPairB == -1)
      chB_on = false;

    //if( ch != FLEX1553_CH_A && ch != FLEX1553_CH_B && ch != FLEX1553_CH_ALL )
    //  return -3;  // invalid input

   // double check that the pin pair is allowed
   if(chA_on) {
      if( m_pair[chPairA].allowed == false )
         return -3;  // this should never happen
   }
   if(chB_on) {
      if( m_pair[chPairB].allowed == false )
         return -3;  // trying to set a channel disallowed in constructor
   }

   // make sure Flex1 clock is enabled
    if( !clock_running() )
       return( -1 );

   // wait for transmitter to empty before changing
   time = micros();
   while(1) {
      if( transmitter_busy() == 0 )
         break;
      if( (micros() - time) > 100 ) { // or timeout
         return( -2 );  // timeout
         break;
      }
   }

   //for( int i=0; i<4; i++ ) {
   //   if( m_pair[i].allowed ) {
   //      if( ch == i || ch == 4 ) {
   //         // turn this pair on
   //         setPinMux( m_pair[i].t_posPin, m_altFlex);
   //         setPinMux( m_pair[i].t_negPin, m_altFlex);
   //      }
   //      else {
   //         // turn this pair off
   //         setPinMux( m_pair[i].t_posPin, m_altGpio);
   //         setPinMux( m_pair[i].t_negPin, m_altGpio);
   //      }
   //   }
   //}

   if(chA_on) {
      // turn this pair on
      setPinMux(m_pair[chPairA].t_posPin, m_altFlex);
      setPinMux(m_pair[chPairA].t_negPin, m_altFlex);
   }
   else {
      // turn this pair off
      setPinMux(m_pair[chPairA].t_posPin, m_altGpio);
      setPinMux(m_pair[chPairA].t_negPin, m_altGpio);
   }

   if(chB_on) {
      setPinMux(m_pair[chPairB].t_posPin, m_altFlex);
      setPinMux(m_pair[chPairB].t_negPin, m_altFlex);
   }
   else {
      setPinMux(m_pair[chPairB].t_posPin, m_altGpio);
      setPinMux(m_pair[chPairB].t_negPin, m_altGpio);
   }

   m_chan = ch;
   return( 0 );
}



uint8_t FlexIO_1553TX::parity( uint32_t data )
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


