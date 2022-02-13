#include <Arduino.h>
#include <FlexIOBase.h>


// Constructor
//FlexIO_Base::FlexIO_Base(uint8_t flex_num, uint8_t pll_divider)
FlexIO_Base::FlexIO_Base(uint8_t flex_num, float flex_clock_freq)
{
   m_flex_num = flex_num;
   //m_pll_divider = pll_divider;
   m_pll_divider = (uint8_t)(480.0 / flex_clock_freq + 0.5);

   // setup the pointer to the FlexIO hardware
   switch( flex_num ) {
      case 1:  m_flex = &IMXRT_FLEXIO1_S; break;
      case 2:  m_flex = &IMXRT_FLEXIO2_S; break;
      case 3:  m_flex = &IMXRT_FLEXIO3_S; break;
      default: m_flex = NULL; flex_num = 0;
   }
}

// configure the FlexIO hardware and check for configuration errors
bool FlexIO_Base::begin( void )
{
   uint8_t prediv, postdiv;

   if(m_flex_num == 0)
      return false;  // invalid flexIO module number

   // the Flex clock MUST BE CONFIGURED FIRST. Accessing and Flex register without
   // a clock, will hang the code.
   //if( config_clock() == false )  // configures the CCM clock control module
   //   return false;

   // calculate dividers
   calc_pll_clock_div( m_pll_divider, &prediv, &postdiv );

   // write to CCM hardware
   if( !config_clock_div(prediv, postdiv) )
      return false;  // abort on error

   return true;
}

// clear the FlexIO configuration
void FlexIO_Base::end( void )
{
   m_flex->CTRL |= 2;    // reset Flex module
   m_flex->CTRL &= 0xfffffffc;  // release reset and leave Flex disabled
}

// this enables the flex module
// in most cases, this should be overridden to resume operation from a disable()
bool FlexIO_Base::enable( void )
{
   m_flex->CTRL |= 1;    // enable FLEXIO module
   return true;
}

// this disables the flex module
// in most cases, this should be overridden to stop or pause operation of the flex circuit
bool FlexIO_Base::disable( void )
{
   m_flex->CTRL &= 0xfffffffc;  // disable the entire Flex module
   return true;
}



bool FlexIO_Base::setPinMux( uint8_t teensyPin )
{
   uint8_t pin_mux_alt = (m_flex_num == 3)? 9 : 4; // FlexIO3 uses ALT9, FLEXIO1 & 2 use ALT4
   return( setPinMux( teensyPin, pin_mux_alt ) );
}



// general utility to configure pin mux
// @param teensyPin   pin number
// @param alt         pin mux selector
bool FlexIO_Base::setPinMux( uint8_t teensyPin, uint8_t alt)
{
   if( teensyPin < 0 || alt < 0 )
      return(false);

   // this seems really crude, need a more efficient way to do this

   switch( teensyPin ) {
      case 0 : CORE_PIN0_CONFIG  = alt; break;
      case 1 : CORE_PIN1_CONFIG  = alt; break;
      case 2 : CORE_PIN2_CONFIG  = alt; break;
      case 3 : CORE_PIN3_CONFIG  = alt; break;
      case 4 : CORE_PIN4_CONFIG  = alt; break;
      case 5 : CORE_PIN5_CONFIG  = alt; break;
      case 6 : CORE_PIN6_CONFIG  = alt; break;
      case 7 : CORE_PIN7_CONFIG  = alt; break;
      case 8 : CORE_PIN8_CONFIG  = alt; break;
      case 9 : CORE_PIN9_CONFIG  = alt; break;
      case 10: CORE_PIN10_CONFIG = alt; break;
      case 11: CORE_PIN11_CONFIG = alt; break;
      case 12: CORE_PIN12_CONFIG = alt; break;
      case 13: CORE_PIN13_CONFIG = alt; break;
      case 14: CORE_PIN14_CONFIG = alt; break;
      case 15: CORE_PIN15_CONFIG = alt; break;
      case 16: CORE_PIN16_CONFIG = alt; break;
      case 17: CORE_PIN17_CONFIG = alt; break;
      case 18: CORE_PIN18_CONFIG = alt; break;
      case 19: CORE_PIN19_CONFIG = alt; break;
      case 20: CORE_PIN20_CONFIG = alt; break;
      case 21: CORE_PIN21_CONFIG = alt; break;
      case 22: CORE_PIN22_CONFIG = alt; break;
      case 23: CORE_PIN23_CONFIG = alt; break;
      case 24: CORE_PIN24_CONFIG = alt; break;
      case 25: CORE_PIN25_CONFIG = alt; break;
      case 26: CORE_PIN26_CONFIG = alt; break;
      case 27: CORE_PIN27_CONFIG = alt; break;
      case 28: CORE_PIN28_CONFIG = alt; break;
      case 29: CORE_PIN29_CONFIG = alt; break;
      case 30: CORE_PIN30_CONFIG = alt; break;
      case 31: CORE_PIN31_CONFIG = alt; break;
      case 32: CORE_PIN32_CONFIG = alt; break;
      case 33: CORE_PIN33_CONFIG = alt; break;
      case 34: CORE_PIN34_CONFIG = alt; break;
      case 35: CORE_PIN35_CONFIG = alt; break;
      case 36: CORE_PIN36_CONFIG = alt; break;
      case 37: CORE_PIN37_CONFIG = alt; break;
      case 38: CORE_PIN38_CONFIG = alt; break;
      case 39: CORE_PIN39_CONFIG = alt; break;
    #if CORE_NUM_TOTAL_PINS > 40
      case 40: CORE_PIN40_CONFIG = alt; break;
      case 41: CORE_PIN41_CONFIG = alt; break;
      case 42: CORE_PIN42_CONFIG = alt; break;
      case 43: CORE_PIN43_CONFIG = alt; break;
      case 44: CORE_PIN44_CONFIG = alt; break;
      case 45: CORE_PIN45_CONFIG = alt; break;
    #endif
    #if CORE_NUM_TOTAL_PINS > 46
      case 46: CORE_PIN46_CONFIG = alt; break;
      case 47: CORE_PIN47_CONFIG = alt; break;
      case 48: CORE_PIN48_CONFIG = alt; break;
      case 49: CORE_PIN49_CONFIG = alt; break;
      case 50: CORE_PIN50_CONFIG = alt; break;
      case 51: CORE_PIN51_CONFIG = alt; break;
      case 52: CORE_PIN52_CONFIG = alt; break;
      case 53: CORE_PIN53_CONFIG = alt; break;
      case 54: CORE_PIN54_CONFIG = alt; break;
    #endif
      default: return(false);
   }
   return(true);
}



// convert FlexIO pin number to Teensy pin number
// @param pin:             pin number to look up
// @param reverseLookup:   false = FlexIO to Teensy pin
//                         true  = Teensy to FlexIO pin
// returns -1 on error
int8_t FlexIO_Base::getTeensyPin( int8_t pin, bool reverseLookup )
{
   int8_t *teensyPinLookup = NULL;
   int8_t numFlexPins = 0;

   // tables are indexed by the FlexIO pin, and contain the associated Teensy pin
                        //  flex pin:{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15}
   int8_t flex1TeensyPinLookup[16] = {-1,-1,-1,-1, 2, 3, 4,33, 5,-1,-1,-1,52,49,50,54};

                        //  flex pin:{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15}
   int8_t flex2TeensyPinLookup[32] = {10,12,11,13,-1,-1,-1,-1,-1,-1, 6, 9,32,-1,-1,-1,
                                       8, 7,36,37,-1,-1,-1,-1,-1,-1,-1,-1,35,34,-1,-1};
                        //  flex pin:{16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}

                        //  flex pin:{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15}
   int8_t flex3TeensyPinLookup[32] = {19,18,14,15,40,41,17,16,22,23,20,21,38,39,26,27,
                                       8, 7,36,37,-1,-1,-1,-1,-1,-1,-1,-1,35,34,-1,-1};
                        //  flex pin:{16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}

   switch( m_flex_num )
   {
      case FLEXIO1:
         numFlexPins = 16;
         teensyPinLookup = flex1TeensyPinLookup;
         break;

      case FLEXIO2:
         numFlexPins = 32;
         teensyPinLookup = flex2TeensyPinLookup;
         break;

      case FLEXIO3:
         numFlexPins = 32;
         teensyPinLookup = flex3TeensyPinLookup;
         break;

      default:
         return( -1 ); // invalid flex_num
   }

   if( reverseLookup == false ) {
      // FlexIO to Teensy pin
      if( pin >= numFlexPins || pin < 0 )
         return( -1 ); // out of range
      return teensyPinLookup[pin];
   }

   // else use reverse lookup: Teensy to FlexIO pin
   for( int i=0; i<numFlexPins; i++ ) {
      if( teensyPinLookup[i] == pin )
         return( i );
   }
   return( -1 ); // pin not found
}


// get the PARAM register for the FLEXIO module
// @param   param_type:  0=SHIFTERs, 1=TIMERs, 2=PINs, 3=TRIGGERs
uint8_t FlexIO_Base::get_params(uint8_t param_type)
{
   switch( param_type )
   {
      // the PARAM register contains the number of FLEXIO resources avilable in
      // the specified module. Resouces are arranged as 4 bytes
      // TRIGGERs : PINs : TIMERs : SHIFTERs
      // TRIGGER refers to the number of EXTERNAL triggers, have not found any
      // documentation on this yet
      case FLEXIO_SHIFTERS:
         return(m_flex->PARAM & 0xff);
         break;
      case FLEXIO_TIMERS:
         return((m_flex->PARAM >> 8) & 0xff);
         break;
      case FLEXIO_PINS:
         return((m_flex->PARAM >> 16) & 0xff);
         break;
      case FLEXIO_TRIGGERS:
         return((m_flex->PARAM >> 24) & 0xff);
         break;
   }

   return( 0 );
}


// check if the flex clock is running
bool FlexIO_Base::clock_running(void)
{
   switch( m_flex_num )
   {
      // this comes from the CCM clock module, not the FlexIO module
      // this is processor specific and is only valid for i.MXRT106x
      case FLEXIO1:
         if( (CCM_CCGR5 & 0xC) != 0 )  // make sure Flex1 clock is enabled
            return(true);
          break;
      case FLEXIO2:
      case FLEXIO3:
         if( (CCM_CCGR3 & 3) != 0 )  // make sure Flex2/3 clock is enabled
            return(true);
         break;
   }
   return(false);
}


// returns the PIN register, which is the current state of all the FlexIO pins FXIO_Dxx
uint32_t FlexIO_Base::get_pin_states(void)
{
   if( clock_running() )
      return( m_flex->PIN );
   else
      return(0);
}


// Calculate the "best" dividers to use in the CCM_CDCDR register
// @param divider    desired total divider (range: 4 to 64)
// @param p_prediv   pointer to calculated pre-divider (range: 1 to 8)
// @param p_postdiv  pointer to calculated post-divider (range: 1 to 8)
// @param retrun     final divider ratio
int FlexIO_Base::calc_pll_clock_div( uint8_t divider, uint8_t *p_prediv, uint8_t *p_postdiv )
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


// Set the flex clock dividers
// Note that flex2 and flex3 share the same clock
// PLL root clock is 480MHz, minimum total divider is 4
// @param p_prediv   pre-divider (range: 1 to 8)
// @param p_postdiv  post-divider (range: 1 to 8)
// @param retrun     true if no errors
bool FlexIO_Base::config_clock_div( uint8_t prediv, uint8_t postdiv )
{
   // the Flex clock MUST BE CONFIGURED FIRST. Accessing and Flex register without
   // a clock, will hang the code.

   if( prediv > 8 || postdiv > 8 )
      return(false);
   if( (prediv * postdiv) < 4 )
      return(false);

   // this sets registers in the CCM clock control module, not the FlexIO module
   // this is processor specific and is only valid for i.MXRT106x
   switch( m_flex_num )
   {
      case FLEXIO1:
         CCM_CCGR5 &= ~( CCM_CCGR5_FLEXIO1(CCM_CCGR_ON) );        // disable clock

         // set flexIO1 pre-divider
         CCM_CDCDR &= ~( CCM_CDCDR_FLEXIO1_CLK_PRED( 7 ) );       // clear flex clock bits
         CCM_CDCDR |= CCM_CDCDR_FLEXIO1_CLK_PRED( prediv - 1 );   // set divider

         // set flexIO1 post-divider
         CCM_CDCDR &= ~( CCM_CDCDR_FLEXIO1_CLK_PODF( 7 ) );       // clear flex clock bits
         CCM_CDCDR |= CCM_CDCDR_FLEXIO1_CLK_PODF( postdiv - 1 );  // set divider

         CCM_CCGR5 |= CCM_CCGR5_FLEXIO1(CCM_CCGR_ON);             // enable clock
         return(true);
         break;

      case FLEXIO2:
      case FLEXIO3:
         // Note that flex2 and flex3 share the same clock
         CCM_CCGR3 &= ~( CCM_CCGR3_FLEXIO2(CCM_CCGR_ON) );        // disable clock

         CCM_CS1CDR &= ~( CCM_CS1CDR_FLEXIO2_CLK_PRED(7) );       // clear flex clock bits
         CCM_CS1CDR |= CCM_CS1CDR_FLEXIO2_CLK_PRED(prediv - 1);   // set divider

         // note: FlexIO2 and FlexIO3 share the same clock
         CCM_CS1CDR &= ~( CCM_CS1CDR_FLEXIO2_CLK_PODF(7) );       // clear flex clock bits
         CCM_CS1CDR |= CCM_CS1CDR_FLEXIO2_CLK_PODF(postdiv - 1);  // set divider

         CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);             // enable clock
         return(true);
       break;
   }

   return(false);
}



// returns divider values for debug
void FlexIO_Base::get_clock_divider( uint8_t *target_div, uint8_t *pre_div, uint8_t *post_div )
{
   if(target_div != NULL)
      *target_div = m_pll_divider;

   if( pre_div != NULL && post_div != NULL ) {
   //   calc_pll_clock_div( m_pll_divider, pre_div, post_div );

      switch( m_flex_num )
      {
         case FLEXIO1:
            *pre_div  = ((CCM_CDCDR & CCM_CDCDR_FLEXIO1_CLK_PRED(7) ) >> 12 ) + 1;
            *post_div = ((CCM_CDCDR & CCM_CDCDR_FLEXIO1_CLK_PODF(7) ) >>  9 ) + 1;
            break;

         case FLEXIO2:
         case FLEXIO3:
            *pre_div  = ((CCM_CS1CDR & CCM_CS1CDR_FLEXIO2_CLK_PRED(7) ) >>  9 ) + 1;
            *post_div = ((CCM_CS1CDR & CCM_CS1CDR_FLEXIO2_CLK_PODF(7) ) >> 25 ) + 1;
            break;
      }
   }

}


//bool FlexIO_Base::config_clock(void)
//{
//   uint8_t prediv, postdiv;
//
//   // calculate dividers
//   calc_pll_clock_div( m_pll_divider, &prediv, &postdiv );
//
//   // write to CCM hardware
//   return( config_clock_div(prediv, postdiv) );
//}
//

//bool FlexIO_Base::config_flex(void)
//{
//   return false;
//}



// An interrupt routine can be triggered from a timer output or
// a shifter status flag output. There is only one interrupt vector
// for each flex module, so it is up to you to verify who tripped
// the interrupt (if more than on source is enabled)
void FlexIO_Base::attachInterrupt(void (*isr)(void))
{
   int irq = FlexIO_Base::get_irq_num();
	_VectorsRam[irq + 16] = isr;
	NVIC_ENABLE_IRQ(irq);
}


void FlexIO_Base::attachInterrupt(void (*isr)(void), uint8_t prio)
{
   int irq = FlexIO_Base::get_irq_num();
	_VectorsRam[irq + 16] = isr;
	NVIC_ENABLE_IRQ(irq);
	NVIC_SET_PRIORITY(irq, prio);
}


void FlexIO_Base::detachInterrupt(void)
{
	NVIC_DISABLE_IRQ(FlexIO_Base::get_irq_num());
}


void FlexIO_Base::clearInterrupt(uint8_t source, uint8_t flag_num)
{
	//DMA_CINT = channel;
}


// Each FlexIO module has one interrupt flag for each timer and one
// for each shifter. For the RT1062, there are 8 of each.
// This function sets one FlexIO interrupt flag.
// @param source    FLEXIO_SHIFTERS or FLEXIO_TIMERS
// @param flag_num  timer or shifter number [0 to 7]
void FlexIO_Base::enableInterruptSource(uint8_t source, uint8_t flag_num)
{
   noInterrupts();
   switch(source) {
      case FLEXIO_SHIFTERS:
         m_flex->SHIFTSIEN |= 1 << flag_num;
         break;
      case FLEXIO_TIMERS:
         m_flex->TIMIEN |= 1 << flag_num;
         break;
   }
   interrupts();
}


// This function clears one FlexIO interrupt flag
// @param flag   timer or shifter number [0 to 7]
// @param source FLEXIO_SHIFTERS or FLEXIO_TIMERS
void FlexIO_Base::disableInterruptSource(uint8_t source, uint8_t flag_num)
{
   noInterrupts();
   switch(source) {
      case FLEXIO_SHIFTERS:
         m_flex->SHIFTSIEN &= ~(1 << flag_num);
         break;
      case FLEXIO_TIMERS:
         m_flex->TIMIEN &= ~(1 << flag_num);
         break;
   }
   interrupts();
}


// This function reads the state of the FlexIO interrupt flags for either the shifters
// or the timers. Flags which do NOT have interrupts enabled are ignored.
// @param source FLEXIO_SHIFTERS or FLEXIO_TIMERS
uint32_t FlexIO_Base::readInterruptFlags(uint8_t source)
{
   switch(source) {
      case FLEXIO_SHIFTERS:
         return( m_flex->SHIFTSIEN & m_flex->SHIFTSTAT );

      case FLEXIO_TIMERS:
         return( m_flex->TIMIEN & m_flex->TIMSTAT );
   }
   return(0);
}


// Same as above, but returns only a single flag
bool FlexIO_Base::readInterruptFlags(uint8_t source, uint8_t flag_num)
{
   uint32_t flags = FlexIO_Base::readInterruptFlags(source);
   if( flags & (1 << flag_num))
      return(true);
   else
      return(false);
}



inline int FlexIO_Base::get_irq_num(void)
{
   int irq;

   // only one interrupt vector per FlexIO module
   switch(m_flex_num) {
      case FLEXIO1: irq = IRQ_FLEXIO1; break;
      case FLEXIO2: irq = IRQ_FLEXIO2; break;
      case FLEXIO3: irq = IRQ_FLEXIO3; break;
      default: irq = 0;
   }

   return(irq);
}

