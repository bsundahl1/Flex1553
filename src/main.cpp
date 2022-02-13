#include <Arduino.h>
#include <CmdBuffer.h>
#include <CmdParser.h>
#include <Flex1553.h>
#include <PatternGen.h>

//#include <TeensyDebug.h>
//#pragma GCC optimize ("O0")

#define SW_REV   "Pinger2 v0.1"


const int ledPin = 13;
const int pwmPin = 14;
const int rx1553Pin = 40;
unsigned long loopCount = 0;
unsigned long loopsPerSec = 0;
unsigned long lastLoops = 0;
bool ledState  = false;
//uint32_t pattern[] = {0x12345678, 0x9abcdef0, 0xb0119876, 0x543210ab, 0xdeadbeef};
uint32_t pattern[] = {0x01020304, 0x05060708, 0x090a0b0c, 0x0d0e0f10, 0x11121314};
int syncPattern = FLEX1553_COMMAND_WORD;

//LaParser ser1Parser(1);
CmdParser cmdParser;
CmdBuffer<64> cmdBuffer1;

// FlexIO
FlexIO_1553TX flex1553TX( FLEXIO2, true, false, false, false );
FlexIO_1553RX flex1553RX( FLEXIO3, rx1553Pin );

void processSerialCommand( char *buffer );
static void isr1553Rx(void);


// the setup() method runs once, when the sketch starts
void setup() {
   // initialize USB (host)serial port
   Serial.begin(115200);
   while (!Serial)
      ; // wait for serial port to connect

   Serial.println( SW_REV );

   // Debugger will use second USB Serial;
   //debug.begin(SerialUSB1);

   pinMode(ledPin, OUTPUT);
   pinMode(pwmPin, OUTPUT);

   //cmdParser.setOptKeyValue(true);
   cmdBuffer1.setEcho( true );
   //cmdBuffer1.setStartChar('/');
   //cmdBuffer1.setNumStartChars( 3 );
   //cmdBuffer1.setOptID( '1' );
   cmdParser.setOptParens( '(', ')' );

   // start 1553
   if( !flex1553TX.begin() )
      Serial.println( "flexTX.begin() failed" );
   if( !flex1553RX.begin() )
      Serial.println( "flexRX.begin() failed" );

   analogWrite(pwmPin, 128);

   flex1553RX.attachInterrupt(isr1553Rx);
   flex1553RX.enableInterruptSource(FLEXIO_SHIFTERS, 1);
   flex1553RX.enableInterruptSource(FLEXIO_SHIFTERS, 3);
}


void loop()
{

   // check serial port
      // This reads any avialable characters from the Serial stream into the
      // cmdbuffer. If the end of the command is found (line terminator)
      // then the parser is called
      if( cmdBuffer1.readSerialChar(&Serial) ){
        processSerialCommand( cmdBuffer1.getStringFromBuffer() );
      }



   // toggle LED
      if( (loopCount % 0x0400000L) == 0 ){  // once every few million loops
          ledState = !ledState;
          digitalWrite(ledPin, ledState);
      }

   // track loop time
      if( (millis() % 100) == 0 ){ // every 100 milliseconds
        loopsPerSec = (loopCount - lastLoops); // actually loops per 0.1 second
        lastLoops = loopCount;
      }
      loopCount++;
}



void processSerialCommand( char *buffer )
{
   //Serial.println(buffer);
   delay(10);
   char *errStr = NULL;
   uint32_t ulVal, ulVal2;
   int iVal, iVal2;
   char *str = 0;
   //static bool configured_1553 = false;

   // parses buffer into command and params
   if (cmdParser.parseCmd(buffer) != CMDPARSER_ERROR)
   {  // begin command processing

      // get top level command
      //Serial.println( cmdParser.getCommand() );

      if( cmdParser.equalCommand("Loop") ) {
         Serial.print( loopsPerSec * 10 ); // why does this not work???
         Serial.print( ", " );
         Serial.println(  (loopCount - lastLoops) * 10 );
      }
      else if( cmdParser.equalCommand("Loops") ) {   // Loops
         Serial.print( lastLoops );
         Serial.print( ", " );
         Serial.print( loopCount );
         Serial.print( ", " );
         Serial.println( loopsPerSec );
      }
      else if( cmdParser.equalCommand("Rev") ) {     // Rev
         Serial.println( SW_REV );
      }
      else if( cmdParser.equalCommand("Test") ) {    // Test
         iVal2 = cmdParser.getParamCount();
         Serial.print( "Parameters: " );
         Serial.println( iVal2 );

         // print all parameters
         for( iVal=0; iVal<=iVal2; iVal++ ) {
            Serial.print( "   " );
            Serial.println( cmdParser.getCmdParam(iVal) );
         }
      }
      else if( cmdParser.equalCommand("Key") ) {    // Test
         str = cmdParser.getValueFromKey("bill");
         if(str != NULL) {
            Serial.print( "bill: " );
            Serial.println( str );
         }
         str = cmdParser.getValueFromKey("jeff");
         if(str != NULL) {
            Serial.print( "jeff: " );
            Serial.println( str );
         }
         str = cmdParser.getValueFromKey("anh");
         if(str != NULL) {
            Serial.print( "anh: " );
            Serial.println( str );
         }
      }
      else if( cmdParser.equalCommand("Num") ) {    // Number test
         if( cmdParser.getParamCount() >= 1 )
           str = cmdParser.getCmdParam(1);
         else
           str = NULL;

         //
         Serial.print( "neg   " );
         Serial.println( cmdParser.negInStr(str) );
         Serial.print( "int   " );
         Serial.println( cmdParser.intInStr(str) );
         Serial.print( "float " );
         Serial.println( cmdParser.floatInStr(str) );
         Serial.print( "hex   " );
         Serial.println( cmdParser.hexInStr(str) );
      }
      else if( cmdParser.equalCommand("Float") ) {    // Float test
         float fVal = cmdParser.getCmdParamAsFloat(1);
         if( !cmdParser.isParseError() ) { // if no errors
            Serial.print( "val: " );
            Serial.println( fVal );
         }
      }
      else if( cmdParser.equalCommand("Int") ) {    // Int test
         iVal = cmdParser.getCmdParamAsInt(1, -5, 300);
         if( !cmdParser.isParseError() ) { // if no errors
            Serial.print( "val: " );
            Serial.println( iVal );
         }
      }
      else if( cmdParser.equalCommand("Test2") ) {    // Test2
         iVal = 0;
         int8_t cVal = cmdParser.getCmdParamAsInt(1, -128, 127);
         float  fVal = cmdParser.getCmdParamAsFloat(2, -5.5, 10.5);
         if( cmdParser.getParamCount() > 2 )
            iVal = cmdParser.getCmdParamAsInt(3, -5, 300);
         if( !cmdParser.isParseError() ) { // if no errors
            Serial.print( "val1: " );
            Serial.print( (int)cVal );
            Serial.print( "   val2: " );
            Serial.print( fVal );
            Serial.print( "   val3: " );
            Serial.println( iVal );
         }
      }
      // *************** 1553 ***************
      //else if( cmdParser.equalCommand("Config") ) {  // Config
      //   iVal = 0;
      //   if( cmdParser.getParamCount() >= 1 )
      //     iVal = atoi( cmdParser.getCmdParam(1) );
      //   //Serial.print( "Config FlexIO1: " );
      //   //Serial.println( Flex1553TX_config() );
      //   //Serial.print( "Config FlexIO2: " );
      //   //Serial.println( Flex2_1553TX_config() );
      //   //Serial.print( "Config FlexIO3: " );
      //   //Serial.println( Flex3_1553RX_config() );
      //   Serial.print( "Config FlexIO1: " );
      //   switch(iVal)
      //   {
      //      case 1: Serial.println( Flex1_1553Sync_config1() );
      //         break;
      //      case 2: Serial.println( Flex1_1553Sync_config2() );
      //         break;
      //      case 3: Serial.println( Flex1_1553Sync_config3() );
      //         break;
      //      case 4: Serial.println( Flex1_1553Sync_config4() );
      //         break;
      //      case 5: Serial.println( Flex1_1553Sync_config5() );
      //         break;
      //      default:
      //         Serial.println( "Missing config #" );
      //   }
      //}
      else if( cmdParser.equalCommand("Send") ) {    // Send ************************
         if( cmdParser.getParamCount() >= 1 )
           iVal = atoi( cmdParser.getCmdParam(1) );
         else
           iVal = 2;

         // this sets up the receiver watch for a command word
         flex1553RX.set_sync(FLEX1553_COMMAND_WORD);

         //iVal = Flex2_1553TX_send( FLEX1553_COMMAND_WORD, 0x1234 );
         Serial.print( "cmd ret:  " );     // RTA, SA, WC
         //Serial.println( Flex1553TX_send_command(3, 7, 1) );
         //Serial.println( Flex2_1553TX_send( FLEX1553_COMMAND_WORD, 0x18e1 ) );
         Serial.println( flex1553TX.send( FLEX1553_COMMAND_WORD, 0x18e1 ) );

         if( iVal > 1 ) {
            Serial.print( "cmd ret:  " );
            //Serial.println( Flex1553TX_send( FLEX1553_COMMAND_WORD, 0x5678 ) );
            //Serial.println( Flex2_1553TX_send( FLEX1553_DATA_WORD, 0x5678 ) );
            Serial.println( flex1553TX.send( FLEX1553_DATA_WORD, 0x5678 ) );
         }

         if( iVal > 2 ) {
            Serial.print( "cmd ret:  " );
            //Serial.println( Flex2_1553TX_send( FLEX1553_DATA_WORD, 0xbead ) );
            Serial.println( flex1553TX.send( FLEX1553_DATA_WORD, 0xbead ) );
         }
      }

      else if( cmdParser.equalCommand("Chan") ) {
         if( cmdParser.getParamCount() >= 1 )
            iVal = atoi( cmdParser.getCmdParam(1) );
         else
            iVal = 1;

         Serial.print( "Channel " );
         Serial.print( iVal );
         Serial.print( ": ret: " );
         Serial.println( flex1553TX.set_channel(iVal) );
      }
      else if( cmdParser.equalCommand("Trigger") ) {
        if( cmdParser.getParamCount() >= 1 )
          ulVal = cmdParser.getCmdParamAsInt(1);  // pattern
        else
          ulVal = 0xff00;
        if( cmdParser.getParamCount() >= 2 )
          ulVal2 = cmdParser.getCmdParamAsInt(2);  // mask
        else
          ulVal2 = 0x0;
        Serial.print( "status: " );
       // Serial.print( Flex1553RX_trigger(ulVal, ulVal2) );
        Serial.print( flex1553RX.set_trigger(ulVal, ulVal2) );
      }
      else if( cmdParser.equalCommand("Read") ) {   // Read *******************************
         //Serial.print( "RX: 0x" );
         //Serial.print( Flex3_1553RX_read_data(), HEX );
         //Serial.print( flex1553RX.read_data(), HEX );
         uint32_t rx_data = flex1553RX.read_data();
         Serial.print( "RX Sync: " );
         Serial.print( (rx_data >> 17) & 0x07 );
         Serial.print( " data: " );
         Serial.print( (rx_data >> 1)  & 0xffff, HEX );
         Serial.print( " parity: " );
         Serial.print( rx_data & 0x01 );
         Serial.print( "  fault: 0x" );
         Serial.print( flex1553RX.read_faults(), HEX );
         Serial.println();
      }
      else if( cmdParser.equalCommand("Status") ) {
         Serial.print( "status TX1 0x" );
         Serial.print( flex1553TX.get_status(), HEX );
         Serial.print( "  RX 0x" );
         Serial.println( flex1553RX.get_status(), HEX );
      }
      else if( cmdParser.equalCommand("SetS3") ) {  // write to shifter3 buffer
         if( cmdParser.getParamCount() >= 1 ) {
            //ulVal = atoi( cmdParser.getCmdParam(1) );
            ulVal = cmdParser.getCmdParamAsInt(1);
            Serial.print( "status TX1 0x" );
            Serial.println( ulVal, HEX );
            Flex1_writeShifter3( ulVal );
         }
      }
      else if( cmdParser.equalCommand("readS3") ) { // read from shifter3 buffer
         Serial.print( "RX: 0x" );
         Serial.println( Flex1_readShifter3(), HEX );
         //Flex1_writeShiftConfig3( ulVal );
      }
      //else if( cmdParser.equalCommand("Pins") ) {
      //  ulVal = 0;
      //  //if( cmdParser.getParamCount() >= 1 )
      //  //  iVal = atoi( cmdParser.getCmdParam(1) );
      //  //else
      //  //  iVal = 1;
      //
      //  ulVal = flex1553TX.get_pin_states();
      //  Serial.print( "Flex Pins: 0x" );
      //  Serial.println( ulVal, HEX);
      //}

      // ************* general FlexIO stuff **********
      else if( cmdParser.equalCommand("GetParams") ) {
         unsigned int uVal = 0;
         //if( cmdParser.getParamCount() >= 1 )
         //  iVal = atoi( cmdParser.getCmdParam(1) );
         //else
         //  iVal = 1;

         uVal = flex1553TX.get_params( FLEXIO_TRIGGERS );
         Serial.print( " TRIG: " );
         Serial.print( uVal );

         uVal = flex1553TX.get_params( FLEXIO_PINS );
         Serial.print( "  PIN: " );
         Serial.print( uVal );

         uVal = flex1553TX.get_params( FLEXIO_TIMERS );
         Serial.print( "  TIMER: " );
         Serial.print( uVal );

         uVal = flex1553TX.get_params( FLEXIO_SHIFTERS );
         Serial.print( "  SHIFTER: " );
         Serial.print( uVal );
         Serial.println();
      }


      else if( cmdParser.equalCommand("Pins") ) {
         unsigned long ulVal = flex1553TX.get_pin_states();
         Serial.print( "Flex Pins: 0x" );
         Serial.println( ulVal, HEX);
      }

      //else if( cmdParser.equalCommand("Config") ) {
      //   Serial.print( "flex_test(): " );
      //   Serial.print( flex_test() );
      //   Serial.println();
      //}

      //else if( cmdParser.equalCommand("SetClock") ) {
      //   if( cmdParser.getParamCount() >= 1 ) {
      //        int iVal = cmdParser.getCmdParamAsInt(1);
      //        if( !cmdParser.isParseError() ) { // if no errors
      //           Serial.print( "ClkDiv: " );
      //           Serial.print( iVal );
      //
      //           uint8_t prediv = 0, postdiv = 0;
      //           //flex.calc_pll_clock_div( iVal, &prediv, &postdiv );
      //           Serial.print( "  PreDiv:" );
      //           Serial.print( prediv );
      //           Serial.print( "  PostDiv:" );
      //           Serial.print( postdiv );
      //           Serial.println();
      //
      //           Serial.print( "config_clock_div(): " );
      //           //Serial.print(flex1553TX.config_clock_div(prediv, postdiv));
      //           Serial.println();
      //        }
      //   }
      //}

      else if( cmdParser.equalCommand("GetClock") ) {
         uint8_t total_div, pre_div, post_div;
         flex1553TX.get_clock_divider( &total_div, &pre_div, &post_div );
         Serial.print( "  Target:" );
         Serial.print( total_div );
         Serial.print( "  PreDiv:" );
         Serial.print( pre_div );
         Serial.print( "  PostDiv:" );
         Serial.print( post_div );
         Serial.println();
      }
      //else if( cmdParser.equalCommand("SetPin") ) {
      //   if( cmdParser.getParamCount() >= 1 ) {
      //        int8_t iVal = cmdParser.getCmdParamAsInt(1);
      //        if( !cmdParser.isParseError() ) { // if no errors
      //           int8_t Fpin = iVal;
      //           Serial.print( "FlexPin: " );
      //           Serial.print( Fpin );
      //
      //           int8_t Tpin = flex1553TX.getTeensyPin( Fpin );
      //           Serial.print( "  TeensyPin: " );
      //           Serial.print( Tpin );
      //           Serial.println();
      //
      //           Serial.print( "setPinMux(): " );
      //           Serial.print(flex1553TX.setPinMux( Tpin ));
      //           Serial.println();
      //        }
      //   }
      //}

      // *********** for pattern generator **********
      else if( cmdParser.equalCommand("110Config") ) {
         Serial.print( "Config FlexIO1: " );
         Serial.println( Flex1_PatGen_config() );
      }
      else if( cmdParser.equalCommand("110Load") ) {
         Serial.print( "Load FlexIO1: " );
         Serial.println( Flex1_PatGen_load(pattern, sizeof(pattern)/4 ));
      }
      else if( cmdParser.equalCommand("110Start") ) {
         Serial.print( "Ret: " );
         Serial.println( Flex1_PatGen_start() );
      }
      else if( cmdParser.equalCommand("110Stop") ) {
         Serial.print( "Ret: " );
         Serial.println( Flex1_PatGen_stop() );
      }
      else if( cmdParser.equalCommand("div") ) {
         if( cmdParser.getParamCount() >= 1 ) {
            iVal = atoi( cmdParser.getCmdParam(1) );
            Serial.print( "Ret: " );
            Serial.println( Flex1_PatGen_clock(iVal, NULL, NULL) );
         }
      }
      else if( cmdParser.equalCommand("110bits") ) {
         if( cmdParser.getParamCount() >= 1 )
            iVal = atoi( cmdParser.getCmdParam(1) );
         else
            iVal = 16;
         Serial.print( "Ret: " );
         Serial.println( Flex1_PatGen_set_bits(iVal) );
      }

      else {
         // ************** default **************
         Serial.println( "Invalid Command" );
      }
   } // end of command processing

   // print any error message
   if( errStr == NULL )   // local errors take priority
      errStr = cmdParser.getErrorStr();  // next is parser errors
   if( errStr == NULL )
      errStr = cmdParser.getWarningStr(); // last is parser warnings
   if( errStr != NULL )
      Serial.println( errStr );  // only the highest priority is printed

   return;
}



static void isr1553Rx(void)
{
   int flags = flex1553RX.readInterruptFlags(FLEXIO_SHIFTERS);

   if(flags & 0x08) { // found sync pattern
      flex1553RX.clearInterrupt(FLEXIO_SHIFTERS, 3);  // not needed in Match Continuous mode?
      if(syncPattern == FLEX1553_COMMAND_WORD) { // change to data sync
         flex1553RX.set_sync(FLEX1553_DATA_WORD);
      }

      Serial.println("found sync");
   }

   if(flags & 0x02) { // received one word

      uint32_t rx_data = flex1553RX.read_data();  // this also clears the interrupt flag
      //flex1553RX.clearInterrupt(FLEXIO_SHIFTERS, 1);

      Serial.print("received word: ");
      Serial.println(rx_data >> 1, HEX);
   }

}
