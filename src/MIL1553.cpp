#include <Arduino.h>
#include <MIL1553.h>
#include <Flex1553.h>



MIL_1553_BC::MIL_1553_BC(FlexIO_1553TX *tx0, FlexIO_1553RX *rx0, FlexIO_1553RX *rx1)
{


   flexIO_tx0 = tx0;
   flexIO_rx0 = rx0;
   flexIO_rx1 = rx1;

}



bool MIL_1553_BC::begin(void)
{
   bool result = true;

   if(flexIO_tx0 != NULL) {
      if( flexIO_tx0->begin() == false )
         result = false;  // function failed
   }
   if(flexIO_rx0 != NULL) {
      if( flexIO_rx0->begin() == false )
         result = false;  // function failed
   }
   if(flexIO_rx1 != NULL) {
      if( flexIO_rx1->begin() == false )
         result = false;  // function failed
   }
   return(result);
}


// this will send one packet to a Remote Terminal
// This uses the FlexIO_1553TX class to access the hardware
// Note that in 1553 nomenclature, this is called a RECEIVE command
bool MIL_1553_BC::send(packet_1553 *packet, int8_t chan)
{
   //tx_bus     = bus;
   uint8_t wc = packet->getWordCount();

   flexIO_tx0->set_channel(chan);  // set bus channel A or B
   flexIO_tx0->send_command(packet->getRta(), packet->getSubAddress(), wc);
   //int flexIO_tx0->send_data( uint16_t data )


}
