#ifdef __cplusplus
extern "C" {
#endif


int Flex1_PatGen_config( void );
int Flex1_PatGen_load( uint32_t *data, uint8_t size );
int Flex1_PatGen_start( void );
int Flex1_PatGen_stop( void );
//int Flex1_PatGen_get_status( void );
int Flex1_PatGen_set_bits( int bitCount );
int Flex1_PatGen_clock( uint8_t divider, uint8_t *p_prediv, uint8_t *p_postdiv );



#ifdef __cplusplus
}
#endif
