

//---
void usartInit (void);
void usartInterruptInit(void);
void usartTx(unsigned char c);
unsigned char usartRx(void);
void usartFlush(void);
void setUp32MHzInternalOsc(void);

//------ for interrupt mode
/* USART buffer defines. */

// /* \brief  Receive buffer size: 2,4,8,16,32,64,128 or 256 bytes. */
// #define USART_RX_BUFFER_SIZE 4
// /* \brief Transmit buffer size: 2,4,8,16,32,64,128 or 256 bytes */
// #define USART_TX_BUFFER_SIZE 4
// /* \brief Receive buffer mask. */
// #define USART_RX_BUFFER_MASK ( USART_RX_BUFFER_SIZE - 1 )
// /* \brief Transmit buffer mask. */
// #define USART_TX_BUFFER_MASK ( USART_TX_BUFFER_SIZE - 1 )
//
//
// #if ( USART_RX_BUFFER_SIZE & USART_RX_BUFFER_MASK )
// #error RX buffer size is not a power of 2
// #endif
// #if ( USART_TX_BUFFER_SIZE & USART_TX_BUFFER_MASK )
// #error TX buffer size is not a power of 2
// #endif
//
//
// /* \brief USART transmit and receive ring buffer. */
// typedef struct USART_Buffer
// {
// 	/* \brief Receive buffer. */
// 	volatile uint8_t RX[USART_RX_BUFFER_SIZE];
// 	/* \brief Transmit buffer. */
// 	volatile uint8_t TX[USART_TX_BUFFER_SIZE];
// 	/* \brief Receive buffer head. */
// 	volatile uint8_t RX_Head;
// 	/* \brief Receive buffer tail. */
// 	volatile uint8_t RX_Tail;
// 	/* \brief Transmit buffer head. */
// 	volatile uint8_t TX_Head;
// 	/* \brief Transmit buffer tail. */
// 	volatile uint8_t TX_Tail;
// } USART_Buffer_t;
//
//
// /*! \brief Struct used when interrupt driven driver is used.
// *
// *  Struct containing pointer to a usart, a buffer and a location to store Data
// *  register interrupt level temporary.
// */
// typedef struct Usart_and_buffer
// {
// 	/* \brief Pointer to USART module to use. */
// 	USART_t * usart;
// 	/* \brief Data register empty interrupt level. */
// 	USART_DREINTLVL_t dreIntLevel;
// 	/* \brief Data buffer. */
// 	USART_Buffer_t buffer;
// } USART_data_t;
//
// /* Macros. */
//
//
// /* Functions for interrupt driven driver. */
// void USART_InterruptDriver_Initialize(USART_data_t * usart_data,
//                                       USART_t * usart,
//                                       USART_DREINTLVL_t dreIntLevel );
//
// void USART_InterruptDriver_DreInterruptLevel_Set(USART_data_t * usart_data,
//                                                  USART_DREINTLVL_t dreIntLevel);
//
// bool USART_TXBuffer_FreeSpace(USART_data_t * usart_data);
// bool USART_TXBuffer_PutByte(USART_data_t * usart_data, uint8_t data);
// bool USART_RXBufferData_Available(USART_data_t * usart_data);
// uint8_t USART_RXBuffer_GetByte(USART_data_t * usart_data);
// bool USART_RXComplete(USART_data_t * usart_data);
// void USART_DataRegEmpty(USART_data_t * usart_data);
