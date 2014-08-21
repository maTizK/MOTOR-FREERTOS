#include "modbus_mk.h"


/* This funcion initializes the USART1 peripheral
 * 
 * Arguments: baudrate --> the baudrate at which the USART is 
 * 						   supposed to operate
 */
void init_USART1(void){
	
	
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
	
	/* enable APB2 peripheral clock for USART1 
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART1 
	 */
	USART_InitStruct.USART_BaudRate = 38400;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	
	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);	 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}


/* This function is used to transmit a string of characters via 
 * the USART specified in USARTx.
 * 
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 						   (volatile) char *s is the string you want to send
 * 
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 * 
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
 * */
void USART_puts(volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx, *s);
		*s++;
	}
}

// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void){
	
	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		
		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART1->DR; // the character from the USART1 data register is saved in t
		
		/* check if the received character is not the LF character (used to determine end of string) 
		 * or the if the maximum string length has been been reached 
		 */
		if( (t != '\n') && (cnt < MAX_STRLEN) ){ 
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;
			//USART_puts(USART1, received_string);
		}
	}
}



/* function for  */
static uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
     uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
     uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
     unsigned int i; /* will index into CRC lookup */
 
     /* pass through message buffer */
     while (buffer_length--) {
         i = crc_hi ^ *buffer++; /* calculate the CRC  */
         crc_hi = crc_lo ^ table_crc_hi[i];
         crc_lo = table_crc_lo[i];
     }
 
     return (crc_hi << 8 | crc_lo);
}


/* write to modbus */
int write_modbus( uint8_t * req, int nb)
{
	int i; 

	
	USART_puts(volatile char *s);
	
	
	for (i = 0; i < num; i++) printf("{%.2X}", req[i]);
	printf("\n");
	return nb;
}

/* read from modbus line (RS485) */
int read_modbus( uint8_t * req, int nb)
{
	
	memcpy(req, recived_string, nb);
	for (i = 0; i < nb; i++) 
		printf("[%.2X]", req[i]);
	printf("\n"); 
	return nb; 	
}

/* modbus read input bits */
int modbus_RIB( int16_t address, int nb, uint8_t *dst )
{
	/* first send request */

	uint8_t req[REQ_MAX_LEN];
	uint8_t rsp[REQ_MAX_LEN];
	int byte_count; 
	int req_length; 
	int bit_check = 0; 
	int pos = 0; 
	
	req[0] = 0x36; 
	req[1] = 0x2;
	req[2] = address >> 8;
	req[3] = address & 0x00ff;
	req[4] = nb >> 8;
	req[5] = nb & 0x00ff; 
	req_length = 6;
	uint16_t crc = crc16(req, req_length);

	req[req_length++] = crc >> 8; 
	req[req_length++] = crc & 0x00ff; 
	int rc;	
	/* write request  to modbus line */
	
	rc =  write_modbus( req, req_length);

	/* if req was sent, recieve response */
	if (rc > 0) {
		rc = read_modbus( dst, 6); 
		if (rc < 0) return -1; 

	}
	return rc;
}


/* modbus write input bits */
int modbus_WIB( int16_t address, int nb, uint8_t *src )
{
	/* first send request */

	uint8_t req[REQ_MAX_LEN];
	int byte_count; 
	int req_length; 
	int bit_check = 0; 
	int pos = 0; 
	
	req[0] = 0x36; 
	req[1] = 0xF;
	req[2] = address >> 8;
	req[3] = address & 0x00ff;
	req[4] = nb >> 8;
	req[5] = nb & 0x00ff; 
	req_length = 6; 
	
    	byte_count = (nb / 8) + ((nb % 8) ? 1 : 0);
  	req[req_length++] = byte_count;

	int i; 
    	for (i = 0; i < byte_count; i++) {
        
		int bit;
		bit = 0x01;
        	req[req_length] = 0;

        	while ((bit & 0xFF) && (bit_check++ < nb)) {
            		if (src[pos++])
                		req[req_length] |= bit;
            		else
                		req[req_length] &=~ bit;

            		bit = bit << 1;
        	}	
        	
		req_length++;
	 }

	uint16_t crc = crc16(req, req_length);

	req[req_length++] = crc >> 8; 
	req[req_length++] = crc & 0x00ff; 
	
	/* write request  to modbus line */
	int rc =  write_modbus( req, req_length); 
	
	/* if req was sent read from modbus line */
	if (rc > 0) read_modbus( req, 8);
	return rc;
}

/* Write the values from the array to the registers of the remote device */
int modbus_WR( int address, int nb, const uint16_t *src)
{
	int rc;
	int i;
    	int req_length;
    	int byte_count;

    	uint8_t req[REQ_MAX_LEN];

	req[0] = 0x36; 
	req[1] = 0x10;
	req[2] = address >> 8;
	req[3] = address & 0x00ff;
	req[4] = nb >> 8;
	req[5] = nb & 0x00ff; 
	req_length = 6; 
	
   	byte_count = nb * 2;
    	req[req_length++] = byte_count;

    	for (i = 0; i < nb; i++) {
		req[req_length++] = src[i] >> 8;
        	req[req_length++] = src[i] & 0x00ff;
    	}

	uint16_t crc = crc16(req, req_length);

	req[req_length++] = crc >> 8; 
	req[req_length++] = crc & 0x00ff;


	/* write request  to modbus line */
	rc = write_modbus( req, req_length); 
	
	/* if req was sent, read from modbus line */ 

	if (rc > 0) read_modbus(req, 8);
	return rc;
}


/* Read the values from the array to the registers of the remote device */
int modbus_RR( int address, int nb, uint16_t *src)
{
    int rc;
    int i;
    int req_length;
    int byte_count;

    uint8_t req[REQ_MAX_LEN];
    uint8_t rsp[REQ_MAX_LEN];

	req[0] = 0x36; 
	req[1] = 0x4;
	req[2] = address >> 8;
	req[3] = address & 0x00ff;
	req[4] = nb >> 8;
	req[5] = nb & 0x00ff; 
	req_length = 6; 
	
    	byte_count = nb * 2;
	
	uint16_t crc = crc16(req, req_length);

	req[req_length++] = crc >> 8; 
	req[req_length++] = crc & 0x00ff; 
	/* write request  to modbus line */
	rc = write_modbus( req, req_length); 
	if (rc < 0 ) return -1; 	
	rc = read_modbus( rsp, nb*2 + 5);
	int offset = 1;
	for (i = 0; i < nb; i++) {
            /* shift reg hi_byte to temp OR with lo_byte */
            src[i] = (rsp[offset + 2 + (i << 1)] << 8) |
                rsp[offset + 3 + (i << 1)];
		
        }

	return 0;
}

void modbusMOTOR_task(void * pvParameters)
{
	
	int i;

	uint8_t dest[8];
	uint8_t src[4];
	uint16_t tab_reg[10];
	uint16_t spd[5]; 
	
	
	src[0]=1; src[1]=1; src[2]=1; src[3]=1;
	
	spd[0]=2000;spd[1]=0;spd[2]=2250;spd[3]=15;spd[4]=15;
	
	
	if ( modbus_RIB( 0,8,dest) < 0 )
	{
		 printf("ERORR\n!");
	}
	
	modbus_RR(0,10,tab_reg);
	
	modbus_WIB( 0, 3, src);
	
	modbus_WR( 0, 5, spd);
	
	const TickType_t xDelay = 5000 / portTICK_PERIOD_MS;
	
	for (;;)
	{
		
		modbus_RR(0, 10, tab_reg);
		vTaskDelay( xDelay );
		
	}
	return 0;
	
}
