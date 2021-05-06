
/* declarations to suppress warnings for Interrupt Handlers */
void USART1_IRQHandler(void);


#define SERIALBUFSZ		256
/* set XMITDEBUG to a usart number to continuously 0x55 to verify that
 * USART function. 0 disabled
 */
#define XMITDEBUG		0
/*
 * if non zero report OmniBus serial port Receive errors and buffer full errors
 * by stuffing an @ for errors and a ~ for buffer full into the DEBUG port
 * transmit buffer.
 */
#define REPORTOMNIBUSRECVERR 0

struct SerialBuf {
	unsigned		inPtr;
	unsigned		outPtr;
	unsigned char	buf[SERIALBUFSZ];
};

struct SerialPort {
	USART_TypeDef *uart;
	struct SerialBuf recv;
	struct SerialBuf xmit;
};

struct SerialPort debugPort = { .uart = USART1 };

static void (*busCompleteCallback)(void);

/**
 * @brief	set busCompleteCallback
 *
 * @param	callback, pointer to call back function
 */
void setBusCompleteCallback(void (*callback)(void))
{
	busCompleteCallback = callback;
}

/**
 * @brief	Check output buffer and return empty, not empty
 *
 * @param	file, fileID of buffer to check
 *
 * @return	1, if cir buffer empty, 0 if not, -1 if invalid ID
 */
int serialOutputEmpty(enum fd file)
{
	switch(file) {
	case in:
	case out:
	case err:
		return debugPort.xmit.inPtr == debugPort.xmit.outPtr;
	default:
		break;
	}
	return -1;
}

/**
 * @brief	Check input buffer and return empty, not empty
 *
 * @param	file, fileID of buffer to check
 *
 * @return	0, if cir buffer empty, 1 if not, -1 if invalid ID
 */
int serialInputEmpty(enum fd file)
{
	switch(file) {
	case in:
	case out:
	case err:
		return debugPort.recv.inPtr != debugPort.xmit.outPtr;
	default:
		break;
	}
	return -1;
}

/**
 * @brief	Get the next byte from a cir buffer.
 */
static int usartGetByte(struct SerialBuf *buf)
{
	int input;

	// is the quque empty?
	if(buf->inPtr == buf->outPtr) {
		// Yes - return an empty indication
		return -1;
	}
	// No - get the byte from the queue and return it
	input = buf->buf[buf->outPtr];
	buf->outPtr = (buf->outPtr + 1) % SERIALBUFSZ;

	return input;
}

/**
 * @brief	Put a byte at the end of a cir buffer.
 */
static int usartPutByte(struct SerialBuf *buf, uint8_t b)
{
	// Where is the next slot in the cir queue?
	int next = (buf->inPtr + 1) % SERIALBUFSZ;

	// Is there room in the queue for another value?
	if(next != buf->outPtr) {
		// Yes - add it and move the head pointer.
		buf->buf[buf->inPtr] = b;
		buf->inPtr = next;
	} else {
		// No -return an error indication
		return -1;
	}

	// Return success
	return 0;
}

/**
 * @brief	This function handles USART1 global interrupt for Test Debug.
 */
void USART1_IRQHandler(void)
{
#if	XMITDEBUG != 1
	int b;
#endif

	/* check for receive errors */
	if(USART1->SR & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
		/* next two lines  clear the RXNE and error bits, the are compiler
		 * dependent and have only been verified valid for ARM GCC */
		USART1->SR;
		USART1->DR; /* discard received byte due to errors */
	}
	/* handle receive data */
	if(USART1->SR & USART_SR_RXNE) {
		usartPutByte(&debugPort.recv, (uint8_t) USART1->DR);
	}
	/* handle transmit data */
	if((USART1->SR & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE)) {
#if XMITDEBUG == 1
		USART1->DR = 0x55;
#else	/* XMITDEBUG */
		if((b = usartGetByte(&debugPort.xmit)) < 0) { /* queue empty */
			USART1->CR1 &= ~USART_CR1_TXEIE; /* disable transmit interrupt */
		} else {
			USART1->DR = (uint8_t)b;
		}
#endif	/* XMITDEBUG */
	}
}


/**
 * @brief add a byte to a Serial Port xmit buffer
 *
 * @param	file, a fileno 0,1,2 == debug
 * @param	c, a byte to transmit.
 *
 * @return	int, > 0 failure, <= 0 success
 */
int putSerialByte(int c, enum fd file)
{
	int ret;
	struct SerialBuf *buf;
	USART_TypeDef *usart;

	switch(file) {
	case in:
	case out:
	case err:
		usart = debugPort.uart;
		buf = &debugPort.xmit;
		break;
	default:
		return -1;
		break;
	}
	if(c == '\n') {
		usartPutByte(buf, '\r');
	}
	if((ret = usartPutByte(buf, (uint8_t)c)) < 0) {
		return ret;
	}
	if(usart->CR1 & USART_CR1_UE) { /* if usart enabled */
#if 0
		if(!(usart->CR1 & USART_CR1_TE)) { /* if transmit not enabled */
			usart->CR1 |= USART_CR1_TE; /* enable it */
		}
#endif
		/* if transmitter enabled and Transmitter Empty interrupt disabled */
		if((usart->CR1 & USART_CR1_TE) && !(usart->CR1 & USART_CR1_TXEIE)) {
			usart->CR1 |= USART_CR1_TXEIE; /* enable it */
		}
	}
	return 0;
}

/**
 * @brief	get a byte from a serial Port recv Buffer
 *
 * @param	file, a filenumber 0,1,2 == debug, 3 == omniBus
 *
 * @return	int, > 0 failure, <= 0 success
 */
int getSerialByte(enum fd file)
{
	switch(file) {
	case in:
	case out:
	case err:
		return usartGetByte(&debugPort.recv);
	default:
		break;
	}
	return -1;
}

/**
 * @brief	Initialize Serial IO module
 */
void serialInit(void)
{
	/* initialize usart1 */
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->APB2ENR;	/* time to settle */
	USART1->CR1 = 0; /* disable usart1 */

	gpio_a_ena();
	gpio_init(GPIOA, 9, GPIO_OUT_ALT_PP | GPIO_OUT_50MHZ);
	gpio_init(GPIOA, 10, GPIO_IN_FLOAT);

	/* setup and enable interrupts */
	NVIC_SetPriority(USART1_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(USART1_IRQn);

	/* USART1, 8bit, no parity, 1 stop bit */
	USART1->CR2 = 0;
	USART1->CR3 = 0;
	USART1->CR1 = USART_CR1_TE | USART_CR1_RE;
	/* 230400 bits per second */
	USART1->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), 230400);

	/* enable usart */
	USART1->CR1 |= USART_CR1_UE | USART_CR1_TXEIE | USART_CR1_RXNEIE;

#if 0
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	/* make sure AFIO clock enabled */
	RCC->APB2ENR;	/* time to settle */
	/* select port B for EXTI11 */
	AFIO->EXTICR[2] = (AFIO->EXTICR[2] & ~0xf000) | 0x1000;
	EXTI->IMR |= GPIO_PIN_11;	/* unmask interrupt from pin 11 */
	EXTI->RTSR |= GPIO_PIN_11;	/* interrupt on rising edge */
	EXTI->FTSR |= GPIO_PIN_11;	/* interrupt on falling edge */
	/* BUS RX Monitor, EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
#endif

}
