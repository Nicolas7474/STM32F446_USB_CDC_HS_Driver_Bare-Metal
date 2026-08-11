/***************************************************************************************************************
 **** STM32F446VET - USB OTG HS device (CDC) bare-metal implementation with DMA ****
 **** For USB3300 (ULPI lines) - Nicolas Prata / 2026 ****

- Lightweight, only two files to add to your project: usb_cdc_hs.c and usb_cdc_hs.h

- Usage:
  - USB_CDC_UserRxCallBack_EP1()
    This callback is triggered when new data has been received on EP1 OUT.
    The argument is the number of bytes received.

  - Use USB_CDC_Read(dest, maxLen) to read received data from the USB CDC
    RX buffer into your application buffer.
    The function returns the number of bytes copied.
    Example: uint16_t len = USB_CDC_Read(destBuff, sizeof(destBuff));

    The internal circular RX buffer, DMA buffers and wrap-around handling
    are completely transparent to the application.
    If more data is available than fits in your application buffer,
    call USB_CDC_Read() again to consume the remaining data.

    USB_CDC_Read() automatically:
		  - handles circular-buffer wrap-around
		  - copies the received data into the application buffer
		  - releases the consumed RX data only after it has been copied
		  - re-arms EP1 OUT DMA when buffer space becomes available

  - Use USB_CDC_Write(src, len) to send data to the VCOM host.
    Example: USB_CDC_Write(srcBuff, len);
    The internal TX circular buffer and USB DMA handling are completely
    transparent to the application.

  - The application does not need to access circBufferRx directly or use
    peek_circBufferRx() / commit_circBufferRx().
    These functions are private implementation details of usb_cdc_hs.c.

  - USB descriptors are defined in usb_cdc_hs.c and their declarations/constants are in usb_cdc_hs.h
*
*  Tested at a solid KB/s (Rx) and MB/s (Tx, with Powershell script)
*
***********************************************************************************************************************/

#include "usb_cdc_hs.h"
#include "stm32f4xx.h"
#include "stm32f446xx.h"

/****************************************************************
 * 		RX buffers for Endpoint structure
*****************************************************************/

#define RX_BUFFER_EP0_SIZE 64U // 8 is normally enough but 64 costs almost nothing in RAM and can prevent the most common USB crashes
#define RX_BUFFER_EP1_SIZE 512 // In HS mode, the bulk OUT EP packet size (DOEPT_TRANSFER_SIZE) is defined as 512 bytes

__attribute__((aligned(4))) static uint8_t rxBufferEp0[RX_BUFFER_EP0_SIZE]; /* Received data is stored here after application reads DFIFO. RX FIFO is shared */

/**********************************************************************
 * 	The application will set linecoding according to the host request
 ***********************************************************************/
/* Since you are overwriting lineCoding[7], the static uint8_t lineCoding initialization can be simplified to just zeros,
   or the standard "115200, 8-N-1" default : 115200 (0x0001C200), 1 Stop bit (0), No Parity (0), 8 Data bits (8)
*/
__attribute__((aligned(4))) static uint8_t lineCoding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

/* USB descriptors are defined here (not in the header) so they are stored only once. */
/* Device string descriptor */
const uint8_t deviceDescriptor [DEVICE_DESCRIPTOR_LENGTH] __attribute__((aligned(4))) = {
	DEVICE_DESCRIPTOR_LENGTH, //
	0x01, /* Descriptor type - device */
	0x00, /* 0x0110 = usb 1.1 ; 0x0200 = usb 2.0 */
	0x02,
	0x00, /* Class information is specified at the interface level */
	0x00, /* Subclass */
	0x00, /* Protocol */
	EP0_SIZE, /* EP0 size */
	LOBYTE(USBD_VID),
	HIBYTE(USBD_VID),
	LOBYTE(USBD_PID_HS),
	HIBYTE(USBD_PID_HS),
	0x00, /* ver. (BCD) */
	0x02, /* ver. (BCD) */
	0x01, /* Manufactor string index */
	0x02, /* Product string index */
	0x03, /* Serial number string index */
	1 /* configuration count */
};

/* Configuration descriptor */
const uint8_t configurationDescriptor [CONFIGURATION_DESCRIPTOR_LENGTH] __attribute__((aligned(4))) = {
	// EP0 being the mandatory "Default Control Pipe" for all USB devices, it is never declared in the Configuration Descriptor.
	/*Configuration Descriptor*/
	0x09,   /* bLength: Configuration Descriptor size */
	0x02,      /* bDescriptorType: Configuration */
	CONFIGURATION_DESCRIPTOR_LENGTH,                /* wTotalLength:no of returned bytes */
	0x00,
	0x02,   /* bNumInterfaces: 2 interface */
	0x01,   /* bConfigurationValue: Configuration value */
	0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
	0xC0,   /* bmAttributes: self powered */
	0x32,   /* MaxPower 0 mA */

	/*---------------------------------------------------------------------------*/

	/*Interface 0 Descriptor */
	0x09,   /* bLength: Interface Descriptor size */
	0x04,  /*  bDescriptorType: Interface */
	0x00,   /* bInterfaceNumber: Number of Interface */
	0x00,   /* bAlternateSetting: Alternate setting */
	0x01,   /* bNumEndpoints: one CDC notification IN endpoint (EP2) */
	0x02,   /* bInterfaceClass: Communication Interface Class */
	0x02,   /* bInterfaceSubClass: Abstract Control Model */
	0x01,   /* bInterfaceProtocol: Common AT Commands (CDC ACM) */
	0x00,   /* iInterface: */

	/*Header Functional Descriptor*/
	0x05,   /* bLength: Endpoint Descriptor size */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x00,   /* bDescriptorSubtype: Header Func Desc */
	0x10,   /* bcdCDC: spec release number */
	0x01,

	/*Call Management Functional Descriptor*/
	0x05,   /* bFunctionLength */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x01,   /* bDescriptorSubtype: Call Management Func Desc */
	0x00,   /* bmCapabilities: D0+D1 */
	0x01,   /* bDataInterface: 1 */

	/*ACM Functional Descriptor*/
	0x04,   /* bFunctionLength */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
	0x02,   /* bmCapabilities */

	/*Union Functional Descriptor*/
	0x05,   /* bFunctionLength */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x06,   /* bDescriptorSubtype: Union func desc */
	0x00,   /* bMasterInterface: Communication class interface */
	0x01,   /* bSlaveInterface0: Data Class Interface */

	/* CDC notification endpoint (required by the CDC ACM communication interface).
	 * EP2 IN is intentionally not used by the current application, but it is part
	 * of the CDC ACM implementation requirements. The endpoint is configured in
	 * hardware and left NAKed until a Serial-State notification is implemented. */
	0x07,   /* bLength */
	0x05,   /* bDescriptorType: Endpoint */
	(0x80U | USB_CDC_NOTIFICATION_EP),   /* bEndpointAddress: EP2 IN */
	0x03,   /* bmAttributes: Interrupt */
	LOBYTE(USB_CDC_NOTIFICATION_MPS),
	HIBYTE(USB_CDC_NOTIFICATION_MPS),
	USB_CDC_NOTIFICATION_INTERVAL,

	/*---------------------------------------------------------------------------*/

	/*Data class interface descriptor*/
	0x09,   /* bLength: Endpoint Descriptor size */
	0x04,  /* bDescriptorType: */
	0x01,   /* bInterfaceNumber: Number of Interface */
	0x00,   /* bAlternateSetting: Alternate setting */
	0x02,   /* bNumEndpoints: Two endpoints used */
	0x0A,   /* bInterfaceClass: CDC */
	0x00,   /* bInterfaceSubClass: */
	0x00,   /* bInterfaceProtocol: */
	0x00,   /* iInterface: */

	/*Endpoint OUT Descriptor*/
	0x07,   						  /* bLength: Endpoint Descriptor size */
	0x05,     						  /* bDescriptorType: Endpoint */
	0x01,                      		  /* bEndpointAddress */
	0x02,                       	  /* bmAttributes: Bulk */
	LOBYTE(USB_CDC_MAX_PACKET_SIZE),  /* wMaxPacketSize: evaluates to 0x00 for 512 */
	HIBYTE(USB_CDC_MAX_PACKET_SIZE),  /* wMaxPacketSize: evaluates to 0x02 for 512 */
	0x00,                             /* bInterval: ignore for Bulk transfer */

	/*Endpoint IN Descriptor*/
	0x07,   						  /* bLength: Endpoint Descriptor size */
	0x05,     						  /* bDescriptorType: Endpoint */
	0x81,                     		  /* bEndpointAddress (EP1) */
	0x02,                             /* bmAttributes: Bulk */
	LOBYTE(USB_CDC_MAX_PACKET_SIZE),  /* wMaxPacketSize: evaluates to 0x00 for 512 */
	HIBYTE(USB_CDC_MAX_PACKET_SIZE),  /* wMaxPacketSize: evaluates to 0x02 for 512 */
	0x00                              /* bInterval: ignore for Bulk transfer */
};		// EP0 uses address 0x00 (OUT) and 0x80 (IN) implicitly and doesn't need an EP Descriptor (as Default Control Pipe required by the USB spec)
		// Its properties are defined directly in the Device Descriptor

/* Required: tells the host OS how the device would behave if it were plugged into a port running at its other supported speed */
const uint8_t deviceQualifierDescriptor [DEVICE_QUALIFIER_LENGTH] __attribute__((aligned(4))) = {
	DEVICE_QUALIFIER_LENGTH, /* 0x0A (10 bytes) */
	0x06,                    /* bDescriptorType: DEVICE_QUALIFIER (0x06) */
	0x00,                    /* bcdUSB LSB: 0x00 */
	0x02,                    /* bcdUSB MSB: 0x02 -> USB 2.0 specification (0x0200) */
	0x00,                    /* bDeviceClass: Specified at the Interface level */
	0x00,                    /* bDeviceSubClass: 0 */
	0x00,                    /* bDeviceProtocol: 0 */
	0x40,                    /* bMaxPacketSize0: 64 bytes (0x40) for EP0 at the other speed */
	0x01,                    /* bNumConfigurations: 1 valid configuration */
	0x00                     /* bReserved: Must be 0x00 */
};

/* Other Speed Configuration Descriptor (Same as Configuration Descriptor, but type 0x07)
   And Bulk EP wMaxPacketSize set to 64 bytes for Full-Speed mode */
const uint8_t otherSpeedConfigurationDescriptor [CONFIGURATION_DESCRIPTOR_LENGTH] __attribute__((aligned(4))) = {
    0x09,                               /* bLength */
    0x07,                               /* bDescriptorType: OTHER_SPEED_CONFIGURATION */
    CONFIGURATION_DESCRIPTOR_LENGTH, 0x00, /* wTotalLength */
    0x02,                               /* bNumInterfaces */
    0x01,                               /* bConfigurationValue */
    0x00,                               /* iConfiguration */
    0xC0,                               /* bmAttributes: Self-powered */
    0x32,                               /* MaxPower: 100 mA */
    /* Interface 0 (CDC Control) */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    /* Header Functional Descriptor */
    0x05, 0x24, 0x00, 0x10, 0x01,
    /* Call Management Functional Descriptor */
    0x05, 0x24, 0x01, 0x00, 0x01,
    /* ACM Functional Descriptor */
    0x04, 0x24, 0x02, 0x02,
    /* Union Functional Descriptor */
    0x05, 0x24, 0x06, 0x00, 0x01,
    /* CDC notification endpoint (required by CDC ACM; not used by the application). */
    0x07, 0x05, (0x80U | USB_CDC_NOTIFICATION_EP), 0x03,
    LOBYTE(USB_CDC_NOTIFICATION_MPS), HIBYTE(USB_CDC_NOTIFICATION_MPS),
    USB_CDC_NOTIFICATION_INTERVAL,
    /* Interface 1 (CDC Data) */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    /* Endpoint OUT Descriptor (FS Bulk: Max Packet Size = 64 bytes) */
    0x07, 0x05, 0x01, 0x02,
    LOBYTE(64), HIBYTE(64),
    0x00,
    /* Endpoint IN Descriptor (FS Bulk: Max Packet Size = 64 bytes) */
    0x07, 0x05, 0x81, 0x02,
    LOBYTE(64), HIBYTE(64),
    0x00
};

/* Language string descriptor */
const uint8_t languageStringDescriptor [LANG_DESCRIPTOR_LENGTH] __attribute__((aligned(4))) = {
	LANG_DESCRIPTOR_LENGTH,				 /* USB_LEN_LANGID_STR_DESC */
	0x03,    			/* USB_DESC_TYPE_STRING */
	LOBYTE(USBD_LANGID_STRING),
	HIBYTE(USBD_LANGID_STRING)
};

/* Vendor string descriptor */
const uint8_t manufactorStringDescriptor [MFC_DESCRIPTOR_LENGTH] __attribute__((aligned(4))) = {
	MFC_DESCRIPTOR_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	'S', 0x00,
	'T', 0x00,
	'M', 0x00,
	'i', 0x00,
	'c', 0x00,
	'r', 0x00,
	'o', 0x00,
	'e', 0x00,
	'l', 0x00,
	'e', 0x00,
	'c', 0x00,
	't', 0x00,
	'r', 0x00,
	'o', 0x00,
	'n', 0x00,
	'i', 0x00,
	'c', 0x00,
	's', 0x00
};

/* Product string descriptor */
const uint8_t productStringDescriptor [PRODUCT_DESCRIPTOR_LENGTH]  __attribute__((aligned(4))) = {
	PRODUCT_DESCRIPTOR_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	'S', 0x00,
	'T', 0x00,
	'M', 0x00,
	'3', 0x00,
	'2', 0x00,
	' ', 0x00,
	'V', 0x00,
	'i', 0x00,
	'r', 0x00,
	't', 0x00,
	'u', 0x00,
	'a', 0x00,
	'l', 0x00,
	' ', 0x00,
	'C', 0x00,
	'o', 0x00,
	'm', 0x00,
	'P', 0x00,
	'o', 0x00,
	'r', 0x00,
	't', 0x00
};

/*
 * Serial number string descriptor.
 * This buffer is intentionally mutable so a generic application can provide
 * its own stable product/manufacturing serial number when required. If the
 * application does not call USB_CDC_SetSerialNumber(), the driver populates
 * it from the STM32 96-bit UID during USB initialization.
 */
uint8_t serialNumberStringDescriptor[SERIAL_DESCRIPTOR_LENGTH] __attribute__((aligned(4)));
static uint8_t serialNumberIsUserDefined = 0U;

const uint8_t stringInterface [INTERFACE_STRING_LENGTH] __attribute__((aligned(4))) = {
	INTERFACE_STRING_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	'C', 0x00,
	'D', 0x00,
	'C', 0x00,
	' ', 0x00,
	'I', 0x00,
	'n', 0x00,
	't', 0x00,
	'e', 0x00,
	'r', 0x00,
	'f', 0x00,
	'a', 0x00,
	'c', 0x00,
	'e', 0x00
};

const uint8_t configurationStringDescriptor [CONFIG_STRING_LENGTH] __attribute__((aligned(4))) = {
	CONFIG_STRING_LENGTH,
	0x03,	/* USB_DESC_TYPE_STRING */
	'C', 0x00,
	'D', 0x00,
	'C', 0x00,
	' ', 0x00,
	'C', 0x00,
	'o', 0x00,
	'n', 0x00,
	'f', 0x00,
	'i', 0x00,
	'g', 0x00
};


/****************************************************************************/

static uint32_t device_state = DEVICE_STATE_DEFAULT; /* Device state */

EndPointStruct EndPoint[EP_COUNT];	/* All the Enpoints are included in this array */

static USB_setup_req_data setup_pkt_data; /* Setup Packet var */

/****************************************************************
 * 		static functions' declarations
*****************************************************************/

/* Device state */
static inline void set_device_status(eDeviceState state);

/* Init EP */
static void initEndPoints(void);

/* EP routine */
static inline uint32_t is_tx_ep_ready(uint8_t EPnum);

static void Get_ID_To_String(uint8_t *dest); // Build default serial number from STM32 UID
static uint8_t USB_CDC_prepare_EP1_OUT_DMA(void);

static circBufferAddress peek_circBufferRx(uint16_t len);
static void commit_circBufferRx(uint16_t len);

/**********************************************************************
 * GetSysTick() is required for Timeout detection in several functions
 * you can add msTicks++ in your SysTick_Handler if you use it already
 * or you can provide your own Timeout function (eg with a timer)
 **********************************************************************/

/* Weak implementation so the user can override it in main.c or application code */
__WEAK uint32_t GetSysTick(void) {
    extern volatile uint32_t msTicks; // Or return HAL_GetTick() / custom timer tick
    return msTicks;
}


/****************************************************
* 		Circular buffer*
***************************************************/

__attribute__((aligned(4))) static uint8_t circBufferTx[CIRC_BUFFER_TX_SIZE];
static volatile uint16_t writePtrTxCbuf = 0U;
static volatile uint16_t readPtrTxCbuf = 0U;

/*
 * EP1 IN DMA staging buffer.
 *
 * Always 4-byte aligned and maximum one HS bulk packet.
 * We copy from circBufferTx here before starting USB DMA.
 */
__attribute__((aligned(4))) static uint8_t ep1TxDmaBuffer[MAX_CDC_EP1_TX_SIZ];

/*
 * Information about the transfer currently owned by USB DMA.
 * The circular-buffer read pointer is advanced only after XFRC.
 */
static volatile uint16_t ep1TxPendingLen = 0U;

__attribute__((aligned(4))) uint8_t circBufferRx[CIRC_BUFFER_RX_SIZE];
static volatile uint16_t writePtrRxCbuf = 0U;  // make is static again if ISR functions included in this page
static volatile uint16_t readPtrRxCbuf = 0U; // make is static again if ISR functions included in this page

/*
 * EP1 OUT DMA staging buffer.
 *
 * USB DMA always writes to this aligned buffer. The received data is copied
 * to circBufferRx only after the USB transfer has completed. This avoids
 * unaligned DMA addresses and circular-buffer wrap-around crossing the end
 * of the array.
 */
__attribute__((aligned(4))) static uint8_t ep1RxDmaBuffer[RX_BUFFER_EP1_SIZE];
static volatile uint16_t ep1RxPendingIndex = 0U;


static inline uint32_t is_circBufferTx_empty(){
	return (readPtrTxCbuf == writePtrTxCbuf);
}

static inline uint32_t get_circBufferTx_freeSize(){
	/* Snapshot pointers to avoid interrupt race conditions - the pointers aren't modified, so no need to __disable_irq() here */
	uint16_t w = writePtrTxCbuf; // copying the values of volatile global variables into local variables at the beginning of a function.
	uint16_t r = readPtrTxCbuf; // creates a "frozen" version of the state that won't change while your math is running, even if an interrupt fires in the middle of your calculation.

	// We return SIZE - 1 to ensure we always leave one gap (the "N-1" Rule for full buffer)
	if (w == r) return CIRC_BUFFER_TX_SIZE - 1; // Empty
	if (w > r) {
		// Data is contiguous: free space is the two ends
		return CIRC_BUFFER_TX_SIZE - (w - r) - 1 ;
	} else {
		// Data is wrapped: free space is the gap in the middle
		return (r - w) - 1;
	}
}

static inline uint32_t get_circBufferRx_freeSize(){
	/* Snapshot pointers to avoid interrupt race conditions - the pointers aren't modified, so no need to __disable_irq() here */
	uint16_t w = writePtrRxCbuf; // copying the values of volatile global variables into local variables at the beginning of a function  creates a "frozen"
	uint16_t r = readPtrRxCbuf; // version of the state that won't change while your math is running, even if an interrupt fires in the middle of your calculation.

	// We return SIZE - 1 to ensure we always leave one gap (the "N-1" Rule for full buffer)
	if (w == r) return CIRC_BUFFER_RX_SIZE - 1; // Empty
	if (w > r) {
		// Data is contiguous: free space is the two ends
		return CIRC_BUFFER_RX_SIZE - (w - r) - 1;
	} else {
		// Data is wrapped: free space is the gap in the middle
		return (r - w) - 1;
	}
}


void write_to_circBufferTx(uint8_t *buf, uint16_t len){

	if (len == 0) return;

	// Check if we need to wrap around
	if((writePtrTxCbuf + len) >= CIRC_BUFFER_TX_SIZE){
		uint32_t write_tail = (uint32_t)((CIRC_BUFFER_TX_SIZE) - writePtrTxCbuf); // Calculate how much fits at the end of the array
		memcpy(&circBufferTx[writePtrTxCbuf], buf, write_tail); // Copy the tail part

		uint32_t remaining = (uint32_t)(len - write_tail); // Copy the remaining part to the beginning of the array
		if (remaining > 0) {
			memcpy(&circBufferTx[0], (buf + write_tail), remaining);
		}
		writePtrTxCbuf = (uint16_t)remaining; // Update pointer with modulo or manual wrap
	}
	else{
		memcpy(&circBufferTx[writePtrTxCbuf], buf, len); // Simple contiguous copy
		writePtrTxCbuf = (uint16_t)(writePtrTxCbuf + len); // Update pointer
	}
	// Safety check: if the pointer exactly hits the end of the array, wrap it to 0
	if (writePtrTxCbuf == CIRC_BUFFER_TX_SIZE) writePtrTxCbuf = 0;
}


/**
* brief  Return  current index and length of datas from the circular buffer
* param  Maximum length to get. For Example, if len=256, the function returns 256 length string index, if used space > 256.
* param  If used space < x, the function returns used space.
* retval First character's index and length
*/
static circBufferAddress peek_circBufferTx(uint16_t len) {

	circBufferAddress result = {0, 0};

	uint16_t w = writePtrTxCbuf; // Snapshot
	uint16_t r = readPtrTxCbuf;

	// 1. Check if truly empty
	if (w == r) return result;

	// 2. Calculate EXACT data count (Direct Math)
	uint16_t writtenSize;
	if (w > r) 	writtenSize = w - r;
	else writtenSize = (CIRC_BUFFER_TX_SIZE - r) + w;

	if (len > writtenSize) len = writtenSize;  // Constrain requested len to available data

	result.index = r;

	// 4. Wrap-around logic (note: Use '>' not '>=' because index starts at 0)
	// If r + len == SIZE, the last byte is at SIZE-1, which is perfectly valid.
	if ((r + len) > CIRC_BUFFER_TX_SIZE) {
		// Tell the USB driver to only send the bytes that actually exist at the end of the buffer.
		result.len = (uint16_t)(CIRC_BUFFER_TX_SIZE - r);
	} else {
		result.len = len; // The data fits perfectly within the remaining space of the array, it returns the full requested len
	}

	/*
	 * IMPORTANT: do not advance readPtrTxCbuf here.
	 * USB DMA owns the transfer until the EP1 XFRC interrupt.
	 */

	return result;
}

static void commit_circBufferTx(uint16_t len) {

	if (len == 0) return;

	uint16_t nextReadPtr = (uint16_t)(readPtrTxCbuf + len);
	if (nextReadPtr >= CIRC_BUFFER_TX_SIZE) nextReadPtr -= CIRC_BUFFER_TX_SIZE;

	readPtrTxCbuf = nextReadPtr;
}

circBufferAddress peek_circBufferRx(uint16_t requested_Len) {
/*	If the requested amount crosses the end of the array, peek_circBufferRx() returns only the bytes available before the boundary.
	Therefore, the application should consume the RX buffer inside a loop until no data remains*/

    circBufferAddress result = {0, 0};

    /* Snapshot the pointers. The RX write pointer may be advanced by the USB ISR,
     * but readPtrRxCbuf is owned by the application until commit_circBufferRx(). */
    uint16_t w = writePtrRxCbuf;
    uint16_t r = readPtrRxCbuf;

    /* Direct data count (N-1 compliant). */
    uint16_t actualData;
    if (w == r) {
        return result; // Truly empty
    }
    else if (w > r) {
        actualData = w - r;
    }
    else {
        actualData = (CIRC_BUFFER_RX_SIZE - r) + w;
    }

    /* Clamp requested length to the data currently available. */
    uint16_t len = (requested_Len > actualData) ? actualData : requested_Len;
    result.index = r;

    /* Return only a contiguous block. The application must call peek again after
     * commit_circBufferRx() to consume data on the other side of the wrap. */
    if ((r + len) > CIRC_BUFFER_RX_SIZE) {
        result.len = (uint16_t)(CIRC_BUFFER_RX_SIZE - r);
    }
    else {
        result.len = len;
    }

    /* IMPORTANT: do not advance readPtrRxCbuf here.
     * The application owns this data until it has finished copying/processing it. */
    return result;
}

void commit_circBufferRx(uint16_t len) {

    if (len == 0) return;

    /* The application has finished using the previously peeked data.
     * Only now may the consumed space become available to the USB RX DMA. */
    __disable_irq();

    uint16_t nextReadPtr = (uint16_t)(readPtrRxCbuf + len);
    if (nextReadPtr >= CIRC_BUFFER_RX_SIZE) {
        nextReadPtr = 0;
    }
    readPtrRxCbuf = nextReadPtr;

    /* Re-arm EP1 OUT only after the application has released the consumed data. */
    USB_CDC_prepare_EP1_OUT_DMA();

    __enable_irq();
}


/***************************************************
*
* 	Initialization functions
*
***************************************************/

void SystemClock_Config(void)
{
    /* Enable Power Control clock & set regulator voltage scaling */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS; // Scale 1 mode (up to 180 MHz)

    /* Enable HSE (High-Speed External Crystal) */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    /* Enable Over-Drive Mode (STM32F446 specific) */
    PWR->CR |= PWR_CR_ODEN;
    while (!(PWR->CSR & PWR_CSR_ODRDY));
    PWR->CR |= PWR_CR_ODSWEN;
    while (!(PWR->CSR & PWR_CSR_ODSWRDY));

    /* Configure Flash latency (5 Wait States for 180 MHz @ 3.3V) */
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

    /* Configure Main PLL (e.g., for 180 MHz SYSCLK from 8 MHz HSE) */
    /* PLLM = 8, PLLN = 360, PLLP = 2 -> 180 MHz */
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos)
                 | (360 << RCC_PLLCFGR_PLLN_Pos)
                 | (0 << RCC_PLLCFGR_PLLP_Pos) // Div 2
                 | RCC_PLLCFGR_PLLSRC_HSE;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* Configure AHB/APB prescalers (AHB=1, APB1=4, APB2=2) */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    /* Select PLL as System Clock Source */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    // The USB clock comes from the external 60 MHz crystal on the USB3300 (ULPI_CK) -> no need of internal 48 MHz PLL for USB
}

uint32_t USB_OTG_HS_GPIO_Init(void)
{
    /* 1. Enable Clocks for GPIOA, GPIOB, GPIOC, and OTG HS + ULPI */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
                 |  RCC_AHB1ENR_GPIOBEN
                 |  RCC_AHB1ENR_GPIOCEN
                 |  RCC_AHB1ENR_OTGHSEN
                 |  RCC_AHB1ENR_OTGHSULPIEN;

    /* 2. Release USB3300 Hardware Reset (PA6) after a small delay (50ms)
     * Set PA6 as Output-Low to release PHY reset and enable 60 MHz CLKOUT.
     */


    GPIOA->MODER   &= ~(0x3 << (6 * 2));
    GPIOA->MODER   |=  (0x1 << (6 * 2));  // General purpose output
    GPIOA->OTYPER  &= ~(1 << 6);          // Push-pull
    GPIOA->OSPEEDR |=  (0x3 << (6 * 2));  // Very High Speed
    GPIOA->BSRR     =  (1 << 6);   // Drive High (Reset)
    for (volatile uint32_t i = 0; i < 600000; i++) { __NOP(); }
    GPIOA->BSRR     =  (1 << (6 + 16));   // Drive Low (Release Reset)

    /* Brief delay for USB3300 clock (ULPI_CK) to stabilize */
    for (volatile int i = 0; i < 10000; i++);

    /* 3. Configure ULPI GPIO Pins to AF10 (Very High Speed)
     * PA3 (D0), PA5 (CK)
     * PB0 (D1), PB1 (D2), PB2 (D4), PB5 (D7), PB10 (D3), PB12 (D5), PB13 (D6)
     * PC0 (STP), PC2 (DIR), PC3 (NXT)
     */
    // Set MODER to Alternate Function (0b10)
    GPIOA->MODER &= ~((0x3 << (3 * 2)) | (0x3 << (5 * 2)));
    GPIOA->MODER |=  ((0x2 << (3 * 2)) | (0x2 << (5 * 2)));

    GPIOB->MODER &= ~((0x3 << (0 * 2))  | (0x3 << (1 * 2))  | (0x3 << (2 * 2))  | (0x3 << (5 * 2)) |
                      (0x3 << (10 * 2)) | (0x3 << (12 * 2)) | (0x3 << (13 * 2)));
    GPIOB->MODER |=  ((0x2 << (0 * 2))  | (0x2 << (1 * 2))  | (0x2 << (2 * 2))  | (0x2 << (5 * 2)) |
                      (0x2 << (10 * 2)) | (0x2 << (12 * 2)) | (0x2 << (13 * 2)));

    GPIOC->MODER &= ~((0x3 << (0 * 2)) | (0x3 << (2 * 2)) | (0x3 << (3 * 2)));
    GPIOC->MODER |=  ((0x2 << (0 * 2)) | (0x2 << (2 * 2)) | (0x2 << (3 * 2)));

    // Set OSPEEDR to Very High Speed (0b11)
    GPIOA->OSPEEDR |= ((0x3 << (3 * 2)) | (0x3 << (5 * 2)));
    GPIOB->OSPEEDR |= ((0x3 << (0 * 2))  | (0x3 << (1 * 2))  | (0x3 << (2 * 2))  | (0x3 << (5 * 2)) |
                       (0x3 << (10 * 2)) | (0x3 << (12 * 2)) | (0x3 << (13 * 2)));
    GPIOC->OSPEEDR |= ((0x3 << (0 * 2))  | (0x3 << (2 * 2))  | (0x3 << (3 * 2)));

    // Map Alternate Function AF10 (0xA) for all ULPI pins
    GPIOA->AFR[0] &= ~((0xF << (3 * 4)) | (0xF << (5 * 4)));
    GPIOA->AFR[0] |=  ((0xA << (3 * 4)) | (0xA << (5 * 4)));

    // PB0, PB1, PB2, PB5 are in AFR[0]
    GPIOB->AFR[0] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)) | (0xF << (2 * 4)) | (0xF << (5 * 4)));
    GPIOB->AFR[0] |=  ((0xA << (0 * 4)) | (0xA << (1 * 4)) | (0xA << (2 * 4)) | (0xA << (5 * 4)));

    // PB10, PB12, PB13 are in AFR[1]
    GPIOB->AFR[1] &= ~((0xF << ((10 - 8) * 4)) | (0xF << ((12 - 8) * 4)) | (0xF << ((13 - 8) * 4)));
    GPIOB->AFR[1] |=  ((0xA << ((10 - 8) * 4)) | (0xA << ((12 - 8) * 4)) | (0xA << ((13 - 8) * 4)));

    GPIOC->AFR[0] &= ~((0xF << (0 * 4)) | (0xF << (2 * 4)) | (0xF << (3 * 4)));
    GPIOC->AFR[0] |=  ((0xA << (0 * 4)) | (0xA << (2 * 4)) | (0xA << (3 * 4)));

    return 0;
}


uint32_t USB_OTG_HS_Core_Init(void)
{
	// device_state = DEVICE_STATE_DEFAULT;

    /* 1. Wait for AHB master Idle state before reset */
    while (!(USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));

    /* 2. Configure GUSBCFG for external ULPI PHY operation */
    uint32_t gusbcfg = USB_OTG_HS->GUSBCFG;

    gusbcfg &= ~(USB_OTG_GUSBCFG_PHYLPCS | USB_OTG_GUSBCFG_PTCI | USB_OTG_GUSBCFG_PHYSEL);
    gusbcfg |=  USB_OTG_GUSBCFG_ULPIFSLS; // Select external ULPI PHY

    /* Set ULPI turnaround time (TRDT = 9 for High Speed @ 60MHz ULPI) */
    gusbcfg &= ~USB_OTG_GUSBCFG_TRDT;
    gusbcfg |=  (9 << USB_OTG_GUSBCFG_TRDT_Pos);

    /* Force Device Mode */
    gusbcfg &= ~USB_OTG_GUSBCFG_FHMOD;
    gusbcfg |=  USB_OTG_GUSBCFG_FDMOD;

    USB_OTG_HS->GUSBCFG = gusbcfg;

    /* 3. Issue Core Soft Reset */
    USB_OTG_HS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    while (USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_CSRST);

    /* Wait 3 PHY clocks after soft reset */
    for (volatile uint32_t i = 0; i < 1000; i++);

    /* 4. Configure Device Mode in Device Configuration Register */
    /* Pointer to OTG HS Device-specific register block */
    USB_OTG_DeviceTypeDef *USB_OTG_HS_DEV = ((USB_OTG_DeviceTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE));
    USB_OTG_HS_DEV->DCFG &= ~USB_OTG_DCFG_DSPD; // 00: Set Speed to High Speed (ULPI 60 MHz)

    /* 5. Enable DMA mode in AHB Configuration Register */
    uint32_t gahbcfg = USB_OTG_HS->GAHBCFG;
    gahbcfg |= USB_OTG_GAHBCFG_DMAEN; // Enable DMA transfer mode
    USB_OTG_HS->GAHBCFG = gahbcfg;

    return 0;
}


void USB_OTG_HS_FIFO_and_Interrupts_Init(void)
{
	/* Populate the default UID serial unless the application supplied a custom
	 * serial number before USB initialization. */
	if (!serialNumberIsUserDefined)
		Get_ID_To_String(serialNumberStringDescriptor);

    /* 1. Configure FIFO Sizes (measured in 32-bit words) */

    // RX FIFO: Start @ 0, Depth = 512 words (2048 bytes)
    USB_OTG_HS->GRXFSIZ = 512;

    // EP0 TX FIFO: Start @ 512, Depth = 64 words (256 bytes)
    USB_OTG_HS->DIEPTXF0_HNPTXFSIZ = (64 << USB_OTG_TX0FD_Pos) | 512;

    // EP1 TX FIFO (CDC Bulk IN): Start @ 576 (512 + 64), Depth = 256 words (1024 bytes)
    USB_OTG_HS->DIEPTXF[0] = (256 << USB_OTG_DIEPTXF_INEPTXFD_Pos) | (512 + 64);

    // EP2 TX FIFO (CDC notification IN): Start @ 832, Depth = 16 words (64 bytes).
    // Total allocation = 512 + 64 + 256 + 16 = 848 words, below the 1024-word HS FIFO RAM.
    USB_OTG_HS->DIEPTXF[1] = (16 << USB_OTG_DIEPTXF_INEPTXFD_Pos) | (512 + 64 + 256);

    /* 2. Clear pending interrupts */
    USB_OTG_HS->GINTSTS = 0xFFFFFFFF; // Global Interrupt Status Register

    /* 3. Unmask required core interrupts */
    /* USB_OTG_HS->GINTMSK = USB_OTG_GINTMSK_RXFLVLM  | // RX FIFO Non-Empty (FIFO Level Mask)
                          USB_OTG_GINTMSK_USBRST   | // USB Reset
                          USB_OTG_GINTMSK_ENUMDNM  | // Enumeration Done
                          USB_OTG_GINTMSK_IEPINT   | // IN Endpoint Interrupt
                          USB_OTG_GINTMSK_OEPINT   | // OUT Endpoint Interrupt
                          USB_OTG_GINTMSK_WUIM;      // Resume/Wakeup Interrupt */
    USB_OTG_HS->GINTMSK = USB_OTG_GINTMSK_USBRST   | // USB Reset
    					  USB_OTG_GINTMSK_ENUMDNEM | // Enumeration Done
                          USB_OTG_GINTMSK_IEPINT   | // IN Endpoint Interrupt
                          USB_OTG_GINTMSK_OEPINT   | // OUT Endpoint Interrupt
                          USB_OTG_GINTMSK_WUIM;      // Resume/Wakeup Interrupt

    /* 4. Enable Global Interrupts in OTG Core */
    USB_OTG_HS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

    /* 5. Enable OTG_HS IRQ in NVIC */
    NVIC_SetPriority(OTG_HS_IRQn, 5);
    NVIC_EnableIRQ(OTG_HS_IRQn);
}

void USB_OTG_HS_Connect(void)
{
    USB_OTG_DeviceTypeDef *USB_OTG_HS_DEV = ((USB_OTG_DeviceTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE));

    // Clear Soft Disconnect bit to pull D+ high via ULPI PHY (connect device to the bus)
    USB_OTG_HS_DEV->DCTL &= ~USB_OTG_DCTL_SDIS; // // Host will now detect the device
}

void USB_OTG_HS_Disconnect(void)
{
    USB_OTG_DeviceTypeDef *USB_OTG_HS_DEV = ((USB_OTG_DeviceTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE));

    // Set Soft Disconnect bit to pull D+ low/tri-state via ULPI PHY (disconnect device from the bus)
    USB_OTG_HS_DEV->DCTL |= USB_OTG_DCTL_SDIS; // Host will detect the device disconnection
}

/**
* brief  fill endpoint structures with initial data
* param
* param
* retval
*/
static uint8_t USB_CDC_prepare_EP1_OUT_DMA(void)
{
	uint16_t w = writePtrRxCbuf;
	uint16_t r = readPtrRxCbuf;

	/*
	 * If EP1 OUT DMA is already active, don't touch the hardware
	 * configuration. This function can safely be called both from the
	 * USB ISR and after the application consumes RX data.
	 */
	if (USB_EP_OUT(1)->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
		return 1;

	/* Keep one byte unused in the circular buffer (N-1 rule).
	 * DMA receives into a dedicated, aligned 512-byte staging buffer.
	 */
	if (get_circBufferRx_freeSize() < RX_BUFFER_EP1_SIZE)
		return 0;

	if (w >= r) {
		if ((CIRC_BUFFER_RX_SIZE - w) < RX_BUFFER_EP1_SIZE) {
			/* Not enough contiguous space until the end; wrap to the beginning. */
			if (r >= RX_BUFFER_EP1_SIZE) {
				w = 0;
				writePtrRxCbuf = 0;
			} else {
				return 0;
			}
		}
	} else {
		/* Free space is contiguous between w and r. */
		if ((r - w) <= RX_BUFFER_EP1_SIZE)
			return 0;
	}

	/* Remember where the completed DMA packet will be committed. */
	ep1RxPendingIndex = w;

	USB_EP_OUT(1)->DOEPTSIZ =
		(1U << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | RX_BUFFER_EP1_SIZE;

	/* USB DMA always uses the aligned staging buffer. */
	USB_EP_OUT(1)->DOEPDMA = (uint32_t)ep1RxDmaBuffer;

	USB_EP_OUT(1)->DOEPCTL |=
		(USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);

	return 1;
}

static void initEndPoints(){
	for (uint32_t i = 0; i < EP_COUNT; i++) {
		/* Global defaults for all Endpoints */
		EndPoint[i].statusRx  = EP_READY;
		EndPoint[i].statusTx  = EP_READY;
		EndPoint[i].rxCounter = 0;
		EndPoint[i].txCounter = 0; // Decrements the remaining bytes to track the transfer progress
		EndPoint[i].setTxBuffer  = &USB_CDC_setTxBuffer;
		EndPoint[i].txCallBack   = &USB_CDC_transferTXCallback;
	}

	/* --- EP0: Control (Shared) --- */
	EndPoint[0].rxBuffer_ptr = rxBufferEp0;
	EndPoint[0].rxCallBack   = &USB_CDC_transferRXCallback_EP0; // Handles Setup packets

	/* --- EP1: CDC Data (Bulk IN/OUT) --- */
	EndPoint[1].rxCallBack   = &USB_CDC_transferRXCallback_EP1;

	/* --- Hardware: Setup EP0 OUT for DMA Reception --- */
	uint32_t ep0_pktcnt = (RX_BUFFER_EP0_SIZE + MAX_CDC_EP0_TX_SIZ - 1) / MAX_CDC_EP0_TX_SIZ;
	USB_EP_OUT(0)->DOEPTSIZ = (ep0_pktcnt << 19)
										| RX_BUFFER_EP0_SIZE
										| USB_OTG_DOEPTSIZ_STUPCNT; // Keeps Setup packet counting capability

	// Point EP0 DMA directly to rxBufferEp0
	USB_EP_OUT(0)->DOEPDMA  = (uint32_t)(rxBufferEp0);

	// Clear NAK and enable EP0 OUT with DMA active
	USB_EP_OUT(0)->DOEPCTL  |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);

	/* --- Hardware: Setup EP1 OUT (Bulk) for DMA Reception --- */
	USB_CDC_prepare_EP1_OUT_DMA();

	/* --- Hardware: EP2 IN CDC notification endpoint --- */
	USB_EP_IN(USB_CDC_NOTIFICATION_EP)->DIEPCTL = USB_OTG_DIEPCTL_SNAK |
		USB_OTG_DIEPCTL_TXFNUM_1 | /* TX FIFO number 2 */
		USB_OTG_DIEPCTL_EPTYP_0 | /* Interrupt endpoint */
		USB_OTG_DIEPCTL_USBAEP |
		USB_CDC_NOTIFICATION_MPS;
	/* EP2 remains NAKed until a Serial-State notification is implemented. */

	USB_OTG_HS->GINTSTS = 0xFFFFFFFF; 			 	// Reset Global Interrupt status (core interrupt register OTG_GINTSTS)
	USB_OTG_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;     // Clearing the bit (~) releases the soft disconnect, effectively "plugging" the device back in virtually
}



/***************************************************
*
* 	Miscellaneous service functions
*
***************************************************/

static void Get_ID_To_String(uint8_t *dest) {
	/* Default serial source: STM32 96-bit UID represented as 24 hex characters. */
	uint8_t *uid = (uint8_t *)0x1FFF7A10; // STM32F446 96-bit UID address
	const char hex_table[] = "0123456789ABCDEF";

	dest[0] = SERIAL_DESCRIPTOR_LENGTH;
	dest[1] = 0x03; // USB_DESC_TYPE_STRING

	for (uint8_t i = 0U; i < 12U; i++) {
		uint8_t byte = uid[i];
		dest[2U + (uint16_t)i * 4U] = hex_table[byte >> 4];
		dest[3U + (uint16_t)i * 4U] = 0x00;
		dest[4U + (uint16_t)i * 4U] = hex_table[byte & 0x0FU];
		dest[5U + (uint16_t)i * 4U] = 0x00;
	}
}

uint32_t USB_CDC_SetSerialNumber(const char *serial) {
	/* Maximum 24 ASCII characters fit in the 50-byte UTF-16LE descriptor. */
	uint32_t len = 0U;

	if (serial == NULL)
		return 0U;

	while (serial[len] != '\0') {
		if (len >= SERIAL_NUMBER_MAX_CHARS)
			return 0U;
		len++;
	}

	if (len == 0U)
		return 0U;

	serialNumberStringDescriptor[0] = (uint8_t)(2U + len * 2U);
	serialNumberStringDescriptor[1] = 0x03; // USB_DESC_TYPE_STRING

	for (uint32_t i = 0U; i < len; i++) {
		serialNumberStringDescriptor[2U + i * 2U] = (uint8_t)serial[i];
		serialNumberStringDescriptor[3U + i * 2U] = 0x00;
	}

	for (uint32_t i = 2U + len * 2U; i < SERIAL_DESCRIPTOR_LENGTH; i++)
		serialNumberStringDescriptor[i] = 0x00;

	serialNumberIsUserDefined = 1U;
	return EP_OK;
}

void USB_CDC_ForceResetState(void) {
	/*	A "Force Reset" means manually clearing the hardware's status registers, flushing the FIFO buffers,
		and re-enabling the "listening" state for the next incoming packet. */

    // 1. Atomic lock, protect against the USB Interrupt firing while we are resetting its world.
    __disable_irq();

    // 2. Software state reset
    readPtrRxCbuf = 0;
    writePtrRxCbuf = 0;
    readPtrTxCbuf = 0;
    writePtrTxCbuf = 0;
    ep1TxPendingLen = 0;
    ep1RxPendingIndex = 0;
    EndPoint[0].txCounter = 0;
    EndPoint[0].statusTx = EP_READY;
    EndPoint[0].totXferLen = 0;
    EndPoint[1].txCounter = 0;
    EndPoint[1].statusTx = EP_READY;
    EndPoint[1].totXferLen = 0;
    EndPoint[2].txCounter = 0;
    EndPoint[2].statusTx = EP_READY;
    EndPoint[2].totXferLen = 0;

    // 3. Harware FIFO flush : Flush TX FIFO 0 (or whichever FIFO your IN endpoint uses)
    USB_OTG_HS->GRSTCTL = (1 << 5) | (16 << 6); // 0x20: TxFIFO Flush + all Tx FIFO
    while (USB_OTG_HS->GRSTCTL & (1 << 5));    // Wait for hardware to finish flushing

    // Flush ALL RX FIFOs
    USB_OTG_HS->GRSTCTL = (1 << 4);            // 0x10: RxFIFO Flush
    while (USB_OTG_HS->GRSTCTL & (1 << 4));    // Wait for hardware to finish flushing

    // 4. Clear stuck interrupts: Clear any pending transfer complete or error flags for EP1 (your CDC data EP)
    // Writing 1 to these bits usually clears them in CMSIS/Bare-metal
    USB_EP_OUT(1)->DOEPINT = 0xFF;
    USB_EP_IN(1)->DIEPINT  = 0xFF;
    USB_EP_IN(USB_CDC_NOTIFICATION_EP)->DIEPINT  = 0xFF;

    // 5. RE-PRIME THE RECEIVE ENDPOINT (The "Unstick" Step)
    // If a big packet caused a NAK, try to start listening again for a fresh packet.
    // Use the same buffer-space check as the normal EP1 OUT path so a reset
    // can never arm DMA when the circular buffer cannot accept one full packet.
    USB_CDC_prepare_EP1_OUT_DMA();

    // 6. Synchronize our software flag with the fresh hardware state and atomic unlock
    EndPoint[1].statusTx = EP_READY;

    __enable_irq();
}


/**
* brief  Send Zero Length Packet
* In CDC (and most USB protocols), a ZLP is used to terminate a data transfer when the last payload sent was exactly the size of the Maximum Packet Size (MPS)
* param
* param
* retval OK/FAILED
*/
void send_zlp(uint8_t EPnum) {

    EndPoint[EPnum].statusTx = EP_ZLP;  // Update status so the ISR cleanup logic knows what to do next

    // Set Transfer Size: 1 packet, 0 bytes. We do this in one write to ensure hardware atomic-like update
    USB_EP_IN(EPnum)->DIEPTSIZ = (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos) & USB_OTG_DIEPTSIZ_PKTCNT_Msk;
    __DMB(); // Ensure the register write is finished before enabling EP

    // Enable the endpoint and clear NAK to allow the Host to read the 0-byte packet
    USB_EP_IN(EPnum)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;

}

 /**
 * brief  Flush TxFifo
 * param  Fifo number, 10 = all Tx Fifos,
 * param  timeout (default FLUSH_FIFO_TIMEOUT)
 * retval 1 = OK, 0 = Failed
 */
uint32_t USB_FlushTxFifo(uint32_t EPnum, uint32_t timeout){
	uint32_t count = 0;
	USB_OTG_HS->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (EPnum << 6));
	do{
		if (++count > timeout){
			return EP_FAILED;
		}
	}
	while ((USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

	 return EP_OK;
}

 /**
 * brief  Flush RxFifo
 * param  timeout (default FLUSH_FIFO_TIMEOUT)
 * param
 * retval 1 = OK, 0 = Failed
 */
uint32_t USB_FlushRxFifo(uint32_t timeout){
	uint32_t count = 0;
	USB_OTG_HS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
	do{
		if (++count > timeout){
			return EP_FAILED;
		}
	}
	while ((USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);

	return EP_OK;
}

/****************************************************
* 		EndPoints' Callbacks*
***************************************************/

/**
* This Callback function is where the high-level logic meets the actual hardware registers.
* Take a buffer of any size, chop it into USB-compliant packets, and manage the ZLP logic required by the USB specification
* Atomic-Style Execution: Because you'll call this from the scheduler inside a __disable_irq() block, the registers are set,
* the DMA transfer is programmed and the hardware is ready before any other part of your code can interfere.
* brief  check Endpoint TX status
* param  EP number
* retval
*/
uint32_t USB_CDC_transferTXCallback(uint8_t EPnum){
    // 1. Wait for Hardware Readiness
    uint32_t start_tick = GetSysTick();
    while(USB_EP_IN(EPnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
    	if (GetSysTick() - start_tick > 5) return EP_FAILED;
    }

    uint16_t len = EndPoint[EPnum].txCounter;

    if (EPnum == 0) {
    	/* CONTROL HANDSHAKE / DESCRIPTORS - DMA-driven */
    	// Calculate packets: (67 + 63) / 64 = 2 packets
    	uint32_t pktcnt = (len == 0) ? 1 : (len + MAX_CDC_EP0_TX_SIZ - 1) / MAX_CDC_EP0_TX_SIZ;

    	// Hardware setup for exactly what is in the buffer
    	// DIEPTSIZ: [PKTCNT (bits 20:19)] | [XFRSIZ (bits 18:0)]
    	USB_EP_IN(0)->DIEPTSIZ = (pktcnt << 19) | len;

    	// 2. Point DMA engine directly to the control buffer source address
    	USB_EP_IN(0)->DIEPDMA = (uint32_t)(EndPoint[0].txBuffer_ptr);

     	__DMB();

    	// 3. Arm EP and trigger the hardware DMA transfer immediately across the AHB bus
    	USB_EP_IN(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
    }
    else {
    	/* DATA EP1 - DMA MODE */
    	EndPoint[1].statusTx = EP_BUSY;
    	EndPoint[1].totXferLen = len;

    	uint32_t total = len;
    	uint32_t pktcnt = (total == 0) ? 1 : (total + (MAX_CDC_EP1_TX_SIZ - 1)) / MAX_CDC_EP1_TX_SIZ;

    	// 1. Configure Transfer Size and Packet Count
    	USB_EP_IN(EPnum)->DIEPTSIZ = (pktcnt << 19) | total;

    	// 2. Point DMA engine directly to the source buffer (must be 32-bit aligned)
    	// Note: EndPoint[EPnum].txBuffer_ptr points to the aligned EP1 DMA staging buffer.
    	// You supply the exact source pointer address:
    	USB_EP_IN(EPnum)->DIEPDMA = (uint32_t)(EndPoint[EPnum].txBuffer_ptr);

     	__DMB();

    	__DMB(); // Ensure the DMA source copy and DMA address are visible before EPENA

    	// 3. Arm EP and trigger the hardware DMA transfer (firing the endpoint enable EPENA) immediately across the AHB bus
    	USB_EP_IN(EPnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
    	// No manual FIFO servicing is needed: the hardware DMA engine streams the whole payload directly from RAM
    }
    return EP_OK;
}


/*
 * brief  Set and start TX transaction
* param  EP number, TX Buffer, length
* retval OK/FAILED
*/
uint32_t USB_CDC_setTxBuffer(uint8_t EPnum, uint8_t *txBuff, uint16_t len){

	/************** Safety check - Previous transaction is not finished ****************/
	if((EndPoint[EPnum].txCounter != 0) || (EndPoint[EPnum].statusTx == EP_ZLP)
										|| (check_USB_device_status(DEVICE_STATE_TX_PR) == EP_OK)) {
		return EP_FAILED;
	}

	// 1. REMOVE the 128-byte cap check!
	// The hardware DIEPTSIZ register can handle up to 512KB. Let's allow len up to 4096 for now.
	if(len > 4096) return EP_FAILED;

	/* Set data to send */
	if(len != 0){
		EndPoint[EPnum].txBuffer_ptr = txBuff; 	// *txBuff points to the first index to read on the circular buffer
		EndPoint[EPnum].txCounter = len; //
		/* SEND DATA (calling USB_CDC_transferTXCallback) */
		set_device_status(DEVICE_STATE_TX_PR);
		EndPoint[EPnum].txCallBack(EPnum);

		/* DMA owns the complete transfer; no TX FIFO-empty interrupt is required. */
		return EP_OK;
	}
	/* Zero-Length Packet (with EP0 after enumerate_Setup() send a zlp ACK) */
	else {
		// 1. Clear out any previous state (optional but safer in Turbo)
		USB_EP_IN(EPnum)->DIEPINT = 0xFF;
		// 2. Set SIZES FIRST (One single write)
		// This sets PKTCNT=1 and XFRSIZ=0 in one bus cycle
		USB_EP_IN(EPnum)->DIEPTSIZ = (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
		// 3. ENABLE LAST
		// This is the only time you should touch EPENA in this block
		USB_EP_IN(EPnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
		// 4. Arm the OUT endpoint
		USB_EP_OUT(EPnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);

		return EP_OK;
	}
}

/**
* brief  Perform some action with recieved data (EP0) and refresh EP buffer counter
* brief  For EP0 - Set lineCoding
* param  command
* retval
*/
uint32_t USB_CDC_transferRXCallback_EP0(uint32_t param){
	uint16_t len = EndPoint[0].rxCounter;

	// Safety checks
	if (len == 0) return EP_OK;
	if (len > CDC_LINE_CODING_LENGTH) len = CDC_LINE_CODING_LENGTH; // Stay in bounds
	if (EndPoint[0].statusRx == EP_BUSY) return EP_FAILED;

	if (param == CDC_SET_LINE_CODING) {
		// Overwrite our variable lineCoding with whatever the PC sent (7 bytes).
		// Even if the PC asks for 9600 baud or 115200, we just store it.
		memcpy(lineCoding, rxBufferEp0, CDC_LINE_CODING_LENGTH);

		// Clear the counter so we don't process the same data twice.
		EndPoint[0].rxCounter = 0;

		/* The OUT data stage is complete. Now send the required IN ZLP
		 * status stage for the control transfer. */
		EndPoint[0].setTxBuffer(0, NULL, 0);
	}
	return EP_OK;
}



/**
* brief  Perform some action with received data (EP1) and refresh EP buffer counter
* param  a command or a dummy param
* param
* retval
*/
uint32_t USB_CDC_transferRXCallback_EP1(uint32_t param){

	if(EndPoint[1].statusRx == EP_BUSY) return EP_FAILED;


	USB_CDC_UserRxCallBack_EP1((uint16_t)param); // Weak function called upon interrupt XFRC (Transfer completed) - use then read_circBuffer() to process

	return param;
}

/***************************************************
*			USB enumeration
***************************************************/

void enumerate_Reset(){

	/************************************************************/
	/* 1. CLEAN THE PIPES FIRST                                 */
	/************************************************************/
	USB_FlushRxFifo(2000);       // Clear the Global Receive FIFO
	USB_FlushTxFifo(0, 2048);    // Clear Control EP0 TX FIFO
	USB_FlushTxFifo(1, 2048);    // Clear CDC Data EP1 TX FIFO

	/************************************************************/
	/* 2. RESET SOFTWARE STATE                                  */
	/************************************************************/
	set_device_status(DEVICE_STATE_RESET);
	USB_OTG_HS->GINTSTS = 0xFFFFFFFF; // Clear interrupts

	/************************************************************/
	/* 3. RECONFIGURE ENDPOINTS 							    */
	/************************************************************/
	initEndPoints(); // Hardware-enable EP0 and reassert EP1/EP2 structure

	USB_OTG_HS->GINTSTS = 0xFFFFFFFF; // reset OTG core interrupt register

	/* OTG all endpoints interrupt mask register */
	USB_OTG_DEVICE->DAINTMSK = 0x00030000U | (1U << 0) | (1U << 1) | (1U << USB_CDC_NOTIFICATION_EP); // IEPINT-> IN EP0, EP1 & EP2 interrupts unmasked, OEPINT: OUT endpoint 0 & 1 interrupts unmasked
	USB_OTG_DEVICE->DOEPMSK  = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM; /* Unmask SETUP Phase done Mask,  TransfeR Completed interrupt for OUT */
	USB_OTG_DEVICE->DIEPMSK  = USB_OTG_DIEPMSK_XFRCM; /* TransfeR Completed interrupt for IN */

	USB_OTG_DEVICE->DCFG  &= ~USB_OTG_DCFG_DAD_Msk;  /* before Enumeration set address 0 */

	/* Endpoint 1 */
	USB_EP_IN(1)->DIEPCTL = USB_OTG_DIEPCTL_SNAK |
			USB_OTG_DIEPCTL_TXFNUM_0 |  /* TX Number 1 */
			USB_OTG_DIEPCTL_EPTYP_1 |  /* Eptype 10 means Bulk */
			USB_OTG_DIEPCTL_USBAEP |  /* Set Endpoint active */
			USB_CDC_MAX_PACKET_SIZE;  /* Max Packet size (bytes) */

	/* EP1 OUT DMA will be armed by initEndPoints() after the RX ring has been reset. */
	USB_EP_OUT(1)->DOEPTSIZ = 0;
	USB_EP_OUT(1)->DOEPCTL = USB_OTG_DOEPCTL_SNAK |
			USB_OTG_DOEPCTL_EPTYP_1 |
			USB_OTG_DOEPCTL_USBAEP |
			USB_CDC_MAX_PACKET_SIZE;

	/* EP2 is part of the CDC ACM descriptor even though the application does not
	 * currently send Serial-State notifications. Keep it NAKed until needed. */
	USB_EP_IN(USB_CDC_NOTIFICATION_EP)->DIEPCTL = USB_OTG_DIEPCTL_SNAK |
			USB_OTG_DIEPCTL_TXFNUM_1 |
			USB_OTG_DIEPCTL_EPTYP_0 |
			USB_OTG_DIEPCTL_USBAEP |
			USB_CDC_NOTIFICATION_MPS;
}


/**
 * brief  Handle all host requests, send all descriptors data
 * param
 * param
 * retval
 */
void enumerate_Setup(){
	// Combined request for your existing logic
	    uint16_t request = (setup_pkt_data.setup_pkt.bRequest << 8) | setup_pkt_data.setup_pkt.bmRequestType;
	    uint16_t len = setup_pkt_data.setup_pkt.wLength;
	    static uint8_t dest[128] __attribute__((aligned(4))); // Use static and keep the DMA source 4-byte aligned!

	switch(request){

	case REQ_TYPE_HOST_TO_DEVICE_GET_DEVICE_DECRIPTOR:
		switch(setup_pkt_data.setup_pkt.wValue){
		case DESCRIPTOR_TYPE_DEVICE: 				/* Request 0x0680  Value 0x0100 */
			if(DEVICE_DESCRIPTOR_LENGTH < len) len = DEVICE_DESCRIPTOR_LENGTH;
			memcpy(dest, deviceDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_CONFIGURATION: 			/* Request 0x0680  Value 0x0200 */
			if(CONFIGURATION_DESCRIPTOR_LENGTH < len) len = CONFIGURATION_DESCRIPTOR_LENGTH;
			memcpy(dest, configurationDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_DEVICE_QUALIFIER: 			/* Request 0x0680  Value 0x0600 */
			if(DEVICE_QUALIFIER_LENGTH < len) len = DEVICE_QUALIFIER_LENGTH;
			memcpy(dest, deviceQualifierDescriptor, len);
			//	return; /* CUBE MX CDC actually doesn't send any data here */
			break;
		case DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION: 	/* Request 0x0680  Value 0x0700 */
			// Returns the configuration structure that would apply if connected at FS (64-byte Bulk packets instead of 512)
		    if(CONFIGURATION_DESCRIPTOR_LENGTH < len) len = CONFIGURATION_DESCRIPTOR_LENGTH;
		    memcpy(dest, otherSpeedConfigurationDescriptor, len);
		    break;
		case DESCRIPTOR_TYPE_LANG_STRING: 			/* Request 0x0680  Value 0x0300 */
			if(LANG_DESCRIPTOR_LENGTH < len) len = LANG_DESCRIPTOR_LENGTH;
			memcpy(dest, languageStringDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_MFC_STRING: 			/* Request 0x0680  Value 0x0301 */
			if(MFC_DESCRIPTOR_LENGTH < len) len = MFC_DESCRIPTOR_LENGTH;
			memcpy(dest, manufactorStringDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_PROD_STRING: 			/* Request 0x0680  Value 0x0302 */
			if(PRODUCT_DESCRIPTOR_LENGTH < len) len = PRODUCT_DESCRIPTOR_LENGTH;
			memcpy(dest, productStringDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_SERIAL_STRING: 			/* Request 0x0680  Value 0x0303 */
			/* Either the default STM32 UID or the application-defined serial. */
			if (serialNumberStringDescriptor[0] < len)
				len = serialNumberStringDescriptor[0];
			memcpy(dest, serialNumberStringDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_CONFIGURATION_STRING: 		/* Request 0x0680  Value 0x0304 */
			if(CONFIG_STRING_LENGTH < len) len = CONFIG_STRING_LENGTH;
			memcpy(dest, configurationStringDescriptor, len);
			break;
		case DESCRIPTOR_TYPE_INTERFACE_STRING: 			/* Request 0x0680  Value 0x0305 */
			if(INTERFACE_STRING_LENGTH < len) len = INTERFACE_STRING_LENGTH;
			memcpy(dest, stringInterface, len);
			break;
		default:
			return;
		}
		break;

	case REQ_TYPE_DEVICE_TO_HOST_SET_ADDRESS: 				/* Request 0x0500  */
		len=0; /* ZLP */
		USB_OTG_DEVICE->DCFG &= ~((uint32_t)0x7F << 4); // Clear the 7-bit DAD field (bits 4 to 10) first!
		USB_OTG_DEVICE->DCFG |= (uint32_t)(setup_pkt_data.setup_pkt.wValue << 4);
		device_state = DEVICE_STATE_ADDRESSED;
		break;
	case REQ_TYPE_DEVICE_TO_HOST_SET_CONFIGURATION: 			/* Request 0x0900  */
		len=0; /* ZLP */
		USB_CDC_ForceResetState(); // When the PC assigns an address or sets the configuration, it will start a fresh session with this hardware
		device_state = DEVICE_STATE_CONFIGURED;
		break;

	case CDC_GET_LINE_CODING: 						/* Request 0x21A1  */
		if(CDC_LINE_CODING_LENGTH < len) len = CDC_LINE_CODING_LENGTH;
		memcpy(dest, lineCoding, len);
		set_device_status(DEVICE_STATE_LINECODED);
		break;

	case CDC_SET_LINE_CODING: 						/* Request 0x2021  */
		/* The status-stage ZLP is sent only after the OUT data stage
		 * has completed through EP0 DMA/XFRC. */
		return;
	case CDC_SET_CONTROL_LINE_STATE: /* Request 0x2221 - when click "Connect" in TeraTerm, the PC sends this request (Data Terminal Ready signal) */
		len=0;
		USB_CDC_ForceResetState(); // Ensures that if there is no "garbage" left in the circular buffer from a previous session
		break;
	case CLEAR_FEATURE_ENDP: 						/* Request 0x0201  */
		uint8_t ep_num = setup_pkt_data.setup_pkt.wIndex & 0x7F;
		uint8_t is_in = setup_pkt_data.setup_pkt.wIndex & 0x80;
		if (is_in) {
			// 1. Clear the STALL bit in DIEPCTL
			USB_EP_IN(ep_num)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
			// 2. Reset Data Toggle to DATA0
			USB_EP_IN(ep_num)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
		} else {
			// 1. Clear the STALL bit in DOEPCTL
			USB_EP_OUT(ep_num)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
			// 2. Reset Data Toggle to DATA0
			USB_EP_OUT(ep_num)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
		}
		len = 0; // Prepare ZLP for Status Phase
		break;   // Fall through to EndPoint[0].setTxBuffer
	default:
		// For unsupported control requests, stall EP0 IN and OUT
		// The USB spec dictates that the hw automatically clears the STALL condition on EP0 when a new Setup packet is received
		// When the host sees that an EP has been stalled, it will issue the standard control request CLEAR_FEATURE (specifically ENDPOINT_HALT)
		USB_EP_IN(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
		USB_EP_OUT(0)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
		break;
	}

	EndPoint[0].setTxBuffer(0, dest, len); // must be sent even if len == 0
}

/**
 * brief  Schedule a new transmission after a previous one finished
 * retval EP_FAILED if conditions are not met
 */
uint32_t USB_CDC_transmit_scheduler(){

	// if we are in the process of sending multiple packets or if Fifo is ready
	if(is_tx_ep_ready(1) == EP_OK) {
		if(!is_circBufferTx_empty())
		{
			// set the length of the string to send by peeking at the ring buffer
			circBufferAddress newMsgaddr = peek_circBufferTx(MAX_CDC_EP1_TX_SIZ); // MAX_CDC_EP1_TX_SIZ =
			if (newMsgaddr.len == 0) return EP_READY ;

			/*
			 * Save the transaction information. The circular-buffer read
			 * pointer is committed only after the USB DMA transfer completes.
			 */
			ep1TxPendingLen = newMsgaddr.len;

			/*
			 * DMA source must be 32-bit aligned. The circular-buffer index
			 * may point to any byte, so copy the contiguous chunk to the
			 * always-aligned staging buffer first.
			 */
			memcpy(ep1TxDmaBuffer,
			       &circBufferTx[newMsgaddr.index],
			       newMsgaddr.len);

			EndPoint[1].setTxBuffer(1, ep1TxDmaBuffer, newMsgaddr.len); // EndPoint[i].setTxBuffer = &USB_CDC_setTxBuffer;
			return EP_OK;
		}
	}
	return EP_FAILED;
}


/************************************************************/
/*************************** inline *************************/
/************************************************************/

/* Device status functions. Set/clear/check*/

static inline void set_device_status(eDeviceState state){
	device_state |= state;
}

void clear_USB_device_status(eDeviceState state){
	device_state &= ~state;
}

uint32_t check_USB_device_status(eDeviceState state){
	if(device_state & state){
		return EP_OK;
	}
	else return EP_FAILED;
}

/**
* brief  Check if TX FIFO is ready to push there data
* param  EP number
* param  param. If you use TX queue, "param" would be message count var pending in the queue
* param  if you have a var like message_counter, you use it here, otherwise use something > 0
* retval
*/

static inline uint32_t is_tx_ep_ready(uint8_t EPnum){
	if (!(device_state & DEVICE_STATE_TX_PR) &&
		!(USB_EP_IN(EPnum)->DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ) &&
		((USB_EP_IN(EPnum)->DIEPTSIZ & USB_OTG_HCTSIZ_PKTCNT) == 0) &&
		!(USB_EP_IN(EPnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)) {
		return EP_OK;
	}
	else return EP_FAILED;
}

/*************************** End of inline functions *************************/


/* User function for sending data */
// TODO NON BLOCKING Version
uint32_t USB_CDC_Write(uint8_t *txBuff, uint16_t len){

	uint32_t start_tick = GetSysTick(); // Custom timer

	if (len == 0) return EP_OK;

	/* - "get_circBufferTx_freeSize() < len", if the Main Loop is faster than the USB, the while loop slows it down, preventing accidental overwriting of the buffer
	   - "EndPoint[1].statusTx != EP_READY", when you use an if, you might miss the "opening" of a frame and have to wait for the next loop. With the while,
	   you are hitting the hardware the microsecond it becomes available. Most importantly, without this instruction, USB_CDC_send_data() simply won't work
	   in a loop, or only with slowing down the loop, because EndPoint[1].statusTx will most often remain on EP_BUSY (latency synchronization issue).
	*/
	while (EndPoint[1].statusTx != EP_READY || get_circBufferTx_freeSize() < len) {
		// blocking wait with small 6ms timeout for less impact on global flow (interrupts are not disabled for this function so we can use SystTick)
		if (GetSysTick() - start_tick > 5) {
			return EP_FAILED;
		}
	}

	uint16_t sentBytes = 0;

	while (sentBytes < len) {

		if (GetSysTick() - start_tick > 30) { // 30ms timeout
			USB_CDC_ForceResetState();
			return EP_FAILED;
		}
		uint16_t freeSize = get_circBufferTx_freeSize(); // if empty size = CIRC_BUFFER_TX_SIZE

		if (freeSize > 1) {
			// Note: if(freeSize > 0): if freeSize=1, chunk becomes 0 -> sentBytes += chunk does nothing, and the while() will spin forever until the timeout hits
			// Calculate how much we can actually fit right now
			uint16_t chunk = (len - sentBytes);
			if (chunk > freeSize) chunk = freeSize;
			write_to_circBufferTx(&txBuff[sentBytes], chunk);
			sentBytes += chunk;

			/* Since you are using the circular buffer to bridge the Main Loop (Producer) and the ISR (Consumer), you need to ensure they don't
			step on each other's toes. Whenever you call the scheduler from the "outside" (the Main Loop), wrap it in a brief interrupt disable.
			By checking both the hardware bit (EPENA) and your software flag (statusTx) inside the disabled-interrupt zone, you create a "Hard Lock."
			It is physically impossible for the packets to mix because the Main Loop will see that the hardware is busy and simply walk away,
			knowing the ISR will pick up the remaining data in the circular buffer as soon as the current packet finishes.
			*/
			__disable_irq();
			if (!(USB_EP_IN(1)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) && (EndPoint[1].statusTx == EP_READY)) {
				USB_CDC_transmit_scheduler(); // will read circular buffer and prepare transfer
			}
			__enable_irq();
		}
	}
	return EP_OK;
}

/* it is recommended to store RX data in a buffer and process the data in a main loop or separated task */
__WEAK uint32_t USB_CDC_UserRxCallBack_EP1(uint16_t length){

	return length;
}



/*********************************************************************************/
/**************************  Application-facing USB CDC functions  ***************/
/*********************************************************************************/

uint16_t USB_CDC_Read(uint8_t *dest, uint16_t maxLen)
{
    uint16_t total = 0;

    while (total < maxLen)
    {
        circBufferAddress data;

        /* peek_circBufferRx() returns only a contiguous block and does not advance readPtrRxCbuf */
        data = peek_circBufferRx(maxLen - total);

        if (data.len == 0)
            break;

        memcpy(dest + total,
               &circBufferRx[data.index],
               data.len);

        /* Release the RX data only after memcpy() has finished. This is what prevents a newly armed
         * USB DMA transfer from overwriting the block while the application is copying it. */
        commit_circBufferRx(data.len);

        total += data.len;
    }

    return total;
}

/*********************************************************************************/
/**************************** OTG FS ISR *****************************************/
/*********************************************************************************/

extern void OTG_HS_IRQHandler(void);

void OTG_HS_IRQHandler(){

/*	Typically handle the ISR these in order:
	-ENUMDNE (Enumeration Done): The hardware tells you the speed is set.
	-RXFLVL (RX FIFO Non-Empty): There is a packet from the PC. You read it to see if it's a SETUP, OUT, or DATA packet.
	-IEPINT (In Endpoint Interrupt): A previous "Send" (In) operation finished successfully; you can now send more data. */

	// Identify only the interrupts that are both PENDING and ENABLED
	uint32_t active_irq = USB_OTG_HS->GINTSTS & USB_OTG_HS->GINTMSK; /* MODIFIED FOR HS */

	/* SUSPEND DETECTED - to use uncomment in USB_OTG_HS_init_registers()
		 If USB bus has gone quiet (which happens about 3ms after you unplug the cable or the PC goes to sleep) */
	if (active_irq & USB_OTG_GINTSTS_USBSUSP){
		USB_OTG_HS->GINTSTS = USB_OTG_GINTSTS_USBSUSP; // clear flag /* MODIFIED FOR HS */
		USB_CDC_ForceResetState(); // Perform the robust cleanup
	}

	/* Wakeup DETECTED - to use uncomment in USB_OTG_HS_init_registers() */
	if (active_irq & USB_OTG_GINTSTS_WKUINT){
		USB_OTG_HS->GINTSTS = USB_OTG_GINTSTS_WKUINT; // clear flag /* MODIFIED FOR HS */
		USB_CDC_ForceResetState();
	}

	/**************************************************************/
	/****************** USBRST Reset event ************************/
	/**************************************************************/
	// The core sets this bit to indicate that a reset is detected on the USB.
	if(active_irq & USB_OTG_GINTSTS_USBRST){
		USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_USBRST);
		enumerate_Reset();
		return;
	}

	/**************************************************************/
	/****************** ENUMDNEM event ****************************/
	/**************************************************************/
	/* The core sets this bit to indicate that speed enumeration is complete: the hardware-level connection and speed negotiation (reset/chirp) are complete,
		   allowing software to begin handling USB control transfers (such as address assignment and descriptor setup).
		   The application must read the OTG_DSTS register to obtain the enumerated speed.
	 */
	if(active_irq & USB_OTG_GINTSTS_ENUMDNE){
		USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_ENUMDNE);
		set_device_status(DEVICE_STATE_DEFAULT);
	}

	/**************************************************************/
	/*************** IN endpoint event ****************************/
	/**************************************************************/

	/* IN endpoint event */
	if(USB_OTG_HS->GINTSTS & USB_OTG_GINTSTS_IEPINT){ /* MODIFIED FOR HS */

		uint32_t epnums = USB_OTG_DEVICE->DAINT;

		// --- Check EP0 IN ---
		if (epnums & (1U << USB_CDC_CONTROL_EP)) {
			uint32_t IN_interrupt = USB_EP_IN(0)->DIEPINT;

			if (IN_interrupt & USB_OTG_DIEPINT_XFRC) {
				// CRITICAL: Handshake for Control Status Phase -  Prepare for the next Setup Packet
				USB_EP_OUT(0)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);

				EndPoint[0].statusTx = EP_READY;
				clear_USB_device_status(DEVICE_STATE_TX_PR);
				USB_EP_IN(0)->DIEPINT = USB_OTG_DIEPINT_XFRC;
			}
			CLEAR_IN_EP_INTERRUPT(0, IN_interrupt);
		}

		// --- Check EP1 IN ---
		if (epnums & (1U << USB_CDC_DATA_IN_EP)) {
			uint32_t IN_interrupt = USB_EP_IN(1)->DIEPINT;

			if (IN_interrupt & USB_OTG_DIEPINT_XFRC) {
				CLEAR_IN_EP_INTERRUPT(1, USB_OTG_DIEPINT_XFRC);

				// 1. Handle ZLP Logic First
				if (EndPoint[1].statusTx == EP_BUSY) {
					if (EndPoint[1].totXferLen > 0 && (EndPoint[1].totXferLen % MAX_CDC_EP1_TX_SIZ == 0)) { /* MODIFIED FOR HS: 512 bytes instead of 64 */
						send_zlp(1);
						return; // Exit ISR, wait for ZLP's XFRC
					}
				}

				// 2. The transfer is officially DONE
				/*
				 * Commit the circular-buffer data only now, after USB DMA
				 * has reported XFRC. For a packet-size-multiple transfer,
				 * this also happens after the terminating ZLP.
				 */
				commit_circBufferTx(ep1TxPendingLen);
				ep1TxPendingLen = 0;

				EndPoint[1].txCounter = 0;
				EndPoint[1].statusTx = EP_READY;
				clear_USB_device_status(DEVICE_STATE_TX_PR);

				// 3. Re-prime the OUT endpoint only if the RX circular buffer
				// has room for a complete HS packet. If not, leave it NAKed;
				// the application will re-arm it when it consumes RX data.
				USB_CDC_prepare_EP1_OUT_DMA();

				// 4. Trigger next transfer IF there is data in the buffer
				if (is_circBufferTx_empty() == 0) {
					USB_CDC_transmit_scheduler();
			        } else {
			            // Only SNAK if the buffer is truly empty and we aren't about to send more
			            USB_EP_IN(1)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
			        }
		    }

		    // Safety: Clear any other pending bits for this EP
		    USB_EP_IN(1)->DIEPINT = IN_interrupt;
		    }

		// --- Check EP2 IN (CDC notification endpoint) ---
		if (epnums & (1U << USB_CDC_NOTIFICATION_EP)) {
			uint32_t IN_interrupt = USB_EP_IN(USB_CDC_NOTIFICATION_EP)->DIEPINT;
			if (IN_interrupt & USB_OTG_DIEPINT_XFRC) {
				/* No Serial-State notification producer is implemented yet.
				 * EP2 remains NAKed after any completed notification transfer. */
				USB_EP_IN(USB_CDC_NOTIFICATION_EP)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
			}
			USB_EP_IN(USB_CDC_NOTIFICATION_EP)->DIEPINT = IN_interrupt;
		}
	}
	// DO NOT call USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_IEPINT) here! It is a read-only logical OR of the individual endpoint interrupts.
	// If you manually write to GINTSTS to clear IEPINT, you might accidentally clear EP1's "Global" flag before you've had a chance to see EP1's flag.
	//USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_IEPINT);

	/***************************************************************/
	/*************** OUT endpoint event - OEPINT ********************
	 ***************************************************************		*/

	if(USB_OTG_HS->GINTSTS & USB_OTG_GINTSTS_OEPINT){ /* OUT endpoint event */

		USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_OEPINT);
		uint32_t epnums  = USB_OTG_DEVICE->DAINT;    /* Read out EndPoint INTerrupt bits */

		/************************	 EP 0  	OUT	  *******************************/
		if( epnums & 0x00010000){ /* EndPoint INTerrupt bits correspond to EP0 OUT */
			uint32_t epint = USB_EP_OUT(0)->DOEPINT; /* Read out Endpoint Interrupt register for EP0 */

			if(epint & USB_OTG_DOEPINT_STUP){		/*  SETUP phase done, Setup packet received */
				/* The SETUP packet was received by EP0 DMA into rxBufferEp0. */
				memcpy(setup_pkt_data.raw_data, rxBufferEp0, sizeof(setup_pkt_data.raw_data));
				enumerate_Setup();
			}
			if(epint & USB_OTG_DOEPINT_XFRC){

				/* EP0 OUT DMA completed. Capture the actual data-stage length. */
				uint16_t remaining = (uint16_t)(USB_EP_OUT(0)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ);
				uint16_t len = (uint16_t)(RX_BUFFER_EP0_SIZE - remaining);
				EndPoint[0].rxCounter = len;

				/* SET_LINE_CODING has an OUT data stage. Process it only after DMA XFRC. */
				if (setup_pkt_data.setup_pkt.bRequest == CDC_SET_LINE_CODING && len > 0)
					EndPoint[0].rxCallBack(CDC_SET_LINE_CODING);

				/* Re-arm EP0 OUT for DMA reception, preserving the Setup Count so the hardware can continue to intercept incoming USB setup commands. */
				uint32_t ep0_pktcnt = (RX_BUFFER_EP0_SIZE + MAX_CDC_EP0_TX_SIZ - 1) / MAX_CDC_EP0_TX_SIZ;
				USB_EP_OUT(0)->DOEPTSIZ = (ep0_pktcnt << 19)
																| RX_BUFFER_EP0_SIZE
																| USB_OTG_DOEPTSIZ_STUPCNT;

				// Point DMA engine back to the start of the control buffer destination
				USB_EP_OUT(0)->DOEPDMA = (uint32_t)(rxBufferEp0);

				// CNAK and EPENA must be set again to activate hardware reception via DMA
				USB_EP_OUT(0)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
			}
			CLEAR_OUT_EP_INTERRUPT(0, epint);
		}

		/* Originally used the TXEF interrupt (that triggers whenever the FIFO is empty) and used USB_OTG_DIEPINT_XFRC interrupt only
		  for the last packet but it was required a blocking while(); to wait EPENA to deactivate from the previous transfer -> switched to all XFRC interrupts */

		/************************	 EP 1  	OUT	  *******************************/
		if( epnums & 0x00020000){ /* EndPoint INTerrupt bits correspond to EP1 OUT */

			// OTG_DOEPINTx : this register indicates the status of an endpoint with respect to USB- and AHB-related events.
			uint32_t epint = USB_EP_OUT(1)->DOEPINT; /* Read out Endpoint Interrupt register for EP1 */

			// XFRC: Transfer completed interrupt. Indicates that the programmed transfer is complete on the AHB as well as on the USB, for this endpoint.
			if(epint & USB_OTG_DOEPINT_XFRC){

				uint16_t remaining =
					USB_EP_OUT(1)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ;

				uint16_t len = RX_BUFFER_EP1_SIZE - remaining;

				/*
				 * USB DMA wrote to the staging buffer. Now that XFRC has
				 * confirmed completion, commit the received bytes to the
				 * circular buffer and advance its write pointer.
				 */
				if (len > 0) {
					memcpy(&circBufferRx[ep1RxPendingIndex],
					       ep1RxDmaBuffer,
					       len);

					writePtrRxCbuf = (uint16_t)(ep1RxPendingIndex + len);
					if (writePtrRxCbuf >= CIRC_BUFFER_RX_SIZE)
						writePtrRxCbuf = 0;

					ep1RxPendingIndex = writePtrRxCbuf;

					EndPoint[1].rxCallBack(len); // EndPoint[i].rxCallBack = &USB_CDC_transferRXCallback_EP1
				}

				// Re-arm EP1 OUT for the next DMA reception.
				USB_CDC_prepare_EP1_OUT_DMA();
			}
			CLEAR_OUT_EP_INTERRUPT(1, epint);
		}
		return;
	}
}
