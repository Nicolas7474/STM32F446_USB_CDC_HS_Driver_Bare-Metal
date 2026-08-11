/****************************************************************
* STM32F446
* USB OTG HS device (CDC) implementation
*
* Nicolas Prata 2026
*
****************************************************************/
#pragma once

#include <string.h>
#include "stm32f446xx.h"

/******************************************************************************
* This section contains some macros and defines for better compatibility
* with other libs. If you find any conflicting types, delete them from here
*******************************************************************************/

#define USB_CLEAR_INTERRUPT(IRQ)    (USB_OTG_HS->GINTSTS = (IRQ))
#define USB_MASK_INTERRUPT(IRQ)     (USB_OTG_HS->GINTMSK &= ~(IRQ))
#define USB_UNMASK_INTERRUPT(IRQ)   (USB_OTG_HS->GINTMSK |= (IRQ))

#define CLEAR_IN_EP_INTERRUPT(NUM, IRQ)          (USB_EP_IN(NUM)->DIEPINT = (IRQ))
#define CLEAR_OUT_EP_INTERRUPT(NUM, IRQ)         (USB_EP_OUT(NUM)->DOEPINT = (IRQ))

#define USB_OTG_DEVICE      		((USB_OTG_DeviceTypeDef *) (USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE))

#define USB_EP_OUT(i) 			((USB_OTG_OUTEndpointTypeDef *) ((USB_OTG_HS_PERIPH_BASE +  USB_OTG_OUT_ENDPOINT_BASE) + ((i) * USB_OTG_EP_REG_SIZE)))
#define USB_EP_IN(i)    		((USB_OTG_INEndpointTypeDef *)	((USB_OTG_HS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE) + ((i) * USB_OTG_EP_REG_SIZE)))

#define USB_OTG_DFIFO(i)    *(__IO uint32_t *)((uint32_t)USB_OTG_HS_PERIPH_BASE + USB_OTG_FIFO_BASE + (i) * USB_OTG_FIFO_SIZE)

#define USB_OTG_PCGCCTL      ((USB_OTG_PCGCCTLTypeDef *)( USB_OTG_HS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE)) // Power and Clock Gating Control Register

typedef struct{
	__IO uint32_t PCGCCTL;
}
USB_OTG_PCGCCTLTypeDef;



/***************************************************
 * User settings
***************************************************/
/* On the STM32F446, the dedicated OTG_HS Packet FIFO RAM is 4 KB (4096 bytes / 1024 words).
This is shared between:
    Rx FIFO (All OUT endpoints)
    Tx FIFOs (Each IN endpoint)
*/

#define FLUSH_FIFO_TIMEOUT		2000

/* In HS mode, the maximum packet size (MPS) for Bulk EP is fixed at 512 bytes. */


#define MAX_CDC_EP0_TX_SIZ  64U   /* EP0 maximum packet size. */
#define MAX_CDC_EP1_TX_SIZ  	512   /* Max TX transaction size for EP1.  Max USB_OTG_DIEPTSIZ_XFRSIZ value for HS Bulk. */


										// (expect 1 discrete packet of up to 512 bytes for this transaction before triggering a transfer complete interrupt)

#define USB_CDC_CIRC_BUFFER_SIZE 2048 // for std baudrate CLI, up to 4096 bytes buffer gives the CPU plenty of time to wake up and process the data before it overflows

/***************************************************
 * EP statuses
***************************************************/
#define EP_READY 				0U
#define EP_BUSY  				1U
#define EP_ZLP   				2U
#define EP_WAIT  				3U

/***************************************************
 * EP functions return values
***************************************************/

#define EP_OK				1U
#define EP_FAILED			0U

/***************************************************
 * Device states
 ***************************************************/
typedef enum {
	DEVICE_STATE_DEFAULT =			0,
	DEVICE_STATE_RESET =			1,
	DEVICE_STATE_ADDRESSED =		2,
	DEVICE_STATE_LINECODED =		4,
	DEVICE_STATE_CONFIGURED =		8,
	DEVICE_STATE_TX_PR =			16, /* TX in PRogress */
	DEVICE_STATE_TX_FIFO1_ERROR =	32,
} eDeviceState;
/***************************************************
 * SETUP stage request templates
***************************************************/

#define REQ_TYPE_HOST_TO_DEVICE_GET_DEVICE_DECRIPTOR	0x0680
#define REQ_TYPE_DEVICE_TO_HOST_SET_ADDRESS				0x0500
#define REQ_TYPE_DEVICE_TO_HOST_SET_CONFIGURATION		0x0900

#define DESCRIPTOR_TYPE_DEVICE							0x0100
#define DESCRIPTOR_TYPE_CONFIGURATION					0x0200
#define DESCRIPTOR_TYPE_LANG_STRING						0x0300
#define DESCRIPTOR_TYPE_MFC_STRING						0x0301
#define DESCRIPTOR_TYPE_PROD_STRING						0x0302
#define DESCRIPTOR_TYPE_SERIAL_STRING					0x0303
#define DESCRIPTOR_TYPE_CONFIGURATION_STRING			0x0304
#define DESCRIPTOR_TYPE_INTERFACE_STRING				0x0305
#define DESCRIPTOR_TYPE_DEVICE_QUALIFIER				0x0600
#define DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION		0x0700

#define CDC_GET_LINE_CODING								0x21A1
#define CDC_SET_LINE_CODING								0x2021
#define CDC_SET_CONTROL_LINE_STATE						0x2221
#define CLEAR_FEATURE_ENDP								0x0102

/***************************************************
* Endpoint structure
***************************************************/

typedef struct EndPointStruct{
	volatile uint16_t statusRx; // Since it can be modified in the ISR, forces the CPU to read the actual memory location
	volatile uint16_t statusTx; // every single time instead of using a cached value in a register

	uint16_t rxCounter;
	uint16_t txCounter;

	uint8_t *rxBuffer_ptr;
	uint8_t *txBuffer_ptr;
	uint32_t (*txCallBack)(uint8_t EPnum);
	uint32_t (*rxCallBack)(uint32_t param);
	uint32_t (*setTxBuffer)(uint8_t EPnum, uint8_t *txBuff, uint16_t len);

	volatile uint16_t totXferLen;

} EndPointStruct;

extern EndPointStruct EndPoint[];

/****************************************************
* Setup packet structure
* is used in union to access data both
* as structure and as raw data*
***************************************************/

typedef struct __attribute__((packed)){
    uint8_t   bmRequestType;
    uint8_t   bRequest;
    uint16_t  wValue;
    uint16_t  wIndex;
    uint16_t  wLength;
} USB_setup_req;	/* SETUP packet buffer.
Always 8 bytes */


typedef union{
	USB_setup_req setup_pkt;
	uint32_t raw_data[2];
} USB_setup_req_data;


/*************************************************** *
 * Functions' declaration *
***************************************************/

/* init functions */
void SystemClock_Config(void);
uint32_t USB_OTG_HS_GPIO_Init(void);
uint32_t USB_OTG_HS_Core_Init(void);
void USB_OTG_HS_FIFO_and_Interrupts_Init(void);
void USB_OTG_HS_Connect(void);
void OTG_HS_IRQHandler(void);

void enumerate_Reset(void);
void enumerate_Setup(void);

/* Endpoint functions */
uint32_t USB_CDC_setTxBuffer(uint8_t EPnum, uint8_t *txBuff, uint16_t len);
uint32_t USB_CDC_transferTXCallback(uint8_t EPnum);
uint32_t USB_CDC_transferRXCallback_EP0(uint32_t param);
uint32_t USB_CDC_transferRXCallback_EP1(uint32_t param);

/* misc */
uint32_t USB_CDC_transmit_scheduler(void);
/* this function monitors if any data is pending in circ buffer or whatever */
uint32_t USB_FlushTxFifo(uint32_t EPnum, uint32_t timeout);
uint32_t USB_FlushRxFifo(uint32_t timeout);

uint32_t check_USB_device_status(eDeviceState state);
void clear_USB_device_status(eDeviceState state);

/* User code functions */
uint32_t USB_CDC_Write(uint8_t *txBuff, uint16_t len);
uint16_t USB_CDC_Read(uint8_t *dest, uint16_t maxLen);
uint32_t USB_CDC_UserRxCallBack_EP1(uint16_t length); // __WEAK function called upon interrupt XFRC: Transfer completed
uint32_t GetSysTick(void);


void send_zlp(uint8_t EPnum);
void USB_CDC_ForceResetState(void);
extern volatile uint32_t msTicks;
/****************************************************
* Circular buffer
****************************************************/

#define CIRC_BUFFER_TX_SIZE USB_CDC_CIRC_BUFFER_SIZE // 2048
#define CIRC_BUFFER_RX_SIZE USB_CDC_CIRC_BUFFER_SIZE

void write_to_circBufferTx(uint8_t *buf, uint16_t len);

typedef struct {
	uint16_t index;
	uint16_t len;
} circBufferAddress;

extern uint8_t circBufferRx[CIRC_BUFFER_RX_SIZE];


/******************************************************************************
* USB CDC device descriptors
* borrowed from STMicroelectronics for educational purposes*
*******************************************************************************/

#define LOBYTE(x)   ((uint8_t)((x) & 0xFFU))
#define HIBYTE(x)   ((uint8_t)(((x) >> 8) & 0xFFU))


#define USB_CDC_MAX_PACKET_SIZE		512 /* 512 bytes for High-Speed Bulk */
#define USB_CDC_CONTROL_EP          0U
#define USB_CDC_DATA_IN_EP          1U
#define USB_CDC_NOTIFICATION_EP		2U
#define USB_CDC_NOTIFICATION_MPS	8U
#define USB_CDC_NOTIFICATION_INTERVAL	0x10U /* 16 microframes at HS; host polling interval */

#define EP0_SIZE			64
#define EP_COUNT			3


#define USBD_VID				    	1155
#define USBD_LANGID_STRING				1033
#define USBD_PID_HS				   		22336

#define DEVICE_DESCRIPTOR_LENGTH		18
#define CONFIGURATION_DESCRIPTOR_LENGTH 67 /* 60-byte CDC data config + 7-byte EP2 notification descriptor */

#define LANG_DESCRIPTOR_LENGTH			4
#define MFC_DESCRIPTOR_LENGTH			38
#define PRODUCT_DESCRIPTOR_LENGTH		44
#define SERIAL_DESCRIPTOR_LENGTH		50 /* Maximum serial descriptor: 24 ASCII characters as UTF-16LE */
#define SERIAL_NUMBER_MAX_CHARS		24
#define DEVICE_QUALIFIER_LENGTH			10
#define INTERFACE_STRING_LENGTH			28
#define CONFIG_STRING_LENGTH			22

#define CDC_LINE_CODING_LENGTH			7

/* Descriptor storage lives in usb_cdc_hs.c so every translation unit shares one copy. */
extern const uint8_t deviceDescriptor[DEVICE_DESCRIPTOR_LENGTH];
extern const uint8_t configurationDescriptor[CONFIGURATION_DESCRIPTOR_LENGTH];
extern const uint8_t deviceQualifierDescriptor[DEVICE_QUALIFIER_LENGTH];
extern const uint8_t otherSpeedConfigurationDescriptor[CONFIGURATION_DESCRIPTOR_LENGTH];
extern const uint8_t languageStringDescriptor[LANG_DESCRIPTOR_LENGTH];
extern const uint8_t manufactorStringDescriptor[MFC_DESCRIPTOR_LENGTH];
extern const uint8_t productStringDescriptor[PRODUCT_DESCRIPTOR_LENGTH];

/* Mutable serial-number descriptor. By default it is populated from the STM32 UID. */
extern uint8_t serialNumberStringDescriptor[SERIAL_DESCRIPTOR_LENGTH];

extern const uint8_t stringInterface[INTERFACE_STRING_LENGTH];
extern const uint8_t configurationStringDescriptor[CONFIG_STRING_LENGTH];

/*
 * Set a user-defined USB serial number. The ASCII string is converted to the
 * USB UTF-16LE string-descriptor format. Maximum length is 24 characters.
 * If this function is never called, the driver uses the STM32 96-bit UID.
 */
uint32_t USB_CDC_SetSerialNumber(const char *serial);

