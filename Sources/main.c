/****************************************************************
*************   Bare-Metal USB HS CDC - Working example    ******
****************************************************************/

#include "../Inc/usb_cdc_hs.h"
#include "../Inc/myConfig.h"
#include "timers.h"

#include <string.h>

volatile uint8_t flag = 0; // 0 = no data, 1 = data arrived

/* EP1 HS Bulk packets are limited to 512 bytes, so the echo buffer
 * only needs to hold one contiguous block returned by peek_circBufferRx(). */
static uint8_t dest[MAX_CDC_EP1_TX_SIZ] __attribute__((aligned(4)));

uint32_t SystemCoreClock = 180000000; // system_stm32f4xx.c is not included in this project so we define SystemCoreClock manually
volatile uint32_t msTicks = 0; // Volatile ensures the compiler doesn't optimize out reads of this value

int main(void)
{
	/* 0. Configure SysTick for 1ms interrupts using CMSIS SystemCoreClock */
	if (SysTick_Config(SystemCoreClock / 1000)) {
		NVIC_SystemReset(); // reset if return value indicate a failure (1)
	}

	/* 1. Configure USB power / clock-dependent settings. */
	SystemClock_Config();

	/* 2. Configure USB3300 PHY pins, release reset, and turn on AHB1 clocks. */
	USB_OTG_HS_GPIO_Init();

	/* 3. Initialize USB OTG HS core registers, ULPI mode, and soft reset.
	 * DMA mode is enabled here. */
	USB_OTG_HS_Core_Init();

	/* 4. Configure the OTG HS FIFO RAM and USB interrupts.	 *
     * This must be done before connecting the device to the host.
     * The FIFO RAM is still used internally by the USB OTG HS core;
     * the CPU no longer services the FIFOs for CDC data transfers.
     */
    USB_OTG_HS_FIFO_and_Interrupts_Init();

    /* 5. Connect the USB device.
     * The host will now detect the device and issue the USB reset/
     * enumeration sequence. Endpoint/DMA state is initialized from
     * the USB reset handler.
     */
    USB_OTG_HS_Connect();

    heartBeatLed();

    /* Give the host time to detect and enumerate the device.
     * This delay is only useful for the example/debug application;
     * it is not required by the USB CDC driver itself.
     */
    NBdelay_ms(4000);

    while (1)
    {
        /* USB_CDC_UserRxCallBack_EP1() only raises a flag from the ISR.
         * The actual RX data is processed here, outside interrupt context.
         */
        if (flag)
        {
            flag = 0;

            while (1)
            {
            	// Loop to consume all the requested data
            	 uint16_t len = USB_CDC_Read(dest, sizeof(dest));

            	 if (len == 0)
            		 break;

                /* Echo the received data back to the host.
                 * USB_CDC_Write() already contains its own
                 * timeout/retry protection. Give it one additional
                 * attempt here, as in the original example.
                 */
                if (USB_CDC_Write(dest, len) != EP_OK)
                {
                    // retry with timeout
                }
            }
        }
    }
}


/* This callback is called from the EP1 OUT USB ISR after the received
 * packet has been committed to circBufferRx. Keep it short.
 */
uint32_t USB_CDC_UserRxCallBack_EP1(uint16_t length)
{
    (void)length;
    flag = 1;
    return EP_OK;
}


void RTC_WKUP_IRQHandler(void)
{
    if (RTC->ISR & RTC_ISR_WUTF)
    {
        /* Clear RTC Wakeup Timer flag */
        RTC->ISR &= ~RTC_ISR_WUTF;

        /* Clear EXTI22 pending flag */
        EXTI->PR = EXTI_PR_PR22;

        /*PE3 is active LOW:  */
        GPIOE->ODR ^= GPIO_ODR_OD3;
    }
}


// The SysTick handler is predefined in the vector table
void SysTick_Handler(void) {
	msTicks++;
}
