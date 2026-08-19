#include "stm32f4xx.h"
#include <myConfig.h>


void activateFPU(void) {

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 20U)|(3UL << 22U));  /* set CP10 and CP11 Full Access */

	// Enable Lazy Stacking for better ISR performance : When an interrupt (like your TIM7 or TIM8 ISRs) occurs, the processor normally has to save all the FPU registers
	// to the stack. This is slow. Lazy Stacking tells the hardware only to save FPU registers if the ISR actually performs a floating-point operation.
	FPU->FPCCR |= FPU_FPCCR_LSPEN_Msk;
#endif
	// Enabling the hardware bits isn't enough; you must also tell your compiler (GCC, Clang, or Keil) to actually generate FPU instructions instead of using slow software libraries.
	// If you are using GCC (arm-none-eabi-gcc), add these flags to your build command:
	// -mfloat-abi=hard: Uses the hardware FPU for calculations and passing arguments.
	// -mfpu=fpv4-sp-d16: Specifies the specific FPU version on the STM32F4.
}


void heartBeatLed(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;          // Enable GPIOE clock

    GPIOE->MODER &= ~GPIO_MODER_MODER3;           // PE3 = output
    GPIOE->MODER |= GPIO_MODER_MODER3_0;
    GPIOE->OTYPER &= ~GPIO_OTYPER_OT3;            // Push-pull
    GPIOE->PUPDR &= ~GPIO_PUPDR_PUPDR3;           // No pull-up/down
    GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;     // High speed
    GPIOE->BSRR = GPIO_BSRR_BS3;                  // LED OFF (active LOW)

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;            // Enable PWR clock
    PWR->CR |= PWR_CR_DBP;                        // Disable backup-domain write protection
    while (!(PWR->CR & PWR_CR_DBP));

    RCC->CSR |= RCC_CSR_LSION;                    // Enable internal LSI clock
    while (!(RCC->CSR & RCC_CSR_LSIRDY));         // Wait for LSI ready

    RCC->BDCR &= ~RCC_BDCR_RTCSEL;                // Clear RTC clock selection
    RCC->BDCR |= RCC_BDCR_RTCSEL_1;               // Select LSI as RTC clock (not an accurate 32 kHz reference)
    RCC->BDCR |= RCC_BDCR_RTCEN;                  // Enable RTC clock

    RTC->WPR = 0xCA;                              // Disable RTC write protection
    RTC->WPR = 0x53;

    RTC->CR &= ~RTC_CR_WUTE;                      // Disable wakeup timer
    while (!(RTC->ISR & RTC_ISR_WUTWF));          // Wait until WUT registers are writable

    RTC->PRER = (127U << RTC_PRER_PREDIV_A_Pos)   // Async prescaler = 128
              | (127U << RTC_PRER_PREDIV_S_Pos);  // Sync prescaler  = 128

    RTC->WUTR = 1023;    			 			  // Wakeup counter,  set reload counter for 500 ms interval
    RTC->CR &= ~RTC_CR_WUCKSEL;   			      // Select RTCCLK / 16 (WUCKSEL = 000)
    RTC->ISR &= ~RTC_ISR_WUTF;                    // Clear RTC wakeup flag

    EXTI->IMR |= EXTI_IMR_IM22;                   // Enable EXTI22 (RTC wakeup)
    EXTI->RTSR |= EXTI_RTSR_TR22;                 // Rising-edge trigger
    EXTI->PR = EXTI_PR_PR22;                      // Clear pending EXTI22 interrupt

    RTC->CR |= RTC_CR_WUTIE;                      // Enable wakeup interrupt
    RTC->CR |= RTC_CR_WUTE;                       // Enable wakeup timer

    RTC->WPR = 0xFF;                              // Re-enable RTC write protection

    NVIC_SetPriority(RTC_WKUP_IRQn, 15);          // Lowest priority
    NVIC_EnableIRQ(RTC_WKUP_IRQn);                // Enable RTC wakeup IRQ
}
