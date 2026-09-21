#include "boot_jump.h"

#include "main.h"
#include "usart.h"

#include "boot_crc.h"
#include "boot_image_verify.h"
#include "gripper_ota_layout.h"

#define BOOT_MAILBOX_MAGIC              (0x544F4F42UL) /* bytes: BOOT */

typedef struct
{
    uint32_t magic;
    uint32_t session_id;
    uint32_t reason;
    uint32_t magic_inv;
    uint32_t crc32;
    uint32_t reserved[3];
} BootMailbox_t;

typedef char BootMailbox_SizeMustBe32Bytes[
    (sizeof(BootMailbox_t) == GRIPPER_BOOT_MAILBOX_SIZE) ? 1 : -1];

uint8_t BootMailbox_TakeRequest(uint32_t *session_id, uint32_t *reason)
{
    volatile BootMailbox_t *mailbox =
        (volatile BootMailbox_t *)GRIPPER_BOOT_MAILBOX_BASE;
    BootMailbox_t copy;
    uint32_t expected_crc;
    uint32_t index;
    uint8_t valid;

    copy.magic = mailbox->magic;
    copy.session_id = mailbox->session_id;
    copy.reason = mailbox->reason;
    copy.magic_inv = mailbox->magic_inv;
    copy.crc32 = mailbox->crc32;
    copy.reserved[0] = mailbox->reserved[0];
    copy.reserved[1] = mailbox->reserved[1];
    copy.reserved[2] = mailbox->reserved[2];

    expected_crc = BootCrc32_Calculate((const uint8_t *)&copy, 16UL);
    valid = ((copy.magic == BOOT_MAILBOX_MAGIC) &&
             (copy.magic_inv == ~BOOT_MAILBOX_MAGIC) &&
             (copy.crc32 == expected_crc))
                ? 1U
                : 0U;

    /* A mailbox is a one-shot request, and cold SRAM is never trusted. */
    for (index = 0UL; index < (GRIPPER_BOOT_MAILBOX_SIZE / 4UL); ++index)
    {
        ((volatile uint32_t *)mailbox)[index] = 0UL;
    }
    __DSB();

    if (valid != 0U)
    {
        if (session_id != NULL)
        {
            *session_id = copy.session_id;
        }
        if (reason != NULL)
        {
            *reason = copy.reason;
        }
    }

    return valid;
}

void BootJump_ToApplication(void)
{
    uint32_t initial_msp;
    uint32_t reset_handler;
    uint32_t index;
    void (*application_reset)(void);

    if (BootImage_VectorIsValid(GRIPPER_APP_SIZE) == 0U)
    {
        return;
    }

    initial_msp = *(const uint32_t *)GRIPPER_APP_BASE;
    reset_handler = *(const uint32_t *)(GRIPPER_APP_BASE + 4UL);
    application_reset = (void (*)(void))reset_handler;

    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET)
    {
        /* The normal path reaches this loop only after a completed response. */
    }
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
    (void)HAL_UART_DeInit(&huart2);

    /*
     * Bootloader runs from the HSI PLL while the Application configures an
     * HSE PLL.  HAL_RCC_OscConfig() refuses to change a PLL that is still
     * driving SYSCLK, so hand the Application a reset-like clock tree.
     * Keep SysTick alive until HAL_RCC_DeInit() has finished because the HAL
     * clock routines use the HAL tick for their timeout handling.
     */
    if (HAL_RCC_DeInit() != HAL_OK)
    {
        NVIC_SystemReset();
        while (1)
        {
            /* The reset request must not return. */
        }
    }

    __disable_irq();
    SysTick->CTRL = 0UL;
    SysTick->LOAD = 0UL;
    SysTick->VAL = 0UL;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

    for (index = 0UL; index < 8UL; ++index)
    {
        NVIC->ICER[index] = 0xFFFFFFFFUL;
        NVIC->ICPR[index] = 0xFFFFFFFFUL;
    }

    SCB->VTOR = GRIPPER_APP_BASE;
    __set_CONTROL(0UL);
    __set_PSP(0UL);
    __set_BASEPRI(0UL);
    __set_FAULTMASK(0UL);
    __DSB();
    __ISB();
    __set_MSP(initial_msp);
    __DSB();
    __ISB();

    /* Do not carry Bootloader PRIMASK=1 into the Application. */
    __enable_irq();
    application_reset();

    while (1)
    {
        /* A valid Reset_Handler must never return. */
    }
}
