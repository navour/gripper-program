#ifndef GRIPPER_OTA_APP_H
#define GRIPPER_OTA_APP_H

#include <stdint.h>

#define GRIPPER_APP_ROLE                 (1U)
#define GRIPPER_APP_VERSION_MAJOR        (1U)
#define GRIPPER_APP_VERSION_MINOR        (15U)
#define GRIPPER_APP_VERSION_PATCH        (0U)
#define GRIPPER_APP_VERSION_U16          ((GRIPPER_APP_VERSION_MAJOR << 8U) | \
                                          GRIPPER_APP_VERSION_MINOR)

#define GRIPPER_BOOT_VERSION_U16         (0x0100U)

#define GRIPPER_APP_OTA_CAP_MAILBOX      (1U << 0)
#define GRIPPER_APP_OTA_CAP_BOOT_PROTO   (1U << 1)
#define GRIPPER_APP_OTA_CAP_SINGLE_SLOT  (1U << 2)
#define GRIPPER_APP_OTA_CAPS             (GRIPPER_APP_OTA_CAP_MAILBOX | \
                                          GRIPPER_APP_OTA_CAP_BOOT_PROTO | \
                                          GRIPPER_APP_OTA_CAP_SINGLE_SLOT)

#define GRIPPER_BOOT_REASON_MODBUS_OTA   (1UL)

uint32_t GripperOtaApp_CreateSessionId(uint32_t entropy);

/* This function does not return. Call it only after the Modbus ACK reached TC. */
void GripperOtaApp_ResetToBootloader(uint32_t session_id, uint32_t reason);

#endif /* GRIPPER_OTA_APP_H */
