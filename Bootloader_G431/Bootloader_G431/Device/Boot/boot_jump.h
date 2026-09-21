#ifndef BOOT_JUMP_H
#define BOOT_JUMP_H

#include <stdint.h>

uint8_t BootMailbox_TakeRequest(uint32_t *session_id, uint32_t *reason);
void BootJump_ToApplication(void);

#endif /* BOOT_JUMP_H */
