#include "pos_persist.h"

#include "gripper_ota_layout.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include <string.h>

/* ====================== 配置：使用 Flash 最后一页 ====================== */
/* STM32G431CB：Flash 128KB，地址范围 0x0800_0000 ~ 0x0801_FFFF，页大小 2KB */
#define POS_PERSIST_FLASH_BASE   (GRIPPER_CALIB_BASE)
#define POS_PERSIST_FLASH_SIZE   (GRIPPER_CALIB_SIZE)

#define POS_PERSIST_MAGIC        (0x505A524Fu) /* 'PZRO' */
#define POS_PERSIST_MAGIC_BIAS   (0x505A4231u) /* 'PZB1' */

/* 一条记录 24 bytes = 3x doubleword，便于用 FLASH_TYPEPROGRAM_DOUBLEWORD 写入 */
typedef struct {
    uint32_t magic;
    uint32_t seq;
    int32_t  pos_mrad;     /* 机械坐标：mrad */
    int32_t  scale_milli;  /* s_pos_scale * 1000 */
    uint32_t crc32;        /* crc32 over first 16 bytes */
    uint32_t rsv;          /* 0xFFFFFFFF */
} pos_rec_t;

static uint32_t crc32_sw(const void *data, uint32_t len)
{
    /* 以太网标准 CRC32 (polynomial 0x04C11DB7 reflected 0xEDB88320) */
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t *p = (const uint8_t*)data;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (uint32_t b = 0; b < 8; b++) {
            uint32_t m = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & m);
        }
    }
    return ~crc;
}

static inline const pos_rec_t* rec_at(uint32_t idx)
{
    return (const pos_rec_t*)(POS_PERSIST_FLASH_BASE + idx * sizeof(pos_rec_t));
}

static inline uint32_t rec_capacity(void)
{
    return POS_PERSIST_FLASH_SIZE / sizeof(pos_rec_t);
}

static uint8_t rec_is_empty(const pos_rec_t *r)
{
    return (r->magic == 0xFFFFFFFFu);
}

static uint8_t rec_is_valid(const pos_rec_t *r)
{
    if (r->magic != POS_PERSIST_MAGIC && r->magic != POS_PERSIST_MAGIC_BIAS) return 0;
    /* 校验 CRC */
    uint32_t c = crc32_sw(r, 16);
    return (c == r->crc32);
}

/**
 * @brief  持久化模块初始化
 * @param  None
 * @retval None
 */
void PosPersist_Init(void)
{
    // 当前实现无需初始化硬件外设
}

/**
 * @brief  读取上一次保存的机械零点
 * @param  pos_rad_out: 输出零点角度（rad）
 * @param  scale_out: 输出保存时的比例系数
 * @retval 1=读取到有效记录，0=无有效记录
 */
uint8_t PosPersist_LoadEx(float *pos_rad_out, float *scale_out,
                          float *bias_rad_out, uint8_t *bias_valid_out)
{
    if (!pos_rad_out || !scale_out) return 0;

    uint32_t best_seq = 0;
    const pos_rec_t *best = NULL;

    uint32_t cap = rec_capacity();
    for (uint32_t i = 0; i < cap; i++) {
        const pos_rec_t *r = rec_at(i);
        if (rec_is_empty(r)) {
            /* 后面都是空的，提前结束 */
            break;
        }
        if (rec_is_valid(r)) {
            if (best == NULL || (uint32_t)(r->seq - best_seq) < 0x80000000u) {
                best = r;
                best_seq = r->seq;
            }
        }
    }

    if (!best) {
        *pos_rad_out = 0.0f;
        *scale_out   = 1.0f;
        if (bias_rad_out) *bias_rad_out = 0.0f;
        if (bias_valid_out) *bias_valid_out = 0u;
        return 0;
    }

    *pos_rad_out = ((float)best->pos_mrad) / 1000.0f;
    *scale_out   = ((float)best->scale_milli) / 1000.0f;
    if (bias_rad_out) {
        *bias_rad_out = ((float)((int32_t)best->rsv)) / 1000.0f;
    }
    if (bias_valid_out) {
        *bias_valid_out = (best->magic == POS_PERSIST_MAGIC_BIAS) ? 1u : 0u;
    }
    return 1;
}

uint8_t PosPersist_Load(float *pos_rad_out, float *scale_out)
{
    return PosPersist_LoadEx(pos_rad_out, scale_out, NULL, NULL);
}

static HAL_StatusTypeDef flash_erase_page(uint32_t page_addr)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0;

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = (page_addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase.NbPages   = 1;

    return HAL_FLASHEx_Erase(&erase, &page_error);
}

/**
 * @brief  写入机械零点到 Flash
 * @param  pos_rad: 机械零点绝对角度（rad）
 * @param  scale: 当前比例系数
 * @param  allow_erase: 1=页满允许擦除，0=页满返回 HAL_BUSY
 * @retval HAL 状态
 */
HAL_StatusTypeDef PosPersist_SaveEx(float pos_rad, float scale,
                                    float bias_rad, uint8_t allow_erase)
{
    /* 找到当前“最后一条有效记录”的 seq，并找到下一个空槽位 */
    uint32_t cap = rec_capacity();
    uint32_t next_idx = cap;
    uint32_t last_seq = 0;
    uint8_t  have_last = 0;

    for (uint32_t i = 0; i < cap; i++) {
        const pos_rec_t *r = rec_at(i);
        if (rec_is_empty(r)) {
            next_idx = i;
            break;
        }
        if (rec_is_valid(r)) {
            if (!have_last || (uint32_t)(r->seq - last_seq) < 0x80000000u) {
                last_seq = r->seq;
                have_last = 1;
            }
        }
    }

    if (next_idx >= cap) {
        if (!allow_erase) {
            return HAL_BUSY;
        }
        /* 页满：擦除后从头写 */
        HAL_FLASH_Unlock();
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
        HAL_StatusTypeDef st = flash_erase_page(POS_PERSIST_FLASH_BASE);
        HAL_FLASH_Lock();
        if (st != HAL_OK) {
            return st;
        }
        next_idx = 0;
        last_seq = 0;
        have_last = 0;
    }

    pos_rec_t w;
    memset(&w, 0xFF, sizeof(w));
    w.magic = POS_PERSIST_MAGIC_BIAS;
    w.seq   = (have_last ? (last_seq + 1u) : 1u);
    w.pos_mrad    = (int32_t)(pos_rad * 1000.0f);
    w.scale_milli = (int32_t)(scale * 1000.0f);
    w.crc32 = crc32_sw(&w, 16);
    w.rsv   = (uint32_t)((int32_t)(bias_rad * 1000.0f));

    uint32_t addr = POS_PERSIST_FLASH_BASE + next_idx * sizeof(pos_rec_t);

    /* 写入 3 个 doubleword */
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    HAL_StatusTypeDef st = HAL_OK;
    const uint64_t *dw = (const uint64_t*)&w;
    for (uint32_t k = 0; k < 3; k++) {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + k*8u, (uint64_t)dw[k]);
        if (st != HAL_OK) {
            break;
        }
    }

    HAL_FLASH_Lock();
    return st;
}

HAL_StatusTypeDef PosPersist_Save(float pos_rad, float scale, uint8_t allow_erase)
{
    return PosPersist_SaveEx(pos_rad, scale, 0.0f, allow_erase);
}
