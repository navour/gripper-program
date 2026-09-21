#include "pressure_history.h"
#include "main.h"
#include <string.h>

typedef struct {
    uint32_t seq, be_ms, le_ms;
    uint16_t flags, cal_mask, raw[MAT_RAW_WORDS];
    uint32_t crc;
} MatRecord;
typedef char mat_record_size_check[(sizeof(MatRecord) == 276u) ? 1 : -1];

static MatRecord queue[MAT_QUEUE_CAPACITY];
static uint64_t session;
static uint32_t produced, acknowledged, final_seq, drops;
static uint16_t command[MAT_COMMAND_WORDS];
static uint16_t device, layout_id, head, used, result, done_token;
static uint8_t recording, stopped, overflow, busy, have_command, writing, readers;

static uint32_t lock(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static void unlock(uint32_t p) { if (!p) __enable_irq(); }
static uint32_t words32(const uint16_t *p) { return ((uint32_t)p[0] << 16) | p[1]; }
static uint64_t command_session(void) {
    return ((uint64_t)words32(command + 1) << 32) | words32(command + 3);
}
static void put32(uint16_t *p, uint32_t v) { p[0] = (uint16_t)(v >> 16); p[1] = (uint16_t)v; }
static void put64(uint16_t *p, uint64_t v) { put32(p, (uint32_t)(v >> 32)); put32(p + 2, (uint32_t)v); }
static uint32_t crc_byte(uint32_t c, uint8_t b) {
    unsigned i;
    c ^= b;
    for (i = 0; i < 8u; ++i) c = (c >> 1) ^ ((c & 1u) ? 0xEDB88320u : 0u);
    return c;
}
static uint32_t crc_word(uint32_t c, uint16_t w) {
    return crc_byte(crc_byte(c, (uint8_t)(w >> 8)), (uint8_t)w);
}
static uint32_t record_crc(const MatRecord *r) {
    uint16_t prefix[16];
    uint32_t c = 0xFFFFFFFFu;
    unsigned i;
    prefix[0] = MAT_PROTOCOL_VER; prefix[1] = device; prefix[2] = layout_id;
    put64(prefix + 3, session); put32(prefix + 7, r->seq);
    put32(prefix + 9, r->be_ms); put32(prefix + 11, r->le_ms);
    prefix[13] = r->flags; prefix[14] = r->cal_mask; prefix[15] = MAT_RAW_WORDS;
    for (i = 0; i < 16u; ++i) c = crc_word(c, prefix[i]);
    for (i = 0; i < MAT_RAW_WORDS; ++i) c = crc_word(c, r->raw[i]);
    return c ^ 0xFFFFFFFFu;
}

void PressureHistory_Init(uint16_t slave, uint16_t layout) {
    memset(queue, 0, sizeof(queue)); memset(command, 0, sizeof(command));
    session = 0; produced = acknowledged = final_seq = drops = 0;
    head = used = result = done_token = 0;
    recording = stopped = overflow = busy = have_command = writing = readers = 0;
    device = slave; layout_id = layout;
}

uint8_t PressureHistory_SubmitCommand(const uint16_t *words) {
    uint32_t p = lock();
    uint8_t error = 0;
    if (busy) {
        error = memcmp(words, command, sizeof(command)) ? 0x06u : 0u;
    } else if (have_command && words[7] == command[7]) {
        error = memcmp(words, command, sizeof(command)) ? 0x03u : 0u;
    } else if (words[0] < MAT_START || words[0] > MAT_ACK_DURABLE ||
               !(words[1] | words[2] | words[3] | words[4]) ||
               (words[0] != MAT_ACK_DURABLE && (words[5] | words[6])) ||
               (words[0] == MAT_ACK_DURABLE && !(words[5] | words[6]))) {
        error = 0x03u;
    } else {
        memcpy(command, words, sizeof(command));
        have_command = busy = 1u; result = 1u;
    }
    unlock(p);
    return error;
}

/* Only short state transitions run with IRQs masked. No SPI, copying of raw
 * frames, CRC computation, UART waits or storage operations in this section. */
static void service(uint8_t boundary) {
    uint32_t p = lock();
    uint64_t requested;
    uint32_t seq;
    uint16_t rc = 0;
    if (!busy || writing || readers ||
        (!boundary && command[0] != MAT_ACK_DURABLE)) { unlock(p); return; }
    requested = command_session(); seq = words32(command + 5);
    if (command[0] == MAT_START) {
        if (requested != session) {
            if (recording || used) rc = 3u;
            else {
                session = requested; produced = acknowledged = final_seq = drops = 0;
                head = 0; recording = 1u; stopped = overflow = 0;
            }
        }
    } else if (requested != session) rc = 2u;
    else if (command[0] == MAT_STOP) {
        recording = 0; stopped = 1u; final_seq = produced;
    } else if (seq <= acknowledged) {
        /* Durable ACK retransmission: never pop a second frame. */
    } else if (!used || queue[head].seq != seq) rc = 4u;
    else {
        acknowledged = seq; head = (uint16_t)((head + 1u) % MAT_QUEUE_CAPACITY); --used;
    }
    result = rc; done_token = command[7]; busy = 0;
    unlock(p);
}
void PressureHistory_BeginCycle(void) { service(1u); }
void PressureHistory_Service(void) { service(0u); }

void PressureHistory_PushCycle(const uint16_t *r0, const uint16_t *r1,
                              const uint16_t *r2, const uint16_t *r3,
                              uint32_t be_ms, uint32_t le_ms,
                              uint16_t flags, uint16_t cal_mask) {
    MatRecord *r;
    uint32_t p = lock();
    if (!recording) { unlock(p); return; }
    ++produced;
    if (used == MAT_QUEUE_CAPACITY) {
        ++drops; overflow = stopped = 1u; recording = 0; final_seq = produced;
        unlock(p); return;
    }
    writing = 1u;
    r = &queue[(head + used) % MAT_QUEUE_CAPACITY];
    r->seq = produced;
    unlock(p);
    r->be_ms = be_ms; r->le_ms = le_ms;
    r->flags = (uint16_t)((flags & 0x000Fu) | 0x8000u); r->cal_mask = cal_mask & 15u;
    memcpy(r->raw, r0, 64u); memcpy(r->raw + 32, r1, 64u);
    memcpy(r->raw + 64, r2, 64u); memcpy(r->raw + 96, r3, 64u);
    r->crc = record_crc(r);
    p = lock();
    ++used; writing = 0;
    if (produced == 0xFFFFFFFFu) { recording = 0; stopped = 1u; final_seq = produced; }
    unlock(p);
}

void PressureHistory_ReadRegs(uint16_t start, uint16_t count, uint8_t *out) {
    uint16_t state[32], header[16];
    const MatRecord *r;
    uint16_t i, value, addr;
    uint32_t p;
    if (start < MAT_REG_FIRST || start >= MAT_REG_END || count > MAT_REG_END - start) return;
    memset(state, 0, sizeof(state)); memset(header, 0, sizeof(header));
    p = lock();
    ++readers;
    r = used ? &queue[head] : 0;
    state[0] = MAT_PROTOCOL_VER;
    state[1] = (uint16_t)(recording | (used ? 2u : 0u) | (overflow ? 4u : 0u) |
                          (stopped ? 8u : 0u) | (busy ? 16u : 0u));
    state[2] = used; state[3] = MAT_QUEUE_CAPACITY;
    put32(state + 4, produced); put32(state + 6, acknowledged);
    put32(state + 8, final_seq); put32(state + 10, drops);
    state[12] = layout_id; state[13] = MAT_RAW_WORDS;
    state[14] = result; state[15] = done_token;
    memcpy(state + 16, command, sizeof(command));
    header[0] = MAT_PROTOCOL_VER; header[1] = layout_id; put64(header + 2, session);
    unlock(p);
    if (r) {
        put32(header + 6, r->seq); put32(header + 8, r->be_ms); put32(header + 10, r->le_ms);
        header[12] = r->flags; header[13] = r->cal_mask; put32(header + 14, r->crc);
    }
    for (i = 0; i < count; ++i) {
        addr = start + i;
        if (addr < MAT_REG_HEAD) value = state[addr - MAT_REG_FIRST];
        else if (addr < MAT_REG_RAW) value = header[addr - MAT_REG_HEAD];
        else value = r ? r->raw[addr - MAT_REG_RAW] : 0u;
        out[2u*i] = (uint8_t)(value >> 8); out[2u*i + 1u] = (uint8_t)value;
    }
    p = lock(); --readers; unlock(p);
}
