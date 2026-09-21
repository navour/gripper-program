"""Matrix protocol v1. All Modbus addresses are zero based."""
from dataclasses import dataclass
import struct
import time
import zlib

STATUS, COMMAND, HEAD, SECOND = 0x150, 0x160, 0x170, 0x1C0
START, STOP, ACK = 1, 2, 3
LAYOUTS = {1: ('BE_C', 'BE_D', 'BE_E', 'LE_C'), 2: ('BE_B', 'BE_C', 'BE_E', 'LE_C')}
PREFIX = struct.Struct('>HHHQIIIHHH')

class ProtocolError(RuntimeError):
    pass

class ModbusError(ProtocolError):
    def __init__(self, code):
        self.code = code
        super().__init__(f'Modbus exception 0x{code:02X}')

def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ (0xA001 if c & 1 else 0)
    return c

def packet(data):
    return data + struct.pack('<H', crc16(data))

def u32(words):
    return (words[0] << 16) | words[1]

def u64(words):
    return (u32(words[:2]) << 32) | u32(words[2:4])

@dataclass(frozen=True)
class Frame:
    slave: int
    layout: int
    session: int
    sequence: int
    be_ms: int
    le_ms: int
    flags: int
    cal_mask: int
    raw: tuple
    crc: int
    version: int = 1

    def payload(self):
        return PREFIX.pack(self.version, self.slave, self.layout, self.session,
                           self.sequence, self.be_ms, self.le_ms, self.flags,
                           self.cal_mask, 128) + struct.pack('>128H', *self.raw)

    def validate(self):
        if (self.version != 1 or self.slave not in (1, 2) or self.layout != self.slave or
                not self.session or not self.sequence or len(self.raw) != 128 or
                not self.flags & 0x8000 or self.flags & ~0x800F or self.cal_mask & ~15):
            raise ProtocolError('Invalid frame identity, version, flags or layout')
        if zlib.crc32(self.payload()) != self.crc:
            raise ProtocolError('Whole-record CRC32 mismatch')
        return self

    @classmethod
    def from_payload(cls, payload, crc):
        if len(payload) != 288:
            raise ProtocolError('Expected 288 canonical bytes')
        ver, slave, layout, session, seq, be, le, flags, cal, count = PREFIX.unpack(payload[:32])
        if count != 128:
            raise ProtocolError('Invalid matrix size')
        return cls(slave, layout, session, seq, be, le, flags, cal,
                   struct.unpack('>128H', payload[32:]), crc, ver).validate()

    @classmethod
    def from_registers(cls, slave, first, second):
        if len(first) != 80 or len(second) != 64:
            raise ProtocolError('Incomplete matrix halves')
        return cls(slave, first[1], u64(first[2:6]), u32(first[6:8]),
                   u32(first[8:10]), u32(first[10:12]), first[12], first[13],
                   tuple(first[16:] + second), u32(first[14:16]), first[0]).validate()

@dataclass(frozen=True)
class Status:
    flags: int
    count: int
    capacity: int
    produced: int
    acknowledged: int
    final: int
    drops: int
    layout: int
    result: int
    done_token: int

    @classmethod
    def parse(cls, r):
        if len(r) != 16 or r[0] != 1 or r[13] != 128:
            raise ProtocolError('Matrix protocol is unavailable or incompatible')
        return cls(r[1], r[2], r[3], u32(r[4:6]), u32(r[6:8]),
                   u32(r[8:10]), u32(r[10:12]), r[12], r[14], r[15])

class SerialTransport:
    """Caller serializes every transaction with controls on the SAME bus lock."""
    def __init__(self, serial_port, timeout=0.15):
        self.serial = serial_port
        self.timeout = timeout
        self.last_end = 0.0

    def __call__(self, request):
        s = self.serial
        time.sleep(max(0, 0.00175 - (time.monotonic() - self.last_end)))
        old = s.timeout
        try:
            s.timeout = min(0.01, self.timeout)
            s.reset_input_buffer()
            s.write(request)
            s.flush()
            deadline = time.monotonic() + self.timeout
            buf = bytearray()
            while time.monotonic() < deadline:
                buf.extend(s.read(1))
                if len(buf) >= 3:
                    length = 5 if buf[1] & 0x80 else (5 + buf[2] if buf[1] == 3 else 8)
                    if len(buf) == length:
                        return bytes(buf)
            raise TimeoutError('RS485 response timeout')
        finally:
            s.timeout = old
            self.last_end = time.monotonic()

class MatrixClient:
    """One command in flight per gripper; transaction callback may yield to controls."""
    def __init__(self, exchange, slave):
        if slave not in (1, 2):
            raise ValueError('slave must be 1 or 2')
        self.exchange, self.slave = exchange, slave
        self.token = None
        self.prefetched = None

    def _request(self, body):
        response = self.exchange(packet(bytes([self.slave]) + body))
        if not response or len(response) < 5 or crc16(response[:-2]) != int.from_bytes(response[-2:], 'little'):
            raise ProtocolError('Missing or corrupt Modbus response')
        if response[0] != self.slave:
            raise ProtocolError('Wrong slave')
        if response[1] == body[0] | 0x80:
            raise ModbusError(response[2])
        if response[1] != body[0]:
            raise ProtocolError('Wrong function')
        return response

    def read(self, address, count):
        for attempt in range(3):
            try:
                r = self._request(struct.pack('>BHH', 3, address, count))
                if len(r) != 5 + 2 * count or r[2] != 2 * count:
                    raise ProtocolError('Wrong register count')
                return list(struct.unpack(f'>{count}H', r[3:-2]))
            except ModbusError:
                raise
            except (TimeoutError, ProtocolError):
                if attempt == 2:
                    raise
                time.sleep(0.001)

    def status(self):
        s = Status.parse(self.read(STATUS, 16))
        if s.layout != self.slave:
            raise ProtocolError('Wrong device layout')
        return s

    def session(self):
        return u64(self.read(HEAD + 2, 4))

    def frame(self):
        first, self.prefetched = self.prefetched, None
        if first is None:
            first = self.read(HEAD, 80)
        if not first[12] & 0x8000:
            return None
        return Frame.from_registers(self.slave, first, self.read(SECOND, 64))

    def command(self, opcode, session, sequence=0, timeout=2.0):
        if self.token is None:
            self.token = (self.read(COMMAND, 8)[7] + 1) & 0xFFFF
        token = self.token
        words = [opcode, *struct.unpack('>4H', struct.pack('>Q', session)),
                 sequence >> 16, sequence & 0xFFFF, token]
        body = struct.pack('>BHHB8H', 16, COMMAND, 8, 16, *words)
        deadline, accepted, last_error = time.monotonic() + timeout, False, None
        while time.monotonic() < deadline:
            try:
                if not accepted:
                    r = self._request(body)
                    if r[2:6] != struct.pack('>HH', COMMAND, 8) or len(r) != 8:
                        raise ProtocolError('Wrong command echo')
                    accepted = True
                # Fast ACK completion: only a durable ACK can advance this head.
                if opcode == ACK:
                    first = self.read(HEAD, 80)
                    if u64(first[2:6]) != session:
                        raise RuntimeError('Gripper reset or session changed during ACK')
                    if first[12] & 0x8000 and u32(first[6:8]) > sequence:
                        self.prefetched = first
                        self.token = (token + 1) & 0xFFFF
                        return None
                s = self.status()
                if not s.flags & 16 and s.done_token == token:
                    if s.result:
                        raise RuntimeError(f'Matrix command rejected: result={s.result}')
                    if opcode != ACK and self.session() != session:
                        raise RuntimeError('Command result belongs to a different session')
                    valid = ((opcode == START and bool(s.flags & 1)) or
                             (opcode == STOP and bool(s.flags & 8) and not s.flags & 1) or
                             (opcode == ACK and s.acknowledged >= sequence))
                    if not valid:
                        raise RuntimeError('Command result has unexpected state')
                    self.token = (token + 1) & 0xFFFF
                    return s
            except ModbusError as exc:
                if exc.code != 6:
                    raise
                last_error, accepted = exc, False
            except (TimeoutError, ProtocolError) as exc:
                last_error, accepted = exc, False
            time.sleep(0.001)
        # The caller must stop/reconnect after an ambiguous completion. Never send
        # different parameters using this token, and never silently create a session.
        raise TimeoutError(f'Command completion unknown; reconnect to recover: {last_error}')
