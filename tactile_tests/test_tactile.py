import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import struct
import tempfile
import threading
import unittest
import zlib
from dataclasses import replace
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(next(ROOT.glob('*id1slave1'))/'PC_Tool'))
from tactile_protocol import Frame,MatrixClient,Status,ProtocolError,packet,crc16,START,STOP,ACK
from tactile_recorder import Recorder,capture,verify_h5
from arm_history import ArmHistory

BASE=Path(os.environ.get('TACTILE_TEST_ROOT',Path(__file__).resolve().parents[1]))

def history(slave=1):
    return ArmHistory(BASE/f'matrix_validation/gripper{slave}/JIAZHUA_RTOS.axf',slave)

def frame(h):
    return Frame.from_registers(h.slave,h.read(0x170,80),h.read(0x1c0,64))

class FirmwareTests(unittest.TestCase):
    def test_empty_and_crc_both_layouts(self):
        for slave in (1,2):
            h=history(slave)
            self.assertEqual(h.read(0x180,128),[0]*128)
            self.assertEqual(h.read(0x17c,1),[0])
            self.assertEqual(h.submit(START,0x123456789ABCDEF0,0,1),0)
            h.service();self.assertTrue(h.read(0x151,1)[0]&16)
            h.boundary();h.push()
            f=frame(h)
            self.assertEqual(f.session,0x123456789ABCDEF0)
            self.assertEqual(f.raw,tuple(range(128)))
            self.assertEqual(f.layout,slave)

    def test_fixed_head_and_ring_wrap(self):
        h=history();h.submit(START,1,0,1);h.boundary()
        for seq in range(1,41):
            h.push(seq)
            f=frame(h)
            self.assertEqual(f.sequence,seq)
            self.assertEqual(frame(h),f)
            h.submit(ACK,1,seq,seq+1);h.service()
            self.assertEqual(h.read(0x17c,1),[0])

    def test_token_retry_busy_and_duplicate_ack(self):
        h=history();self.assertEqual(h.submit(START,1,0,7),0)
        self.assertEqual(h.submit(START,1,0,7),0)
        self.assertEqual(h.submit(STOP,1,0,8),6)
        h.boundary();h.push();h.push(10)
        self.assertEqual(h.submit(ACK,1,1,7),3)
        self.assertEqual(h.submit(ACK,1,1,8),0);h.service()
        self.assertEqual(frame(h).sequence,2)
        self.assertEqual(h.submit(ACK,1,1,8),0);h.service()
        self.assertEqual(frame(h).sequence,2)
        h.submit(ACK,1,1,9);h.service()
        self.assertEqual(frame(h).sequence,2)
        h.submit(ACK,2,2,10);h.service();self.assertEqual(h.read(0x15e,2),[2,10])
        h.submit(ACK,1,3,11);h.service();self.assertEqual(h.read(0x15e,2),[4,11])

    def test_stop_boundary_and_session_reuse(self):
        h=history();h.submit(START,1,0,1);h.boundary();h.push()
        h.submit(STOP,1,0,2);h.service()
        self.assertTrue(h.read(0x151,1)[0]&1)
        h.push();h.boundary()
        self.assertEqual(h.read(0x158,2),[0,2])
        h.submit(START,2,0,3);h.boundary();self.assertEqual(h.read(0x15e,1),[3])
        h.submit(START,1,0,4);h.boundary();h.push()
        self.assertEqual(h.read(0x154,2),[0,2])

    def test_overflow_never_overwrites(self):
        h=history();h.submit(START,1,0,1);h.boundary()
        for i in range(10): h.push(i)
        s=Status.parse(h.read(0x150,16))
        self.assertEqual((s.count,s.produced,s.final,s.drops),(4,5,5,1))
        self.assertEqual(s.flags&13,12)
        self.assertEqual(frame(h).raw[0],0)
        for seq in range(1,5): h.submit(ACK,1,seq,seq+1);h.service()
        self.assertEqual(h.read(0x151,1)[0],12)

    def test_failed_samples_and_sequence_limit(self):
        h=history();h.submit(START,1,0,1);h.boundary();h.push(flags=0)
        self.assertEqual(frame(h).flags,0x8000)
        h.submit(ACK,1,1,2);h.service()
        h.u.mem_write(h.symbols['produced'],struct.pack('<I',0xFFFFFFFE))
        h.push();self.assertEqual(frame(h).sequence,0xFFFFFFFF)
        self.assertEqual(h.read(0x151,1)[0]&9,8)
        h.push();self.assertEqual(frame(h).sequence,0xFFFFFFFF)

class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.output=Path(self.tmp.name)/'test.h5'
        os.environ['TACTILE_SESSION_DB']=str(Path(self.tmp.name)/'ids.sqlite')

    def tearDown(self): self.tmp.cleanup()

    def make_frame(self,session,seq=1,slave=1):
        f=Frame(slave,slave,session,seq,123,456,0x800f,15,tuple(range(128)),0)
        return replace(f,crc=zlib.crc32(f.payload()))

    def test_durable_dedup_h5_and_corrupt_file_recovery(self):
        r=Recorder(self.output);session=r.start_session(1);f=self.make_frame(session)
        self.assertTrue(r.append(f));self.assertFalse(r.append(f))
        bad=replace(f,raw=(999,)+f.raw[1:]);bad=replace(bad,crc=zlib.crc32(bad.payload()))
        with self.assertRaises(RuntimeError): r.append(bad)
        with self.assertRaises(RuntimeError): r.append(self.make_frame(session,3))
        self.assertTrue(r.finish_session(1,session,Status(8,0,4,1,1,1,0,1,0,1)))
        r.close();report=verify_h5(self.output)
        self.assertTrue(next(iter(report.values()))['complete'])
        self.output.write_bytes(b'corrupt interrupted HDF5')
        r=Recorder(self.output);r.close()
        self.assertEqual(verify_h5(self.output),report)

    def test_exclusive_journal_and_disk_failure(self):
        r=Recorder(self.output)
        with self.assertRaises(RuntimeError): Recorder(self.output)
        session=r.start_session(1)
        r.db.execute('PRAGMA query_only=ON')
        with self.assertRaises(sqlite3.OperationalError): r.append(self.make_frame(session))
        self.assertEqual(r.last_sequence(1,session),0)
        r.close()

    def test_two_layouts_and_incomplete_overflow(self):
        r=Recorder(self.output)
        for slave in (1,2):
            session=r.start_session(slave)
            for seq in (1,2):r.append(self.make_frame(session,seq,slave))
            self.assertFalse(r.finish_session(slave,session,Status(12,0,4,3,2,3,1,slave,0,3)))
        r.close();report=verify_h5(self.output)
        self.assertEqual(len(report),2)
        self.assertTrue(all(x=={'frames':2,'complete':False} for x in report.values()))

    def test_end_to_end_real_firmware_lost_ack_response(self):
        for slave in (1,2):
            output=self.output.with_name(f'g{slave}.h5')
            r=Recorder(output);h=history(slave)
            lost=False
            def exchange(req):
                nonlocal lost
                self.assertEqual(crc16(req[:-2]),int.from_bytes(req[-2:],'little'))
                if req[1]==3:
                    addr,count=struct.unpack('>HH',req[2:6]); regs=h.read(addr,count)
                    return packet(bytes([slave,3,count*2])+struct.pack(f'>{count}H',*regs))
                words=struct.unpack('>8H',req[7:-2]);op=words[0]
                session=int.from_bytes(req[9:17],'big');seq=(words[5]<<16)|words[6]
                if op==ACK:
                    self.assertGreaterEqual(r.last_sequence(slave,session),seq)
                rc=h.submit(op,session,seq,words[7]);h.boundary()
                if rc:return packet(bytes([slave,0x90,rc]))
                if op==START and h.read(0x152,1)[0]==0:
                    for i in range(3):h.push(i)
                if op==ACK and not lost:
                    lost=True;raise TimeoutError('injected lost ACK reply')
                return packet(req[:6])
            capture(MatrixClient(exchange,slave),r,threading.Event(),seconds=0,notify=lambda _:None)
            r.close();self.assertTrue(lost)
            self.assertEqual(next(iter(verify_h5(output).values())),{'frames':3,'complete':True})

    def test_process_crash_recovers_committed_journal(self):
        script = """
import os,sys,zlib
from dataclasses import replace
from tactile_recorder import Recorder
from tactile_protocol import Frame
r=Recorder(sys.argv[1]);s=r.start_session(1)
f=Frame(1,1,s,1,1,2,0x800f,15,tuple(range(128)),0)
r.append(replace(f,crc=zlib.crc32(f.payload())))
os._exit(0)
"""
        subprocess.run([sys.executable,'-c',script,str(self.output)],check=True, env={**os.environ,'PYTHONPATH':os.pathsep.join(sys.path)})
        r=Recorder(self.output);r.close()
        self.assertEqual(next(iter(verify_h5(self.output).values())),{'frames':1,'complete':False})

    def test_crc_corruption(self):
        f=self.make_frame(1)
        with self.assertRaises(ProtocolError):replace(f,crc=f.crc^1).validate()
        payload=bytearray(f.payload());payload[-1]^=1
        with self.assertRaises(ProtocolError):Frame.from_payload(payload,f.crc)

if __name__=='__main__':unittest.main(verbosity=2)
