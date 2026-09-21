"""Durable SQLite WAL journal, background HDF5 export, USB recording CLI.

The .wal.sqlite sidecar is the recovery source and is deliberately retained.
ACK is permitted only after SQLite synchronous=FULL commit returns.
"""
import argparse
from contextlib import closing
from datetime import datetime
import json
import os
from pathlib import Path
import sqlite3
import struct
import threading
import time

from tactile_protocol import MatrixClient, Frame, LAYOUTS, START, STOP, ACK

def default_output_path(slave, directory=None):
    """Local date/time filename; never reuse an existing recording or journal."""
    if slave not in (1, 2):
        raise ValueError('slave must be 1 or 2')
    now = datetime.now()
    base = Path(directory) if directory is not None else Path(__file__).resolve().parent / 'recordings'
    folder = base / now.strftime('%Y-%m-%d')
    stem = f"gripper{slave}_{now.strftime('%Y%m%d_%H%M%S_%f')[:-3]}"
    index = 0
    while True:
        suffix = f'_{index:02d}' if index else ''
        path = folder / f'{stem}{suffix}.h5'
        if not any(Path(str(path) + tail).exists() for tail in ('', '.wal.sqlite', '.partial.h5', '.wal.sqlite.lock')):
            return path
        index += 1

class Recorder:
    def __init__(self, output):
        # Import before starting a device session; fail early if dependencies missing.
        import h5py
        import numpy
        self.path = Path(output).resolve()
        self.db_path = Path(str(self.path) + '.wal.sqlite')
        self.partial = Path(str(self.path) + '.partial.h5')
        if self.path.exists() and not self.db_path.exists():
            raise FileExistsError('Existing HDF5 has no matching journal; choose a new path')
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lockfile = open(str(self.db_path) + '.lock', 'a+b')
        self.lockfile.seek(0)
        if os.name == 'nt':
            import msvcrt
            try:
                msvcrt.locking(self.lockfile.fileno(), msvcrt.LK_NBLCK, 1)
            except OSError:
                self.lockfile.close()
                raise RuntimeError('Recording file is in use by another process')
        else:
            import fcntl
            fcntl.flock(self.lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.db = sqlite3.connect(self.db_path)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.executescript('''
          CREATE TABLE IF NOT EXISTS sessions (
            slave INTEGER NOT NULL, session TEXT NOT NULL, final INTEGER,
            drops INTEGER, acknowledged INTEGER, complete INTEGER NOT NULL DEFAULT 0,
            PRIMARY KEY(slave,session));
          CREATE TABLE IF NOT EXISTS frames (
            id INTEGER PRIMARY KEY, slave INTEGER NOT NULL, session TEXT NOT NULL,
            sequence INTEGER NOT NULL, payload BLOB NOT NULL, crc INTEGER NOT NULL,
            received_ns INTEGER NOT NULL, UNIQUE(slave,session,sequence));
        ''')
        self.db.commit()
        self.error = None
        self.finished = threading.Event()
        self.thread = threading.Thread(target=self._export, name='tactile-hdf5', daemon=True)
        self.thread.start()
        self.closed = False

    def check(self):
        if self.error:
            raise RuntimeError(f'HDF5 writer failed; journal retained: {self.error}') from self.error

    def start_session(self, slave):
        self.check()
        # time_ns is a nonzero 64-bit identifier. Monotonic within this persistent journal.
        allocator = Path(os.environ.get('TACTILE_SESSION_DB', str(Path.home() / '.gripper_tactile_sessions.sqlite')))
        allocator.parent.mkdir(parents=True, exist_ok=True)
        with closing(sqlite3.connect(allocator, timeout=5)) as ids:
            ids.execute('PRAGMA synchronous=FULL')
            ids.execute('CREATE TABLE IF NOT EXISTS counter (id INTEGER PRIMARY KEY CHECK(id=1), value TEXT NOT NULL)')
            ids.execute('BEGIN IMMEDIATE')
            row = ids.execute('SELECT value FROM counter WHERE id=1').fetchone()
            last = int(row[0],16) if row else 0
            session = max(time.time_ns(), last+1)
            ids.execute('INSERT OR REPLACE INTO counter VALUES (1,?)',(f'{session:016x}',))
            ids.commit()
        if session >= 2**64:
            raise OverflowError('Session identifier exhausted')
        with self.db:
            self.db.execute('INSERT INTO sessions(slave,session) VALUES (?,?)', (slave, f'{session:016x}'))
        return session

    def has_session(self, slave, session):
        return self.db.execute('SELECT 1 FROM sessions WHERE slave=? AND session=?',
                               (slave, f'{session:016x}')).fetchone() is not None

    def last_sequence(self, slave, session):
        return self.db.execute('SELECT COALESCE(MAX(sequence),0) FROM frames WHERE slave=? AND session=?',
                               (slave, f'{session:016x}')).fetchone()[0]

    def append(self, frame):
        self.check()
        frame.validate()
        key = (frame.slave, f'{frame.session:016x}', frame.sequence)
        if not self.has_session(frame.slave, frame.session):
            raise RuntimeError('Unknown session; refusing to claim a complete recording')
        old = self.db.execute('SELECT payload,crc FROM frames WHERE slave=? AND session=? AND sequence=?', key).fetchone()
        if old:
            if old != (frame.payload(), frame.crc):
                raise RuntimeError('Same frame identity has different contents')
            return False
        expected = self.last_sequence(frame.slave, frame.session) + 1
        if frame.sequence != expected:
            raise RuntimeError(f'Missing matrix: expected {expected}, received {frame.sequence}')
        with self.db:
            self.db.execute('INSERT INTO frames(slave,session,sequence,payload,crc,received_ns) VALUES (?,?,?,?,?,?)',
                            (*key, frame.payload(), frame.crc, time.time_ns()))
        # Only after successful FULL synchronous commit may caller send ACK.
        return True

    def finish_session(self, slave, session, status):
        last = self.last_sequence(slave, session)
        complete = (not status.flags & (1 | 4 | 16) and bool(status.flags & 8) and
                    status.count == 0 and status.drops == 0 and
                    last == status.acknowledged == status.final == status.produced)
        with self.db:
            self.db.execute('UPDATE sessions SET final=?,drops=?,acknowledged=?,complete=? WHERE slave=? AND session=?',
                            (status.final, status.drops, status.acknowledged, int(complete), slave, f'{session:016x}'))
        return complete

    def _export(self):
        import h5py
        import numpy as np
        db = None
        try:
            db = sqlite3.connect(self.db_path)
            last_id = 0
            with h5py.File(self.partial, 'w') as h:
                meta = h.create_group('metadata')
                meta.attrs['protocol_version'] = 1
                meta.attrs['units'] = 'raw_sensor_counts'
                meta.attrs['time_basis'] = 'be_ms/le_ms: MCU uint32 milliseconds; receive_time: Unix nanoseconds'
                meta.attrs['recovery_journal'] = self.db_path.name
                groups = {}
                while True:
                    rows = db.execute('SELECT id,payload,crc,received_ns FROM frames WHERE id>? ORDER BY id LIMIT 128', (last_id,)).fetchall()
                    for rowid, payload, crc, received in rows:
                        f = Frame.from_payload(payload, crc)
                        name = f'gripper{f.slave}/{f.session:016x}'
                        if name not in groups:
                            g = h.require_group(name)
                            g.attrs['regions'] = json.dumps(LAYOUTS[f.layout])
                            g.attrs['layout_id'] = f.layout
                            g.attrs['session_id'] = np.uint64(f.session)
                            g.attrs['complete'] = False
                            g.create_dataset('raw', (0,4,8,4), maxshape=(None,4,8,4), dtype='uint16', chunks=(64,4,8,4), compression='lzf', fletcher32=True)
                            for field,dtype in [('sequence','uint32'),('be_ms','uint32'),('le_ms','uint32'),('receive_time','int64'),('sample_flags','uint16'),('cal_mask','uint16'),('crc32','uint32')]:
                                g.create_dataset(field, (0,), maxshape=(None,), dtype=dtype, chunks=(256,), fletcher32=True)
                            groups[name] = g
                        g = groups[name]
                        index = len(g['sequence'])
                        values = dict(raw=np.asarray(f.raw,dtype='uint16').reshape(4,8,4), sequence=f.sequence,
                                      be_ms=f.be_ms, le_ms=f.le_ms, receive_time=received,
                                      sample_flags=f.flags, cal_mask=f.cal_mask, crc32=f.crc)
                        for field,value in values.items():
                            ds = g[field]
                            ds.resize(index+1, axis=0)
                            ds[index] = value
                        last_id = rowid
                    for slave,session,final,drops,ack,complete in db.execute('SELECT * FROM sessions'):
                        g = h.require_group(f'gripper{slave}/{session}')
                        g.attrs['complete'] = bool(complete)
                        if final is not None:
                            g.attrs['final_sequence'] = final
                            g.attrs['drop_count'] = drops
                            g.attrs['acknowledged'] = ack
                    h.flush()
                    if self.finished.is_set() and not rows:
                        break
                    self.finished.wait(0.05)
            # Closed HDF5 is flushed through the OS. Journal remains for recovery.
            with open(self.partial, 'r+b') as f:
                os.fsync(f.fileno())
            verify_h5(self.partial)
            os.replace(self.partial, self.path)
        except BaseException as exc:
            self.error = exc
        finally:
            if db is not None:
                db.close()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.finished.set()
        self.thread.join()
        self.db.close()
        self.lockfile.close()
        self.check()


def verify_h5(path):
    import h5py
    report = {}
    with h5py.File(path, 'r') as h:
        for slave in (1,2):
            root = h.get(f'gripper{slave}')
            if root is None:
                continue
            for session,g in root.items():
                n = len(g['sequence']) if 'sequence' in g else 0
                if n:
                    if g['raw'].shape != (n,4,8,4) or any(len(g[k]) != n for k in ('be_ms','le_ms','receive_time','sample_flags','cal_mask','crc32')):
                        raise RuntimeError('HDF5 dataset length mismatch')
                    for i in range(n):
                        f = Frame(slave,int(g.attrs['layout_id']),int(session,16),int(g['sequence'][i]),
                                  int(g['be_ms'][i]),int(g['le_ms'][i]),int(g['sample_flags'][i]),int(g['cal_mask'][i]),
                                  tuple(int(x) for x in g['raw'][i].flat),int(g['crc32'][i]))
                        f.validate()
                        if f.sequence != i+1:
                            raise RuntimeError('HDF5 sequence gap/duplicate')
                if g.attrs.get('complete',False) and (n != g.attrs.get('final_sequence',-1) or g.attrs.get('drop_count',-1) != 0 or n != g.attrs.get('acknowledged',-1)):
                    raise RuntimeError('HDF5 completion assertion is inconsistent')
                report[f'gripper{slave}/{session}'] = {'frames': n, 'complete': bool(g.attrs.get('complete',False))}
    return report


def capture(client, recorder, stop_event, resume=False, seconds=None, notify=print):
    status = client.status()
    session = client.session()
    if resume:
        if not session or not recorder.has_session(client.slave, session):
            raise RuntimeError('Resume needs the original journal and a surviving gripper session')
        if recorder.last_sequence(client.slave, session) < status.acknowledged:
            raise RuntimeError('Journal is missing data already released by the gripper')
    else:
        if status.count or status.flags & 1:
            raise RuntimeError('Existing gripper session must be recovered first (--resume)')
        session = recorder.start_session(client.slave)
        client.command(START, session)
    notify(f'Session {session:016x}; slave {client.slave}')
    began, stopping, next_status, frames = time.monotonic(), False, 0.0, 0
    while True:
        recorder.check()
        if not stopping and (stop_event.is_set() or (seconds is not None and time.monotonic()-began >= seconds)):
            client.command(STOP, session)
            stopping = True
        frame = client.frame()
        if frame is not None:
            if frame.session != session:
                raise RuntimeError('Gripper session changed; recording is incomplete')
            recorder.append(frame)
            client.command(ACK, session, frame.sequence)
            frames += 1
            if frames % 100 == 0:
                notify(f'Archived sequence {frame.sequence}')
        else:
            time.sleep(0.002)
        now = time.monotonic()
        if now >= next_status or (stopping and frame is None):
            status = client.status()
            if client.session() != session:
                raise RuntimeError('Gripper reset; recording is incomplete')
            next_status = now + 1.0
            if status.flags & 8:
                stopping = True
            if stopping and not status.count:
                complete = recorder.finish_session(client.slave, session, status)
                notify(f'Finished: complete={complete}, final={status.final}, drops={status.drops}')
                if not complete:
                    raise RuntimeError('Recording incomplete: overflow, sequence gap or missing ACK')
                return


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', help='HDF5 output; fresh recordings default to recordings/date/gripperN_timestamp.h5')
    p.add_argument('--port')
    p.add_argument('--slave',type=int,choices=(1,2),default=1)
    p.add_argument('--baud',type=int,default=115200)
    p.add_argument('--seconds',type=float,default=10.0)
    p.add_argument('--resume',action='store_true')
    p.add_argument('--recover',action='store_true',help='Rebuild HDF5 from retained journal without hardware')
    p.add_argument('--verify',action='store_true')
    a = p.parse_args()
    if a.output is None:
        if a.resume or a.recover or a.verify:
            p.error('--output is required for --resume, --recover or --verify')
        a.output = str(default_output_path(a.slave))
    print(f'HDF5 output: {Path(a.output).resolve()}')
    if a.verify:
        print(json.dumps(verify_h5(a.output),indent=2)); return
    if a.recover and not Path(a.output+'.wal.sqlite').exists():
        p.error('Recovery journal does not exist')
    if not a.recover and not a.port:
        p.error('--port is required for USB capture')
    recorder = Recorder(a.output)
    try:
        if not a.recover:
            import serial
            import signal
            from tactile_protocol import SerialTransport
            stop = threading.Event()
            signal.signal(signal.SIGINT,lambda *_: stop.set())
            with serial.Serial(a.port,a.baud,timeout=0.15,write_timeout=0.5) as s:
                capture(MatrixClient(SerialTransport(s),a.slave),recorder,stop,a.resume,a.seconds)
    finally:
        recorder.close()
    print(json.dumps(verify_h5(a.output),indent=2))

if __name__ == '__main__':
    main()
