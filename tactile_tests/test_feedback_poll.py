"""Feedback polling regression tests; fake serial only, no connected hardware."""
import importlib.util
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]

class Var:
    def __init__(self, value): self.value = value
    def get(self): return self.value
    def set(self, value): self.value = value

class Port:
    def __init__(self, response=b'', error=False):
        self.response = response
        self.timeout = 0.4
        self.writes = []
        self.error = error
    def reset_input_buffer(self): pass
    def write(self, request):
        self.writes.append(request)
        if self.error: raise OSError('simulated serial error')
    def flush(self): pass
    def read(self, count):
        if self.response:
            chunk, self.response = self.response[:count], self.response[count:]
            return chunk
        time.sleep(self.timeout)
        return b''

class FeedbackMixin:
    def setUp(self):
        self.app = self.mod.App.__new__(self.mod.App)
        self.app.ser = Port()
        self.app.rx_lock = threading.Lock()
        self.app.manual_io_pending = threading.Event()
        self.app.timeout_var = Var('0.003')
        self.app._slave = lambda: 2
        self.app._poll_fail_count = 0
        self.app.live_status_var = Var('unchanged')
        self.app.root = SimpleNamespace(after=lambda delay, callback, *args: callback(*args))
    def poll(self): return self.app._poll_read_regs(0x106, 2)
    def response(self, payload=b'\x02\x03\x04\x00\x01\x00\x02'):
        return self.mod.add_crc(payload)
    def test_matrix_lock_skip_does_not_report_failure(self):
        self.app.rx_lock.acquire()
        try:
            for _ in range(10): self.assertIsNone(self.poll())
            self.assertEqual(self.app._poll_fail_count, 0)
            self.assertEqual(self.app.live_status_var.get(), 'unchanged')
            self.assertEqual(self.app.ser.writes, [])
            self.assertTrue(self.app.rx_lock.locked())
        finally: self.app.rx_lock.release()
    def test_manual_priority_skip(self):
        self.app.manual_io_pending.set()
        for _ in range(5): self.poll()
        self.assertEqual(self.app._poll_fail_count, 0)
        self.assertEqual(self.app.ser.writes, [])
    def test_skip_preserves_real_failure_count(self):
        self.app._poll_fail_count = 3
        self.app.rx_lock.acquire()
        self.poll()
        self.app.rx_lock.release()
        self.assertEqual(self.app._poll_fail_count, 3)
        self.poll()
        self.assertEqual(self.app._poll_fail_count, 4)
        self.assertNotEqual(self.app.live_status_var.get(), 'unchanged')
    def test_disconnected_is_not_a_sent_timeout(self):
        self.app.ser = None
        self.assertIsNone(self.poll())
        self.assertEqual(self.app._poll_fail_count, 0)
    def test_real_timeouts_report_failure_and_restore_port(self):
        for _ in range(3): self.poll()
        self.assertEqual(self.app.live_status_var.get(), 'unchanged')
        self.poll()
        self.assertEqual(self.app._poll_fail_count, 4)
        self.assertEqual(len(self.app.ser.writes), 4)
        self.assertNotEqual(self.app.live_status_var.get(), 'unchanged')
        self.assertEqual(self.app.ser.timeout, 0.4)
        self.assertFalse(self.app.rx_lock.locked())
    def test_valid_reply_recovers(self):
        self.app._poll_fail_count = 4
        self.app.ser.response = self.response()
        self.assertEqual(self.poll(), [1, 2])
        self.assertEqual(self.app._poll_fail_count, 0)
        self.assertNotEqual(self.app.live_status_var.get(), 'unchanged')
        self.assertEqual(self.app.ser.timeout, 0.4)
        self.assertFalse(self.app.rx_lock.locked())
    def test_bad_crc_wrong_slave_function_and_count_are_failures(self):
        valid = self.response()
        invalid = [valid[:-1] + bytes([valid[-1] ^ 1]),
                   self.response(b'\x01\x03\x04\x00\x01\x00\x02'),
                   self.response(b'\x02\x83\x02'),
                   self.response(b'\x02\x03\x02\x00\x01'),
                   self.response(b'\x02\x06\x01\x06\x00\x01')]
        for count, response in enumerate(invalid, 1):
            self.app.ser.response = response
            self.assertIsNone(self.poll())
            self.assertEqual(self.app._poll_fail_count, count)
    def test_serial_error_releases_lock_and_timeout(self):
        self.app.ser.error = True
        self.assertIsNone(self.poll())
        self.assertEqual(self.app._poll_fail_count, 1)
        self.assertEqual(self.app.ser.timeout, 0.4)
        self.assertFalse(self.app.rx_lock.locked())
    def test_quiet_position_read_can_skip_without_parse_error(self):
        self.app.log = lambda msg: self.fail('Unexpected parse error: ' + msg)
        self.app.manual_io_pending.set()
        self.assertIsNone(self.app._read_pos_deg(do_log=False))
        self.assertEqual(self.app.ser.writes, [])

projects = sorted(ROOT.glob('5_*slave*/PC_Tool/3_gripper.py'))
if len(projects) != 2:
    raise RuntimeError('Expected both gripper GUI source files')
for index, path in enumerate(projects, 1):
    spec = importlib.util.spec_from_file_location('feedback_gui_' + str(index), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    name = 'FeedbackGripper' + str(index)
    globals()[name] = type(name, (FeedbackMixin, unittest.TestCase), {'mod': module})

if __name__ == '__main__':
    unittest.main()
