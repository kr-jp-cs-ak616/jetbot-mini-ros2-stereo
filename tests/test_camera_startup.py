import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('camera_manager',Path(__file__).resolve().parents[1]/'scripts/manage_ros_cameras.py')
manager=importlib.util.module_from_spec(spec)
spec.loader.exec_module(manager)


class CameraStartupTests(unittest.TestCase):
    def test_waits_for_listening_socket_after_process_starts(self):
        with patch.object(manager,'publisher_pid',return_value=123), \
             patch.object(manager,'publisher_listening',side_effect=[False,False,True]) as listening, \
             patch.object(manager.time,'sleep'):
            manager.wait_for_publisher(timeout=5)
        self.assertEqual(listening.call_count,3)

    def test_detects_listener_even_with_full_accept_backlog(self):
        table='header\n0: 0100007F:223D 00000000:0000 0A 00000000:00000002\n'
        with patch.object(manager.Path,'read_text',return_value=table):
            self.assertTrue(manager.publisher_listening())

    def test_established_connection_is_not_a_listener(self):
        table='header\n0: 0100007F:223D 0100007F:1234 01 00000000:00000000\n'
        with patch.object(manager.Path,'read_text',return_value=table):
            self.assertFalse(manager.publisher_listening())

    def test_fails_if_receiver_never_becomes_ready(self):
        with patch.object(manager.time,'monotonic',side_effect=[0,0,2]), \
             patch.object(manager,'publisher_pid',return_value=None), \
             patch.object(manager.time,'sleep'):
            with self.assertRaisesRegex(RuntimeError,'port 8765'):
                manager.wait_for_publisher(timeout=1)
