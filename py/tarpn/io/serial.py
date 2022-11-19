import fcntl
import logging
import threading
import time
from abc import ABC
from collections import deque
from time import sleep
from typing import Optional

import serial
import serial.tools.list_ports

from tarpn.io import IOProtocol
from tarpn.logging import LoggingMixin
from tarpn.thread.scheduler import Scheduler, CloseableThreadLoop
from tarpn.thread.latch import CountDownLatch
from tarpn.util import BackoffGenerator


class SerialMetrics:
    def __init__(self, port_id: int):
        self.port_id = port_id

    def serial_read(self, n: int, ms: float):
        pass

    def serial_write(self, n: int, ms: float):
        pass

    def serial_unsent_queue_size(self, n: int):
        pass

    def open_error(self):
        pass

    def write_timeout(self):
        pass

    def read_error(self):
        pass

    def write_error(self):
        pass

    def serial_buffer_waiting(self, in_waiting: int, out_waiting: int):
        pass


def find_serial_device_by_serial_number(ser_num: str) -> Optional[str]:
    for port in serial.tools.list_ports.comports():
        if port.serial_number == ser_num:
            return port.device
    return None


def lock_serial(ser: serial.Serial):
    fcntl.flock(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)


def unlock_serial(ser: serial.Serial):
    fcntl.flock(ser.fileno(), fcntl.LOCK_UN)


class SafeSerial(serial.Serial):
    """
    A context-manager wrapper around serial.Serial that ensures exclusive access
    to the serial port.
    """
    def __enter__(self):
        try:
            lock_serial(self)
            return self
        except BlockingIOError:
            self.close()
            raise RuntimeError(f"Cannot open {self.name}, resource is busy.")

    def __exit__(self, exc_type, exc_val, exc_tb):
        unlock_serial(self)


class SerialLoop(CloseableThreadLoop, ABC):
    def __init__(self,
                 thread_name: str,
                 ser: serial.Serial,
                 protocol: IOProtocol,
                 open_event: threading.Event,
                 error_event: threading.Event,
                 closed_latch: CountDownLatch):
        CloseableThreadLoop.__init__(self, name=thread_name)
        self.ser = ser
        self.protocol = protocol
        self.open_event = open_event
        self.error_event = error_event
        self.closed_latch = closed_latch

    def close(self):
        super().close()
        self.closed_latch.countdown()


class SerialReadLoop(SerialLoop, LoggingMixin):
    def __init__(self, port_id: int, read_limit: int, *args, **kwargs):
        SerialLoop.__init__(self, *args, **kwargs)
        LoggingMixin.__init__(self, logger=logging.getLogger("serial"), extra={"port": port_id})
        self.metrics = SerialMetrics(port_id)
        self.read_limit = read_limit

    def iter_loop(self):
        if self.open_event.wait(3.0):
            try:
                if self.read_limit > 0:
                    n = self.read_limit
                else:
                    n = max(8, self.ser.in_waiting)
                t0 = time.time()
                data = self.ser.read(n)
                t1 = time.time()
                dt = (t1 - t0) * 1000
                if len(data) > 0:
                    self.trace(f"Read {len(data)} bytes in {dt} ms: {data}")
                    self.protocol.handle_bytes(data)
                # Move metrics out of the hot path
                self.metrics.serial_read(len(data), dt)
            except serial.SerialException:
                self.error("Failed to read bytes from serial device")
                self.metrics.write_error()
                self.error_event.set()


class SerialWriteLoop(SerialLoop, LoggingMixin):
    def __init__(self, port_id: int, max_buffer: int, *args, bitrate: int, **kwargs):
        SerialLoop.__init__(self, *args, **kwargs)
        LoggingMixin.__init__(self, logger=logging.getLogger("serial"), extra={"port": port_id})
        self.unsent = deque(maxlen=20)
        self.bitrate = bitrate
        self.max_buffer = max_buffer
        self.timeout_backoff = BackoffGenerator(self.ser.write_timeout, 1.5, 10 * self.ser.write_timeout)
        self.metrics = SerialMetrics(port_id)

    def iter_loop(self):
        if self.open_event.wait(0.100):
            if len(self.unsent) > 0:
                to_write = self.unsent.popleft()
                self.metrics.serial_unsent_queue_size(len(self.unsent))
            else:
                # TODO get the trace ID from upper layers and set it here
                to_write = self.protocol.next_bytes_to_write(self.max_buffer)
            if to_write is not None and len(to_write) > 0:
                t0 = time.time()
                try:
                    self.ser.write(to_write)
                    self.ser.flush()
                    t1 = time.time()
                    dt = (t1 - t0) * 1000
                    self.metrics.serial_write(len(to_write), dt)

                    # If we successfully wrote to the serial, check if we need to reset the timeout backoff
                    if self.timeout_backoff.reset():
                        self.ser.write_timeout = self.timeout_backoff.initial_wait

                    if self.is_trace_logging():
                        self.trace(f"Wrote {len(to_write)} bytes in {dt:0.2f} ms: {to_write}")
                    else:
                        self.debug(f"Wrote {len(to_write)} bytes in {dt:0.2f} ms")

                    # TODO why wait here? is this needed?
                    wait = 2. * max(0.050, len(to_write) * 8. / self.bitrate)
                    self.closed_latch.join(timeout=wait)
                except serial.SerialTimeoutException:
                    self.unsent.append(to_write)
                    self.metrics.serial_unsent_queue_size(len(self.unsent))
                    t = next(self.timeout_backoff)
                    self.ser.write_timeout = t
                    t1 = time.time()
                    dt = (t1 - t0) * 1000
                    self.error(f"Failed to write bytes to serial device after {dt:0.2f} ms. "
                               f"Serial timed out. Increasing timeout to {t} seconds.")
                    self.metrics.write_timeout()
                except serial.SerialException:
                    t1 = time.time()
                    dt = (t1 - t0) * 1000
                    self.error(f"Failed to write bytes to serial device after {dt:0.2f} ms.")
                    self.metrics.write_error()
                    self.error_event.set()


class SerialDevice(CloseableThreadLoop, LoggingMixin):
    def __init__(self,
                 protocol: IOProtocol,
                 port_id: int,
                 device_name: str,
                 speed: int,
                 read_timeout_s: float,
                 write_timeout_s: float,
                 read_limit: int,
                 bitrate: int,
                 max_buffer: int):
        LoggingMixin.__init__(self, extra={"port": port_id})
        CloseableThreadLoop.__init__(self, f"SerialDevice(port={port_id}, device={device_name})")
        self.metrics = SerialMetrics(port_id)
        self._device_name = device_name
        self._protocol = protocol
        self._ser = serial.Serial(port=None, baudrate=speed, timeout=read_timeout_s, write_timeout=write_timeout_s)
        self._ser.port = device_name
        self._closed_latch = CountDownLatch(2)
        self._open_event = threading.Event()
        self._error_event = threading.Event()
        self._open_backoff = BackoffGenerator(0.100, 1.2, 5.000)
        # Submit the reader and writer threads first, so they will be shutdown first
        self._read_thread = SerialReadLoop(
            port_id,
            read_limit,
            f"SerialReadLoop(port={port_id}, device={self._ser.name})",
            self._ser,
            self._protocol,
            self._open_event,
            self._error_event,
            self._closed_latch
        )
        self._write_thread = SerialWriteLoop(
            port_id,
            max_buffer, f"SerialWriteLoop(port={port_id}, device={self._ser.name})",
            self._ser,
            self._protocol,
            self._open_event,
            self._error_event,
            self._closed_latch,
            bitrate=bitrate
        )

    def start_threads(self, scheduler: Scheduler):
        scheduler.submit(self._read_thread)
        scheduler.submit(self._write_thread)
        scheduler.submit(self)

    def close(self) -> None:
        # Stop this loop from re-opening the port
        super().close()

        # Signal to reader and writer that the port is closed
        self._open_event.clear()

        # Wait for them to finish
        self._closed_latch.join()

        # Close the port
        unlock_serial(self._ser)
        self._ser.close()

    def iter_loop(self):
        """Try to keep the serial port alive"""
        if self._error_event.is_set():
            self.warning("Had a serial error, attempting to reconnect")
            self._open_event.clear()
            self._error_event.clear()
            unlock_serial(self._ser)
            self._ser.close()
            sleep(next(self._open_backoff))
            return

        if not self._ser.is_open:
            self.info(f"Opening serial port {self._device_name}")
            try:
                self._ser.open()
                lock_serial(self._ser)
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
                self.info(f"Opened serial port {self._device_name}")
                # TODO info metrics about serial port
                self._open_event.set()
                self._open_backoff.reset()
            except serial.SerialException as err:
                t = next(self._open_backoff)
                self.warning(f"Failed to open serial port {self._device_name} with {err}. Trying again in {t:0.3f}s")
                self.metrics.open_error()
                sleep(t)
            except BlockingIOError:
                self._ser.close()
                t = next(self._open_backoff)
                self.warning(f"Failed to open serial port {self._device_name}. The resource is busy. Trying again in {t:0.3f}s")
                self.metrics.open_error()
                sleep(t)
        else:
            self.metrics.serial_buffer_waiting(self._ser.in_waiting, self._ser.out_waiting)
            sleep(0.5)
