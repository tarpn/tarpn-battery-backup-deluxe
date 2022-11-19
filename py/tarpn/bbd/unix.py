import asyncio
import logging
from asyncio import AbstractEventLoop
from asyncio.base_events import Server
from functools import partial
from threading import Thread
from typing import Optional, Any, Callable

from tarpn.logging import LoggingMixin
from tarpn.thread.scheduler import CloseableThread


class AgentSocketProtocol(asyncio.Protocol, LoggingMixin):
    def __init__(self, callback: Callable[[str], None]):
        LoggingMixin.__init__(self, logger=logging.getLogger("unix"))
        self.transport = None
        self.callback = callback

    def data_received(self, data: bytes) -> None:
        cmd = data.decode("ascii").strip()
        self.debug(f"Got {cmd} over unix socket")
        self.callback(cmd)

    def connection_made(self, transport: asyncio.Transport) -> None:
        self.transport = transport

    def connection_lost(self, exc: Optional[Exception]) -> None:
        self.transport = None

    def send_if_connected(self, data: Any):
        if isinstance(data, bytes):
            pass
        elif isinstance(data, str):
            data = data.encode("ascii")
        else:
            data = str(data).encode("ascii")

        if self.transport:
            self.transport.write(data + b"\r\n")


def dummy_factory(obj):
    return obj


class UnixServerThread(CloseableThread):
    def __init__(self, protocol: AgentSocketProtocol):
        Thread.__init__(self)
        self.protocol = protocol
        self.server: Optional[Server] = None
        self._loop: Optional[AbstractEventLoop] = None

    def run(self):
        self._loop = asyncio.new_event_loop()
        self._loop.run_until_complete(self._create_server())
        self._loop.close()

    def close(self):
        if self.server is not None and self.server.is_serving():
            self.server.close()

    def send(self, b: bytes):
        self.protocol.send_if_connected(b)

    async def _create_server(self):
        loop = asyncio.get_event_loop()
        self.server = await loop.create_unix_server(partial(dummy_factory, self.protocol), "/tmp/tarpn-bbd-agent.sock")
        await self.server.start_serving()
        while True:
            if self.server.is_serving():
                await asyncio.sleep(1)
            else:
                break