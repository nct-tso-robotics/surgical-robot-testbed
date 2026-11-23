"""WebSocket command delivery independent of ROS and physical feedback."""

from __future__ import annotations

import asyncio
import threading

import websockets


class GripperBridge:
    """Send the latest desired command to one browser, replaying on reconnect."""

    def __init__(self, host: str = "127.0.0.1", port: int = 6789) -> None:
        self.host = host
        self.port = port
        self.desired = False
        self.delivered: bool | None = None
        self._lock = threading.Lock()
        self._loop = None
        self._changed = None
        self._stop = None
        self._client = None
        self._ready = threading.Event()
        self._error = None
        self._thread = threading.Thread(
            target=self._run, name="gripper-websocket", daemon=True
        )

    def start(self) -> None:
        self._thread.start()
        if not self._ready.wait(timeout=5) or self._error is not None:
            raise RuntimeError(f"Gripper WebSocket failed to start: {self._error}")

    def command(self, opened: bool) -> None:
        with self._lock:
            self.desired = bool(opened)
        if self._loop is not None and self._changed is not None:
            self._loop.call_soon_threadsafe(self._changed.set)

    def delivery_state(self) -> bool | None:
        with self._lock:
            return self.delivered

    def requested_state(self) -> bool:
        """Return command intent, independent of browser connection or feedback."""
        with self._lock:
            return self.desired

    async def _connection(self, client) -> None:
        if self._client is not None:
            await client.close(
                code=1013, reason="One gripper client is already connected"
            )
            return
        self._client = client
        self._changed.set()
        try:
            await client.wait_closed()
        finally:
            self._client = None
            with self._lock:
                self.delivered = None

    async def _worker(self) -> None:
        while True:
            await self._changed.wait()
            self._changed.clear()
            client = self._client
            with self._lock:
                desired = self.desired
            if client is None:
                continue
            try:
                await client.send("1" if desired else "0")
                with self._lock:
                    if self._client is client:
                        self.delivered = desired
            except websockets.ConnectionClosed:
                with self._lock:
                    self.delivered = None

    async def _serve(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._stop = asyncio.Event()
        self._changed = asyncio.Event()
        async with websockets.serve(
            self._connection, self.host, self.port, close_timeout=1
        ):
            worker = asyncio.create_task(self._worker())
            self._ready.set()
            await self._stop.wait()
            worker.cancel()
            await asyncio.gather(worker, return_exceptions=True)

    def _run(self) -> None:
        try:
            asyncio.run(self._serve())
        except (OSError, RuntimeError) as error:
            self._error = error
            self._ready.set()
        finally:
            with self._lock:
                self.delivered = None

    def close(self) -> None:
        if (
            self._loop is not None
            and self._stop is not None
            and self._thread.is_alive()
        ):
            self._loop.call_soon_threadsafe(self._stop.set)
            self._thread.join(timeout=5)
