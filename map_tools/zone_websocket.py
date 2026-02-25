#!/usr/bin/env python3

from __future__ import annotations

import asyncio
import json
import queue
import threading
from typing import Any

try:
    import websockets
except ImportError:  # pragma: no cover
    websockets = None


class ZoneWebSocketBridge:
    """Threaded websocket bridge with thread-safe command/event queues."""

    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._commands: queue.Queue[dict[str, Any]] = queue.Queue()
        self._events: queue.Queue[dict[str, Any]] = queue.Queue()

        self._latest_state_lock = threading.Lock()
        self._latest_state: dict[str, Any] | None = None

        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()

    @property
    def available(self) -> bool:
        return websockets is not None

    def start(self) -> bool:
        if not self.available:
            return False
        if self._thread is not None:
            return True

        self._stop_event.clear()
        self._ready_event.clear()
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()
        self._ready_event.wait(timeout=2.0)
        return self._ready_event.is_set()

    def stop(self) -> None:
        self._stop_event.set()
        if self._loop is not None:
            self._loop.call_soon_threadsafe(lambda: None)
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._loop = None

    def pop_command(self) -> dict[str, Any] | None:
        try:
            return self._commands.get_nowait()
        except queue.Empty:
            return None

    def push_event(self, event: dict[str, Any]) -> None:
        self._events.put(event)
        if self._loop is not None:
            self._loop.call_soon_threadsafe(lambda: None)

    def set_latest_state(self, state: dict[str, Any]) -> None:
        with self._latest_state_lock:
            self._latest_state = state

    def _thread_main(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())
        self._loop.close()

    async def _serve(self) -> None:
        if websockets is None:  # pragma: no cover
            return

        clients: set[Any] = set()

        async def publish_loop() -> None:
            while not self._stop_event.is_set():
                await asyncio.sleep(0.05)
                while True:
                    try:
                        event = self._events.get_nowait()
                    except queue.Empty:
                        break
                    text = json.dumps(event, separators=(",", ":"))
                    for client in list(clients):
                        try:
                            await client.send(text)
                        except Exception:
                            clients.discard(client)

        async def handler(websocket: Any, *_args: Any) -> None:
            clients.add(websocket)
            hello = {"op": "hello", "server": "map_tools.zone_server", "version": 1}
            await websocket.send(json.dumps(hello, separators=(",", ":")))

            with self._latest_state_lock:
                current_state = self._latest_state
            if current_state is not None:
                await websocket.send(
                    json.dumps({"op": "zones_state", "document": current_state}, separators=(",", ":"))
                )

            try:
                async for raw_msg in websocket:
                    await self._handle_client_message(websocket, raw_msg)
            finally:
                clients.discard(websocket)

        try:
            async with websockets.serve(handler, self._host, self._port):
                self._ready_event.set()
                publisher = asyncio.create_task(publish_loop())
                while not self._stop_event.is_set():
                    await asyncio.sleep(0.1)
                publisher.cancel()
                try:
                    await publisher
                except asyncio.CancelledError:
                    pass
        finally:
            self._ready_event.set()

    async def _handle_client_message(self, websocket: Any, raw_msg: str) -> None:
        try:
            msg = json.loads(raw_msg)
        except json.JSONDecodeError as exc:
            await websocket.send(
                json.dumps(
                    {"op": "error", "message": f"invalid json: {exc}"},
                    separators=(",", ":"),
                )
            )
            return

        if not isinstance(msg, dict):
            await websocket.send(json.dumps({"op": "error", "message": "message must be object"}))
            return

        op = msg.get("op")
        if not isinstance(op, str):
            await websocket.send(json.dumps({"op": "error", "message": "missing op"}))
            return

        self._commands.put(msg)
        ack: dict[str, Any] = {"op": "accepted", "request": op}
        if "id" in msg:
            ack["id"] = msg["id"]
        await websocket.send(json.dumps(ack, separators=(",", ":")))
