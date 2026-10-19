import json
import socket
import threading
import time

import websocket

import config as cfg

# websocket.enableTrace(True)   # aktifkan untuk melihat detail jabat tangan saat debug


class RobotClient:
    SEND_HZ = cfg.SEND_HZ
    CONNECT_TIMEOUT = cfg.CONNECT_TIMEOUT
    RETRY_INTERVAL = cfg.RETRY_INTERVAL

    def __init__(self, esp32_ip=cfg.ESP32_IP, port=cfg.ESP32_PORT):
        self.ip = esp32_ip
        self.url = f"ws://{esp32_ip}:{port}/"
        self.ws = None
        self.connected = False

        self._cmd = (0, 0)
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ---------- koneksi ----------
    def _connect(self):
        try:
            self.ws = websocket.create_connection(
                self.url,
                timeout=self.CONNECT_TIMEOUT,
                sockopt=((socket.IPPROTO_TCP, socket.TCP_NODELAY, 1),),
                http_no_proxy=[self.ip],
            )
            self.ws.settimeout(1)
            self.connected = True
            print(f"[RobotClient] Terhubung ke {self.url}")
        except Exception as e:
            self.connected = False
            self.ws = None
            print(f"[RobotClient] Gagal konek ke ESP32 ({self.url}): {e}")

    def _drop(self):
        self.connected = False
        if self.ws:
            try:
                self.ws.close()
            except Exception:
                pass
        self.ws = None

    # ---------- thread pengirim ----------
    def _run(self):
        period = 1.0 / self.SEND_HZ
        while self._running:
            if not self.connected:
                self._connect()
                if not self.connected:
                    time.sleep(self.RETRY_INTERVAL)
                    continue

            with self._lock:
                left, right = self._cmd
            try:
                self.ws.send(json.dumps({"left": left, "right": right}))
            except Exception as e:
                print(f"[RobotClient] Error kirim perintah: {e}")
                self._drop()
                continue
            time.sleep(period)

    # ---------- API untuk main.py ----------
    def send_motor(self, left, right):
        """Set kecepatan motor kiri & kanan (-255..255). Dikirim oleh thread."""
        with self._lock:
            self._cmd = (int(left), int(right))

    def stop(self):
        self.send_motor(0, 0)

    def close(self):
        self.stop()
        time.sleep(cfg.STOP_FLUSH_DELAY)          # beri waktu thread mengirim perintah stop
        self._running = False
        self._thread.join(timeout=1)
        self._drop()