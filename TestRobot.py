"""
Kontrol:
  W / S     : maju / mundur
  A / D     : belok kiri / kanan
  Spasi     : berhenti
  Slider    : atur kecepatan (0..255)
"""

import tkinter as tk
from tkinter import ttk

from robot_client import RobotClient

DEFAULT_IP = "192.168.4.1"
UPDATE_MS = 50 
RELEASE_DELAY_MS = 40


def clamp(v, lo=-255, hi=255):
    return max(lo, min(hi, int(v)))


class App:
    def __init__(self, root):
        self.root = root
        self.robot = None
        self.pressed = set()
        self._release_jobs = {}

        root.title("Kontrol Robot WASD")
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        # --- koneksi ---
        top = ttk.Frame(root, padding=10)
        top.pack(fill="x")
        ttk.Label(top, text="IP ESP32:").pack(side="left")
        self.ip_var = tk.StringVar(value=DEFAULT_IP)
        ttk.Entry(top, textvariable=self.ip_var, width=16).pack(side="left", padx=5)
        self.btn_conn = ttk.Button(top, text="Hubungkan", command=self.toggle_connect)
        self.btn_conn.pack(side="left")
        self.status = tk.Label(top, text="● Terputus", fg="red")
        self.status.pack(side="left", padx=10)

        # --- kecepatan ---
        sp = ttk.Frame(root, padding=(10, 0))
        sp.pack(fill="x")
        ttk.Label(sp, text="Kecepatan:").pack(side="left")
        self.speed = tk.IntVar(value=150)
        ttk.Scale(sp, from_=0, to=255, variable=self.speed, orient="horizontal",
                  length=220, command=lambda _v: self.speed_lbl.config(
                      text=str(self.speed.get()))).pack(side="left", padx=5)
        self.speed_lbl = ttk.Label(sp, text="150", width=4)
        self.speed_lbl.pack(side="left")

        # --- tombol WASD di layar (juga bisa diklik/ditahan) ---
        pad = ttk.Frame(root, padding=10)
        pad.pack()
        self.keys_btn = {}
        layout = {"w": (0, 1), "a": (1, 0), "s": (1, 1), "d": (1, 2)}
        for k, (r, c) in layout.items():
            b = tk.Button(pad, text=k.upper(), width=5, height=2, font=("Arial", 14, "bold"))
            b.grid(row=r, column=c, padx=3, pady=3)
            b.bind("<ButtonPress-1>", lambda e, kk=k: self.press(kk))
            b.bind("<ButtonRelease-1>", lambda e, kk=k: self.release_now(kk))
            self.keys_btn[k] = b
        tk.Button(pad, text="STOP (Spasi)", bg="#d9534f", fg="white",
                  command=self.emergency_stop).grid(row=2, column=0, columnspan=3,
                                                    sticky="we", pady=(8, 0))

        self.info = ttk.Label(root, text="L: 0   R: 0", padding=10)
        self.info.pack()

        # --- keyboard ---
        root.bind("<KeyPress>", self.on_key_press)
        root.bind("<KeyRelease>", self.on_key_release)

        self.loop()

    # ---------- koneksi ----------
    def toggle_connect(self):
        if self.robot is None:
            self.robot = RobotClient(self.ip_var.get().strip())
            self.btn_conn.config(text="Putuskan")
        else:
            self.robot.close()
            self.robot = None
            self.btn_conn.config(text="Hubungkan")
            self.status.config(text="● Terputus", fg="red")

    # ---------- input ----------
    def on_key_press(self, e):
        k = e.keysym.lower()
        if k == "space":
            self.emergency_stop()
        elif k in "wasd" and len(k) == 1:
            self.press(k)

    def on_key_release(self, e):
        k = e.keysym.lower()
        if k in "wasd" and len(k) == 1:
            self._release_jobs[k] = self.root.after(RELEASE_DELAY_MS,
                                                    lambda kk=k: self.release_now(kk))

    def press(self, k):
        job = self._release_jobs.pop(k, None)
        if job:
            self.root.after_cancel(job)
        self.pressed.add(k)
        self.keys_btn[k].config(relief="sunken", bg="#9fd39f")

    def release_now(self, k):
        self._release_jobs.pop(k, None)
        self.pressed.discard(k)
        self.keys_btn[k].config(relief="raised", bg="SystemButtonFace"
                                if self.root.tk.call("tk", "windowingsystem") != "x11"
                                else "#d9d9d9")

    def emergency_stop(self):
        for k in list(self.pressed):
            self.release_now(k)
        if self.robot:
            self.robot.stop()

    # ---------- hitung & kirim ----------
    def compute(self):
        s = self.speed.get()
        f = ("w" in self.pressed) - ("s" in self.pressed)   # maju/mundur
        t = ("a" in self.pressed) - ("d" in self.pressed)   # kanan/kiri

        if f == 0:
            return clamp(t * s), clamp(-t * s)
        left = f * s + t * s * 0.5
        right = f * s - t * s * 0.5
        return clamp(left), clamp(right)

    def loop(self):
        left, right = self.compute()
        if self.robot:
            self.robot.send_motor(left, right)
            if self.robot.connected:
                self.status.config(text="● Terhubung", fg="green")
            else:
                self.status.config(text="● Menyambung...", fg="orange")
        self.info.config(text=f"L: {left}   R: {right}")
        self.root.after(UPDATE_MS, self.loop)

    def on_close(self):
        if self.robot:
            self.robot.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()