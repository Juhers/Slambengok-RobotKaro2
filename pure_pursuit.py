import time
import numpy as np
import config as cfg
from fuzzy_pid_controller import FuzzyPIDController


class PurePursuit:
    def __init__(self, path,
                 lookahead=cfg.LOOKAHEAD_PX,
                 base_speed=cfg.BASE_SPEED,
                 max_speed=cfg.MAX_SPEED,
                 turn_gain=cfg.TURN_GAIN,
                 steer_boost=cfg.STEER_BOOST,
                 turn_limit=cfg.TURN_LIMIT,
                 preview_dist=cfg.PREVIEW_DIST,
                 curve_full_deg=cfg.CURVE_FULL_DEG,
                 corner_speed_ratio=cfg.CORNER_SPEED_RATIO,
                 brake_rate=cfg.BRAKE_RATE,
                 accel_rate=cfg.ACCEL_RATE,
                 turn_smooth=cfg.TURN_SMOOTH,
                 min_pwm=cfg.MIN_PWM):
        self.path = np.array(path, dtype=float)
        self.lookahead = lookahead
        self.base_speed = base_speed
        self.max_speed = max_speed
        self.turn_gain = turn_gain
        self.steer_boost = steer_boost
        self.turn_limit = turn_limit
        self.preview_dist = preview_dist
        self.curve_full_deg = curve_full_deg
        self.corner_speed_ratio = corner_speed_ratio
        self.brake_rate = brake_rate
        self.accel_rate = accel_rate
        self.turn_smooth = turn_smooth
        self.min_pwm = min_pwm

        self.last_idx = 0
        self.controller = FuzzyPIDController()

        self._last_time = None
        self._cur_speed = None
        self._turn_prev = 0.0
        self.last_slow_factor = 0.0   # 0 = lurus, 1 = paling lambat (untuk debug/UI)
        self.last_heading_error = 0.0 # untuk debug/UI

    # ---------- path ----------
    def _find_lookahead_point(self, pos):
        pos = np.array(pos, dtype=float)
        remaining = self.path[self.last_idx:]
        if len(remaining) == 0:
            return self.path[-1]

        dists = np.linalg.norm(remaining - pos, axis=1)
        idx_candidates = np.where(dists >= self.lookahead)[0]

        if len(idx_candidates) > 0:
            found_idx = self.last_idx + idx_candidates[0]
            self.last_idx = found_idx
            return self.path[found_idx]

        self.last_idx = len(self.path) - 1
        return self.path[-1]

    def is_finished(self, pos, tolerance=cfg.FINISH_TOLERANCE_PX):
        near_end = np.linalg.norm(self.path[-1] - np.array(pos)) < tolerance
        reached_last_index = self.last_idx >= len(self.path) - 2
        return near_end and reached_last_index

    def _upcoming_curvature(self):
        """
        Intip path dari last_idx sejauh preview_dist, kembalikan 0..1:
        0 = path di depan lurus, 1 = ada tikungan tajam (>= curve_full_deg).
        Juga memperhitungkan ujung path (melambat saat mendekati finish).
        """
        i0 = self.last_idx
        n = len(self.path)
        if n - i0 < 4:
            return 1.0  # hampir sampai ujung -> pelan

        seg = np.linalg.norm(np.diff(self.path[i0:], axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        end_idx = i0 + int(np.searchsorted(cum, self.preview_dist))
        reaches_end = end_idx >= n - 1
        end_idx = min(end_idx, n - 1)

        k = cfg.CURVE_DIR_STEP  # selang titik untuk hitung arah (meredam jitter)

        def direction(i):
            j = min(i + k, n - 1)
            d = self.path[j] - self.path[i]
            return np.degrees(np.arctan2(d[1], d[0]))

        a0 = direction(i0)
        max_dev = 0.0
        for i in range(i0, max(end_idx - 1, i0 + 1)):
            dev = abs((direction(i) - a0 + 180) % 360 - 180)
            max_dev = max(max_dev, dev)

        curve = float(np.clip(max_dev / self.curve_full_deg, 0.0, 1.0))
        if reaches_end:
            remain = cum[-1] - cum[0]
            curve = max(curve, float(np.clip(1.0 - remain / self.preview_dist, 0.0, 1.0)) * 0.8)
        return curve

    # ---------- utilitas ----------
    def _snap(self, v):
        """PWM di bawah min_pwm hanya membuat motor berdengung tanpa berputar.
        Nilai itu dibulatkan ke 0 (roda berhenti) supaya roda dalam benar-benar
        bisa melambat dan belokan tetap tajam."""
        if abs(v) < self.min_pwm:
            return 0
        return int(v)

    @staticmethod
    def predict_pose(pos, heading_deg, speed_px_s, omega_deg_s, delay=cfg.PREDICT_DELAY):
        """
        Kompensasi delay nirkabel: perkirakan posisi/heading robot `delay` detik
        ke depan. Panggil SEBELUM compute():
            pos, heading = PurePursuit.predict_pose(pos, heading, v, w, 0.15)
        speed_px_s  : kecepatan robot (pixel/detik), boleh estimasi kasar
        omega_deg_s : kecepatan putar (derajat/detik), boleh 0
        """
        h = heading_deg + omega_deg_s * delay
        rad = np.radians(h)
        p = np.array(pos, dtype=float) + speed_px_s * delay * np.array([np.cos(rad), np.sin(rad)])
        return p, (h + 180) % 360 - 180

    # ---------- kontrol ----------
    def compute(self, pos, heading_deg):
        """
        Returns:
            (left_speed, right_speed, target_point)
        """
        pos = np.array(pos, dtype=float)
        target = self._find_lookahead_point(pos)

        dx, dy = target[0] - pos[0], target[1] - pos[1]
        target_angle = np.degrees(np.arctan2(dy, dx))
        heading_error = (target_angle - heading_deg + 180) % 360 - 180
        self.last_heading_error = heading_error
        abs_err = abs(heading_error)

        now = time.time()
        if self._last_time is None:
            dt = cfg.DEFAULT_DT
        else:
            dt = max(now - self._last_time, 1e-3)
        self._last_time = now

        # ---- output belok: fuzzy PID -> filter ringan -> boost ----
        turn = self.controller.compute(heading_error, dt=dt)
        turn = self.turn_smooth * turn + (1 - self.turn_smooth) * self._turn_prev
        self._turn_prev = turn
        turn = float(np.clip(turn * self.steer_boost, -self.turn_limit, self.turn_limit))

        # ---- kecepatan: antisipatif (kelengkungan path) + reaktif (error) ----
        curve_factor = self._upcoming_curvature()
        error_factor = float(np.clip(abs_err / (self.turn_gain * 1.5), 0.0, 1.0))
        slow = max(curve_factor, error_factor)
        self.last_slow_factor = slow

        target_speed = self.base_speed * (1.0 - slow * (1.0 - self.corner_speed_ratio))

        if self._cur_speed is None:
            self._cur_speed = target_speed
        elif target_speed < self._cur_speed:
            self._cur_speed = max(target_speed, self._cur_speed - self.brake_rate * self.base_speed * dt)
        else:
            self._cur_speed = min(target_speed, self._cur_speed + self.accel_rate * self.base_speed * dt)
        speed = self._cur_speed
        speed = max(speed, self.min_pwm)   # kecepatan dasar tidak boleh di zona dengung

        # ---- mixing: roda dalam boleh berhenti, bahkan mundur (turn > 1) ----
        left = speed * (1.0 - turn)
        right = speed * (1.0 + turn)

        left = int(np.clip(left, -self.max_speed, self.max_speed))
        right = int(np.clip(right, -self.max_speed, self.max_speed))

        left = self._snap(left)
        right = self._snap(right)

        return left, right, target

    def reset_controller(self):
        """Panggil saat mulai path baru / marker kembali terdeteksi."""
        self.controller.reset()
        self._last_time = None
        self._cur_speed = None
        self._turn_prev = 0.0