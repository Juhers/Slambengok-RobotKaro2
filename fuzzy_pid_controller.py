import numpy as np

import config as cfg

def _trimf(x, points):
    """Fungsi keanggotaan segitiga. points = (a, b, c): a=kiri(0), b=puncak(1), c=kanan(0)."""
    a, b, c = points
    if x <= a or x >= c:
        return 0.0
    if x == b:
        return 1.0
    if x < b:
        return (x - a) / (b - a)
    return (c - x) / (c - b)


class FuzzyPIDController:
    def __init__(self,
                 error_range=cfg.FUZZY_ERROR_RANGE, ec_range=cfg.FUZZY_EC_RANGE,
                 kp_range=cfg.KP_RANGE, ki_range=cfg.KI_RANGE, kd_range=cfg.KD_RANGE,
                 integral_limit=cfg.INTEGRAL_LIMIT, ec_filter_alpha=cfg.EC_FILTER_ALPHA):
        """
        Args:
            error_range: skala |heading_error| (derajat) yang dianggap
                         "Large" oleh fuzzy scheduler. Juga dipakai sebagai
                         faktor normalisasi output akhir ke rentang -1..1.
            ec_range: skala |laju perubahan error| (derajat/detik) yang
                       dianggap "Large".
            kp_range, ki_range, kd_range: (nilai_min, nilai_max) tiap gain
                       PID. Fuzzy scheduler memilih nilai gain di ANTARA
                       batas ini, bukan nilai tetap.
            integral_limit: batas anti-windup untuk komponen integral,
                             supaya integral tidak menumpuk tak terbatas
                             saat error besar dalam waktu lama.
            ec_filter_alpha: koefisien low-pass filter (0..1) untuk
                             komponen derivative (ec). Makin kecil, makin
                             halus/lambat filter meresponnya, tapi makin
                             tahan terhadap noise/jitter deteksi QR yang
                             bisa menyebabkan "derivative kick" (lonjakan
                             turn tiba-tiba & salah arah saat error
                             berubah cepat karena noise, bukan gerakan
                             asli robot).
        """
        self.error_range = error_range
        self.ec_range = ec_range
        self.kp_range = kp_range
        self.ki_range = ki_range
        self.kd_range = kd_range
        self.integral_limit = integral_limit
        self.ec_filter_alpha = ec_filter_alpha

        # Fuzzy set MAGNITUDE untuk |error| dan |ec| -- 3 kategori:
        # Small, Medium, Large. Dibuat dari nilai absolut (arah tidak
        # relevan buat menentukan seberapa "agresif" gain PID harus diset).
        self.mag_sets_e = {k: tuple(error_range * f for f in v)
                           for k, v in cfg.FUZZY_SET_SHAPES.items()}
        self.mag_sets_ec = {k: tuple(ec_range * f for f in v)
                            for k, v in cfg.FUZZY_SET_SHAPES.items()}

        # Rule table (Sugeno singleton 0..1) -- diatur di config.py
        self.table_kp = cfg.TABLE_KP
        self.table_ki = cfg.TABLE_KI
        self.table_kd = cfg.TABLE_KD

        self._integral = 0.0
        self._prev_error = None
        self._ec_filtered = 0.0

        # nilai gain terakhir yang dipakai, buat ditampilkan/debug di UI
        self.last_gains = (kp_range[0], ki_range[0], kd_range[0])

    def _fuzzify_magnitude(self, value, sets):
        return {label: _trimf(abs(value), pts) for label, pts in sets.items()}

    def _schedule_gain(self, mem_e, mem_ec, table, gain_range):
        """
        Sugeno-style weighted average: tiap kombinasi (kategori_e,
        kategori_ec) punya rule_strength = min(keanggotaan_e, keanggotaan_ec),
        lalu digabung dengan nilai singleton dari tabel jadi satu angka
        ternormalisasi (0..1), yang kemudian dipetakan ke rentang gain asli.
        """
        num, den = 0.0, 0.0
        for e_label, e_strength in mem_e.items():
            if e_strength <= 0:
                continue
            for ec_label, ec_strength in mem_ec.items():
                if ec_strength <= 0:
                    continue
                rule_strength = min(e_strength, ec_strength)
                singleton = table[(e_label, ec_label)]
                num += rule_strength * singleton
                den += rule_strength

        normalized = (num / den) if den > 1e-9 else 0.5  # fallback netral
        g_min, g_max = gain_range
        return g_min + normalized * (g_max - g_min)

    def reset(self):
        """Panggil tiap kali mulai path baru, supaya integral tidak
        membawa akumulasi error dari sesi path following sebelumnya."""
        self._integral = 0.0
        self._prev_error = None
        self._ec_filtered = 0.0

    def compute(self, error, dt=1.0):
        """
        Args:
            error: heading_error saat ini (derajat, sudah dinormalisasi
                    -180..180, dari pure_pursuit.py)
            dt: selang waktu sejak compute() terakhir dipanggil (detik).
                Kalau dipanggil tiap frame kamera, dt kurang lebih 1/fps.

        Returns:
            float, turn dalam rentang -1..1 -- drop-in compatible dengan
            FuzzyTurnController versi sebelumnya (dipakai sama persis di
            pure_pursuit.py).
        """
        if self._prev_error is None:
            raw_ec = 0.0
        else:
            raw_ec = (error - self._prev_error) / max(dt, 1e-6)
        self._prev_error = error

        # batasi ec mentah ke nilai wajar -- lapis keamanan tambahan kalau
        # ada 1 frame QR yang melesat/salah deteksi (posisi/heading
        # melompat drastis), supaya derivative tidak pernah benar-benar
        # ekstrem walau sebelum sempat difilter
        raw_ec = float(np.clip(raw_ec, -self.ec_range * cfg.EC_CLIP_MULT, self.ec_range * cfg.EC_CLIP_MULT))

        # low-pass filter komponen derivative, supaya lonjakan error
        # sesaat (noise/jitter deteksi QR) tidak menyebabkan "derivative
        # kick" -- turn melompat ke arah yang salah secara tiba-tiba
        self._ec_filtered = (self.ec_filter_alpha * raw_ec
                              + (1 - self.ec_filter_alpha) * self._ec_filtered)
        ec = self._ec_filtered

        # ---- Fuzzy gain scheduling: tentukan Kp, Ki, Kd saat ini ----
        mem_e = self._fuzzify_magnitude(error, self.mag_sets_e)
        mem_ec = self._fuzzify_magnitude(ec, self.mag_sets_ec)

        kp = self._schedule_gain(mem_e, mem_ec, self.table_kp, self.kp_range)
        ki = self._schedule_gain(mem_e, mem_ec, self.table_ki, self.ki_range)
        kd = self._schedule_gain(mem_e, mem_ec, self.table_kd, self.kd_range)
        self.last_gains = (kp, ki, kd)

        # ---- PID klasik, dengan gain yang baru saja dijadwalkan fuzzy ----
        self._integral += error * dt
        self._integral = float(np.clip(self._integral, -self.integral_limit, self.integral_limit))

        output = kp * error + ki * self._integral + kd * ec

        # normalisasi ke -1..1 (error_range dipakai sebagai skala referensi,
        # supaya konsisten dengan cara pure_pursuit.py memakai nilai turn)
        turn = output / self.error_range
        return float(np.clip(turn, -1.0, 1.0))
