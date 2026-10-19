import cv2
import numpy as np
from scipy.interpolate import splprep, splev

import config as cfg


class PathDrawer:
    def __init__(self, window_name=cfg.WINDOW_NAME):
        self.window_name = window_name
        self.drawing = False
        self.raw_points = []
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.raw_points = [(x, y)]
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.raw_points.append((x, y))
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False

    def draw_overlay(self, frame):
        """Gambar garis mentah (kuning) di atas frame, buat feedback visual saat user menggambar."""
        pts = self.raw_points
        for i in range(1, len(pts)):
            cv2.line(frame, pts[i - 1], pts[i], (0, 255, 255), 2)
        return frame

    def get_smoothed_path(self, spacing_px=cfg.PATH_SPACING_PX, smoothing=cfg.PATH_SMOOTHING):
        """
        Ubah titik mentah hasil freehand jadi path halus & seragam.

        Args:
            spacing_px: jarak antar titik hasil resample (dalam pixel)
            smoothing: faktor smoothing spline (0 = pas persis ke titik mentah,
                       makin besar makin halus tapi makin "memotong" kurva asli)

        Returns:
            Nx2 numpy array (x, y) titik path yang sudah halus & seragam,
            atau None kalau path terlalu pendek / belum digambar.
        """
        if len(self.raw_points) < cfg.PATH_MIN_RAW_POINTS:
            return None

        pts = np.array(self.raw_points, dtype=float)

        # buang titik duplikat/terlalu rapat (spline butuh titik unik)
        deduped = [pts[0]]
        for p in pts[1:]:
            if np.linalg.norm(p - deduped[-1]) > 1e-3:
                deduped.append(p)
        pts = np.array(deduped)

        if len(pts) < 4:
            return pts

        try:
            tck, _ = splprep([pts[:, 0], pts[:, 1]], s=smoothing)
        except Exception:
            # fallback: kalau spline gagal (misal titik kolinear semua), pakai titik asli
            return pts

        path_length = np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))
        n_samples = max(int(path_length / spacing_px), cfg.PATH_MIN_SAMPLES)

        u_fine = np.linspace(0, 1, n_samples)
        x_fine, y_fine = splev(u_fine, tck)
        return np.stack([x_fine, y_fine], axis=1)

    def draw_smoothed_path(self, frame, smoothed_path):
        """Gambar path yang sudah dihaluskan (hijau tebal), buat konfirmasi visual sebelum robot jalan."""
        if smoothed_path is None:
            return frame
        pts = smoothed_path.astype(int)
        for i in range(1, len(pts)):
            cv2.line(frame, tuple(pts[i - 1]), tuple(pts[i]), (0, 200, 0), 3)
        return frame

    def reset(self):
        self.raw_points = []
