import cv2
import numpy as np

import config as cfg

_CANDIDATES = cfg.ARUCO_CANDIDATES


class QRLocalizer:
    def __init__(self, dict_type=None, marker_id=cfg.MARKER_ID):
        """
        Args:
            dict_type: dictionary ArUco (mis. cv2.aruco.DICT_4X4_100).
                       Kalau None, dicoba otomatis dari daftar kandidat.
            marker_id: ID marker yang dipakai. Kalau None, ambil marker
                       pertama yang terdeteksi.
        """
        self.marker_id = marker_id
        if dict_type is None and cfg.ARUCO_DICT:
            dict_type = getattr(cv2.aruco, cfg.ARUCO_DICT)
        self._new_api = hasattr(cv2.aruco, "ArucoDetector")

        if self._new_api:
            self.params = cv2.aruco.DetectorParameters()
        else:
            self.params = cv2.aruco.DetectorParameters_create()
        # sub-pixel supaya sudut (dan heading) lebih stabil
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        if dict_type is not None:
            self._locked = True
            self._detectors = [(dict_type, self._make(dict_type))]
        else:
            self._locked = False
            self._detectors = []
            for name in _CANDIDATES:
                d = getattr(cv2.aruco, name, None)
                if d is not None:
                    self._detectors.append((d, self._make(d)))

    def _make(self, dict_type):
        if self._new_api:
            dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
            return cv2.aruco.ArucoDetector(dictionary, self.params)
        return cv2.aruco.Dictionary_get(dict_type)

    def _run(self, det, frame):
        if self._new_api:
            corners, ids, _ = det.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(frame, det, parameters=self.params)
        return corners, ids

    def _pick(self, corners, ids):
        """Pilih index marker target, atau None."""
        if ids is None or len(ids) == 0:
            return None
        ids = ids.flatten()
        if self.marker_id is None:
            return 0
        idx = np.where(ids == self.marker_id)[0]
        return int(idx[0]) if len(idx) else None

    def detect(self, frame):
        """
        Deteksi marker pada frame.

        Returns:
            None kalau tidak terdeteksi, atau tuple:
            (x, y, heading_deg, corner_points)
            - x, y: posisi tengah marker dalam pixel
            - heading_deg: sudut hadap robot dalam derajat
                           (koordinat pixel, y ke bawah positif)
            - corner_points: array 4x2 posisi sudut marker (untuk digambar/debug)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame

        corners, i = None, None
        for dict_type, det in self._detectors:
            c, ids = self._run(det, gray)
            i = self._pick(c, ids)
            if i is not None:
                corners = c
                if not self._locked:
                    # kunci ke dictionary yang berhasil
                    self._detectors = [(dict_type, det)]
                    self._locked = True
                    print(f"[QRLocalizer] Dictionary terkunci (kode {dict_type}), "
                          f"ID {ids.flatten()[i]}")
                break

        if corners is None:
            return None

        points = corners[i].reshape(-1, 2)
        if points.shape[0] != 4:
            return None

        cx, cy = points.mean(axis=0)

        # heading dari corner 0 (kiri-atas) ke corner 1 (kanan-atas),
        # dalam koordinat pixel (y ke bawah positif), konsisten dengan
        # perhitungan target_angle di pure_pursuit.py
        p0, p1 = points[0], points[1]
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        heading = np.degrees(np.arctan2(dy, dx))

        return (cx, cy, heading, points)