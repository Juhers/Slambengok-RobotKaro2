"""
config.py -- SEMUA parameter yang perlu di-tuning ada di file ini.
Modul lain (main, pure_pursuit, fuzzy_pid_controller, dst.) hanya membaca
nilai dari sini, jadi tidak perlu mengedit file lain saat tuning.
Semua satuan jarak = pixel, sudut = derajat, waktu = detik.
"""

# =====================================================================
# 1. KONEKSI ESP32 (robot_client.py)
# =====================================================================
ESP32_IP = "192.168.4.1"
ESP32_PORT = 81
SEND_HZ = 25               # frekuensi kirim perintah ke ESP32
CONNECT_TIMEOUT = 5        # detik
RETRY_INTERVAL = 2.0       # detik antar percobaan konek ulang
STOP_FLUSH_DELAY = 0.15    # jeda saat close() agar perintah stop terkirim

# =====================================================================
# 2. KAMERA & TAMPILAN (main.py)
# =====================================================================
CAM_INDEX = 0
WINDOW_NAME = "Robot Path Control"

# =====================================================================
# 3. DETEKSI MARKER (qr_localization.py)
# =====================================================================
MARKER_ID = 67             # None = pakai marker pertama yang terdeteksi
ARUCO_DICT = None          # None = coba otomatis dari ARUCO_CANDIDATES
                           # atau isi nama, mis. "DICT_4X4_100"
ARUCO_CANDIDATES = [
    "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
    "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
    "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
    "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000",
    "DICT_ARUCO_ORIGINAL",
]

# =====================================================================
# 4. PATH (path_planner.py)
# =====================================================================
PATH_SPACING_PX = 8        # jarak antar titik hasil resample
PATH_SMOOTHING = 5.0       # makin besar makin halus tapi menyimpang
PATH_MIN_RAW_POINTS = 4    # minimal titik mentah agar spline dibuat
PATH_MIN_POINTS_TO_RUN = 4 # path hasil harus > (len > 3) agar robot jalan
PATH_MIN_SAMPLES = 10

# =====================================================================
# 5. PURE PURSUIT (pure_pursuit.py)
# =====================================================================
LOOKAHEAD_PX = 40          # makin kecil: ketat tapi rawan osilasi
BASE_SPEED = 180           # PWM saat lurus
MAX_SPEED = 210            # batas PWM roda luar
MIN_PWM = 150              # di bawah ini motor hanya berdengung -> jadi 0

TURN_GAIN = 30.0           # skala heading error (deg) untuk slow-down reaktif
STEER_BOOST = 1.0          # pengali output belok
TURN_LIMIT = 1.0           # batas |turn| (>1 = roda dalam boleh mundur)
TURN_SMOOTH = 1.7          # 1.0 = tanpa filter, makin kecil makin halus

PREVIEW_DIST = 50.0        # jarak intip tikungan di depan (px)
CURVE_FULL_DEG = 30.0      # deviasi arah yg dianggap "tikungan tajam"
CORNER_SPEED_RATIO = 0.85  # speed di tikungan = BASE_SPEED * rasio ini
BRAKE_RATE = 2.0           # laju pengereman (x BASE_SPEED per detik)
ACCEL_RATE = 0.8           # laju akselerasi (x BASE_SPEED per detik)
CURVE_DIR_STEP = 3         # selang titik utk hitung arah path (redam jitter)
FINISH_TOLERANCE_PX = 15   # jarak ke ujung path dianggap selesai
DEFAULT_DT = 1.0 / 30.0    # dt frame pertama

# Kompensasi delay nirkabel (predict_pose)
PREDICT_DELAY = 0.2

# =====================================================================
# 6. FUZZY PID (fuzzy_pid_controller.py)
# =====================================================================
FUZZY_ERROR_RANGE = 80.0   # |heading error| (deg) yang dianggap "Large";
                           # juga faktor normalisasi output ke -1..1
FUZZY_EC_RANGE = 60.0      # |laju error| (deg/s) yang dianggap "Large"

KP_RANGE = (0.2, 1.0)
KI_RANGE = (0.0, 0.4)
KD_RANGE = (0.0, 0.8)

INTEGRAL_LIMIT = 50.0      # anti-windup
EC_FILTER_ALPHA = 0.3      # low-pass derivative (kecil = lebih halus)
EC_CLIP_MULT = 3           # clip ec mentah ke +-(EC_RANGE * nilai ini)

# Bentuk fuzzy set S/M/L: (kiri, puncak, kanan) sebagai kelipatan RANGE
FUZZY_SET_SHAPES = {
    "S": (-0.2, 0.0, 0.45),
    "M": (0.0, 0.5, 1.0),
    "L": (0.55, 1.0, 1.5),
}

# Rule table Sugeno (0..1). Key = (kategori |error|, kategori |ec|)
TABLE_KP = {
    ("S", "S"): 0.2, ("S", "M"): 0.3, ("S", "L"): 0.4,
    ("M", "S"): 0.5, ("M", "M"): 0.6, ("M", "L"): 0.7,
    ("L", "S"): 0.8, ("L", "M"): 0.9, ("L", "L"): 1.0,
}
TABLE_KI = {
    ("S", "S"): 1.0, ("S", "M"): 0.6, ("S", "L"): 0.3,
    ("M", "S"): 0.5, ("M", "M"): 0.3, ("M", "L"): 0.1,
    ("L", "S"): 0.2, ("L", "M"): 0.1, ("L", "L"): 0.0,
}
TABLE_KD = {
    ("S", "S"): 0.3, ("S", "M"): 0.5, ("S", "L"): 0.7,
    ("M", "S"): 0.4, ("M", "M"): 0.6, ("M", "L"): 0.8,
    ("L", "S"): 0.2, ("L", "M"): 0.4, ("L", "L"): 0.5,
}