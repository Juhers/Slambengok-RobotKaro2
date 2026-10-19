# Robot Virtual Path Follower Fuzzy-PID Hybrid Control

Sistem kontrol robot beroda diferensial (differential drive) yang mengikuti **jalur yang digambar langsung dengan mouse** di atas tampilan kamera. Posisi dan arah hadap robot dilacak secara real-time menggunakan **marker ArUco** lewat kamera yang dipasang di atas area kerja, lalu perintah motor dikirim ke **ESP32** melalui **WebSocket**.

<img width="1600" height="900" alt="MENDETEKSI ArUco" src="https://github.com/user-attachments/assets/ea899621-ea9e-458f-9973-4d76ba4c3df6" />

https://github.com/user-attachments/assets/13d531f7-f0ea-47b9-8805-614617504d05

## Daftar Isi

- [Fitur Utama](#fitur-utama)
- [Arsitektur Sistem](#arsitektur-sistem)
- [Struktur Proyek](#struktur-proyek)
- [Kebutuhan](#kebutuhan)
- [Instalasi](#instalasi)
- [Upload Firmware ESP32](#upload-firmware-esp32)
- [Uji Koneksi Robot (TestRobot.py)](#uji-koneksi-robot-testrobotpy)
- [Cara Menjalankan](#cara-menjalankan)
- [Protokol Komunikasi ESP32](#protokol-komunikasi-esp32)
- [Cara Kerja Detail](#cara-kerja-detail)
- [Panduan Tuning](#panduan-tuning-configpy)
- [Troubleshooting](#troubleshooting)
- [Catatan Platform](#catatan-platform)
- [Keselamatan](#keselamatan)
- [Pengembangan Lanjutan](#pengembangan-lanjutan)
- [Kontribusi](#kontribusi)
- [Lisensi](#lisensi)
- [Author](#author)

## Fitur Utama

- **Gambar path bebas (freehand)** — cukup drag mouse di jendela kamera, path otomatis dihaluskan dengan *spline* dan di-*resample* agar jaraknya seragam.
- **Lokalisasi berbasis kamera** — posisi (x, y) dan heading robot dibaca dari marker ArUco dengan *sub-pixel corner refinement* supaya heading stabil.
- **Auto-detect dictionary ArUco** — tidak perlu tahu jenis dictionary marker; sistem mencoba beberapa kandidat lalu mengunci yang berhasil.
- **Pure Pursuit** — robot mengejar *lookahead point* di sepanjang path.
- **Fuzzy PID (gain scheduling)** — nilai Kp, Ki, Kd disesuaikan otomatis oleh logika fuzzy (Sugeno) berdasarkan besar *heading error* dan laju perubahannya.
- **Kecepatan adaptif** — melambat sebelum tikungan (antisipatif) dan saat heading error besar (reaktif), dengan batas laju akselerasi dan pengereman.
- **Komunikasi nirkabel non-blocking** — thread terpisah mengirim perintah motor pada frekuensi tetap, dengan *auto-reconnect* bila koneksi terputus.
- **Fail-safe** — robot langsung berhenti jika marker tidak terdeteksi.
- **Kontrol manual WASD** — `TestRobot.py` untuk menguji koneksi, arah putaran motor, dan batas PWM sebelum mode otomatis dipakai.
- **Semua parameter di satu file** — seluruh tuning dilakukan lewat `config.py`, tanpa menyentuh kode lain.

## Arsitektur Sistem

<img width="1584" height="321" alt="Screenshot 2026-10-19 231518" src="https://github.com/user-attachments/assets/eafb8358-36a3-4a45-b72d-47df1737345f" />

Alur singkat setiap frame:

1. Kamera menangkap frame → `QRLocalizer` mencari marker dan menghitung posisi + heading.
2. `PurePursuit` mencari titik tujuan (*lookahead point*) di path dan menghitung *heading error*.
3. `FuzzyPIDController` mengubah heading error menjadi nilai belok (`turn`) dalam rentang −1..1.
4. Kecepatan dasar disesuaikan dengan kelengkungan path di depan, lalu dicampur (*mixing*) menjadi PWM roda kiri dan kanan.
5. `RobotClient` mengirim `{"left": L, "right": R}` ke ESP32.

## Struktur Proyek

```
.
├── esp32_firmware/
│   └── esp32_firmware.ino    # upload ke ESP32 via Arduino IDE
├── main.py                   # Entry point: loop kamera, UI, kontrol keyboard
├── TestRobot.py              # Kontrol manual WASD (Tkinter) untuk uji koneksi & motor
├── config.py                 # Semua parameter tuning (satu-satunya file yang perlu diedit)
├── qr_localization.py        # Deteksi marker ArUco → posisi & heading
├── path_planner.py           # Gambar path dengan mouse, smoothing spline, resample
├── pure_pursuit.py           # Algoritma pure pursuit + kontrol kecepatan adaptif
├── fuzzy_pid_controller.py   # PID dengan gain scheduling fuzzy (Sugeno)
├── robot_client.py           # Klien WebSocket ke ESP32 (thread + auto-reconnect)
├── requirements.txt          # Daftar dependensi Python
├── LICENSE                   # Lisensi MIT
└── README.md
```

## Kebutuhan

### Perangkat Keras

| Komponen | Keterangan |
|---|---|
| Robot differential drive | 2 motor DC dengan driver motor, dikendalikan ESP32 |
| ESP32 | Menjalankan WebSocket server (firmware ada di folder `esp32_firmware/`) |
| Kamera | Dipasang di atas area kerja, menghadap ke bawah, dengan seluruh area terlihat |
| Marker ArUco | Ditempel di bagian atas robot, ID default **67** |
| Komputer | Terhubung ke WiFi/AP milik ESP32 |

### Perangkat Lunak

- Python 3.8+
- Arduino IDE (untuk meng-upload firmware ke ESP32)
- Library Python (juga tercantum di `requirements.txt`):

```
opencv-contrib-python
numpy
scipy
websocket-client
```

> Catatan: modul `cv2.aruco` ada di paket **`opencv-contrib-python`**. Jangan install bersamaan dengan `opencv-python` biasa karena bisa bentrok.

## Instalasi

```bash
git clone https://github.com/Juhers/Slambengok-RobotKaro2.git
cd Slambengok-RobotKaro2

# (opsional) buat virtual environment
python -m venv venv
venv\Scripts\activate          # Windows
# source venv/bin/activate     # Linux/macOS

pip install -r requirements.txt
```

## Upload Firmware ESP32

Firmware ada di `esp32_firmware/esp32_firmware.ino`. Langkah umum:

1. Pasang **Arduino IDE** dan tambahkan dukungan board ESP32 lewat Boards Manager (paket *esp32* dari Espressif).
2. Buka `esp32_firmware/esp32_firmware.ino`.
3. Pasang library yang tertera pada baris `#include` di bagian atas sketch lewat Library Manager.
4. Sesuaikan definisi **pin motor** di sketch dengan wiring driver motor pada robot Anda, serta pengaturan WiFi (SSID dan password) bila diperlukan.
5. Pilih board ESP32 yang sesuai dan port COM, lalu klik **Upload**.
6. Buka Serial Monitor untuk memastikan ESP32 berjalan dan melihat alamat IP-nya. Pastikan nilainya sama dengan `ESP32_IP` di `config.py` (default `192.168.4.1`).

## Uji Koneksi Robot (TestRobot.py)

Sebelum menjalankan mode otomatis, uji dulu koneksi WiFi, arah putaran kedua motor, dan PWM minimum motor dengan kontrol manual:

```bash
python TestRobot.py
```

Jendela kecil akan terbuka. Isi IP ESP32 (default `192.168.4.1`), klik **Hubungkan**, lalu kendalikan robot:

| Input | Fungsi |
|---|---|
| `W` / `S` | Maju / mundur |
| `A` / `D` | Belok kiri / kanan |
| Spasi | Berhenti (darurat) |
| Slider | Atur kecepatan (0..255) |

Indikator status berubah dari **Menyambung...** (oranye) menjadi **Terhubung** (hijau) saat koneksi berhasil. Hal-hal yang sebaiknya dicek di tahap ini:

- Robot maju saat `W` ditekan. Jika mundur, balik polaritas kabel motor atau logika di firmware.
- Robot berbelok ke arah yang benar saat `A` / `D` ditekan.
- Nilai PWM terkecil yang masih membuat roda berputar. Gunakan nilainya untuk mengatur `MIN_PWM` di `config.py`.

> Tutup `TestRobot.py` sebelum menjalankan `main.py`, karena ESP32 sebaiknya hanya dikendalikan oleh satu klien pada satu waktu.

## Cara Menjalankan

1. **Nyalakan robot/ESP32**, lalu hubungkan komputer ke jaringan WiFi ESP32.
2. Pastikan `ESP32_IP`, `ESP32_PORT`, `CAM_INDEX`, dan `MARKER_ID` di `config.py` sudah sesuai.
3. Jalankan:

   ```bash
   python main.py
   ```

4. Letakkan robot dengan marker terlihat jelas oleh kamera (kotak hijau + panah heading akan muncul).
5. **Drag mouse** pada jendela kamera untuk menggambar path (garis kuning).
6. Tekan **`g`** untuk memulai. Path hasil haluskan (hijau) akan ditampilkan dan robot mulai bergerak.

### Kontrol Keyboard & Mouse

| Input | Fungsi |
|---|---|
| **Drag mouse kiri** | Gambar path baru (menimpa path sebelumnya) |
| `g` | Mulai mengikuti path |
| `s` | Stop paksa |
| `r` | Reset path dan hentikan robot |
| `q` | Keluar |

### Informasi di Layar

- **Kotak hijau + titik merah + panah biru** — marker terdeteksi beserta heading-nya
- **Titik magenta** — lookahead point saat ini
- **Kp / Ki / Kd** — gain yang sedang dipilih oleh fuzzy scheduler
- **slow** — faktor perlambatan (0 = lurus, 1 = paling pelan)
- **Status** — `FOLLOWING` atau `IDLE`

## Protokol Komunikasi ESP32

Program PC bertindak sebagai **WebSocket client** dan terhubung ke `ws://<ESP32_IP>:<ESP32_PORT>/` (default `ws://192.168.4.1:81/`).

Pesan dikirim sebagai JSON pada frekuensi `SEND_HZ` (default 25 Hz):

```json
{"left": 180, "right": 150}
```

- Nilai `left` dan `right` adalah PWM bertanda dalam rentang **−255..255** (negatif = mundur).
- `{"left": 0, "right": 0}` berarti berhenti.
- Perintah terakhir dikirim berulang terus, sehingga **sebaiknya firmware ESP32 menerapkan *watchdog timeout*** (motor berhenti bila tidak ada pesan dalam beberapa ratus ms) sebagai lapisan keamanan tambahan.

> Catatan: jika Anda ingin memakai mikrokontroler atau firmware sendiri, cukup sediakan WebSocket server yang menerima format JSON di atas dan menggerakkan motor.

## Cara Kerja Detail

### 1. Lokalisasi (`qr_localization.py`)

- Posisi robot = titik tengah (rata-rata) keempat sudut marker.
- Heading dihitung dari vektor sudut 0 → sudut 1 marker, dalam koordinat pixel (sumbu Y ke bawah positif). Pastikan **sisi atas marker menghadap ke depan robot**.
- Jika `ARUCO_DICT = None`, sistem mencoba semua dictionary di `ARUCO_CANDIDATES` dan mengunci yang pertama berhasil mendeteksi marker.

### 2. Path Planner (`path_planner.py`)

- Titik mentah dari mouse dibersihkan dari duplikat, lalu dihaluskan dengan `scipy.interpolate.splprep`.
- Hasilnya di-*resample* tiap `PATH_SPACING_PX` pixel agar titik-titik path seragam.
- Jika spline gagal (misalnya titik kolinear), sistem memakai titik asli.

### 3. Pure Pursuit (`pure_pursuit.py`)

- Mencari titik pertama di path yang berjarak ≥ `LOOKAHEAD_PX` dari robot, dan indeks path hanya bergerak maju (tidak mundur).
- *Heading error* = sudut menuju target − heading robot, dinormalisasi ke −180°..180°.
- **Kecepatan adaptif**: `slow = max(curve_factor, error_factor)`
  - `curve_factor` → intip path sejauh `PREVIEW_DIST` untuk mendeteksi tikungan (juga melambat mendekati ujung path).
  - `error_factor` → perlambatan reaktif berdasarkan besar heading error.
  - Perubahan kecepatan dibatasi oleh `BRAKE_RATE` dan `ACCEL_RATE`.
- **Mixing roda**: `left = speed × (1 − turn)`, `right = speed × (1 + turn)`.
- **Dead-zone PWM**: nilai di bawah `MIN_PWM` dibulatkan ke 0 karena motor hanya berdengung tanpa berputar.
- Path dianggap **selesai** bila robot berada dalam `FINISH_TOLERANCE_PX` dari ujung path.

### 4. Fuzzy PID (`fuzzy_pid_controller.py`)

Pengendali belok berupa PID klasik yang gain-nya dijadwalkan secara fuzzy:

- **Input**: `|error|` (derajat) dan `|ec|` (laju perubahan error, deg/s, difilter low-pass).
- **Himpunan fuzzy**: tiga kategori — Small (S), Medium (M), Large (L) — berbentuk segitiga.
- **Inferensi**: Sugeno *weighted average* dengan 9 aturan per gain (tabel `TABLE_KP`, `TABLE_KI`, `TABLE_KD`), kekuatan aturan = `min(μ_e, μ_ec)`.
- **Output**: Kp, Ki, Kd dipetakan ke rentang `KP_RANGE`, `KI_RANGE`, `KD_RANGE`.
- **Pengaman**: *anti-windup* integral, clipping `ec`, dan filter low-pass pada derivative untuk mencegah *derivative kick* akibat jitter deteksi marker.
- Output dinormalisasi dengan `FUZZY_ERROR_RANGE` ke rentang −1..1, lalu dihaluskan lagi di `pure_pursuit.py` (`TURN_SMOOTH`).

### 5. Robot Client (`robot_client.py`)

- Thread daemon mengirim perintah terakhir secara periodik, terpisah dari loop kamera, sehingga FPS kamera tidak terganggu latensi jaringan.
- `TCP_NODELAY` aktif untuk latensi rendah.
- Jika koneksi putus, otomatis mencoba menyambung ulang tiap `RETRY_INTERVAL` detik.
- `close()` mengirim perintah stop dan memberi jeda `STOP_FLUSH_DELAY` sebelum menutup koneksi.
- Dipakai bersama oleh `main.py` (mode otomatis) dan `TestRobot.py` (mode manual).

## Panduan Tuning (`config.py`)

Semua satuan: jarak = pixel, sudut = derajat, waktu = detik.

| Gejala | Parameter yang disarankan |
|---|---|
| Robot berosilasi / zig-zag di jalur lurus | Naikkan `LOOKAHEAD_PX`, turunkan `STEER_BOOST`, turunkan `KP_RANGE` maksimum, turunkan `TURN_SMOOTH` |
| Robot memotong tikungan | Turunkan `LOOKAHEAD_PX`, turunkan `CORNER_SPEED_RATIO`, naikkan `PREVIEW_DIST` |
| Robot terlalu lambat bereaksi saat belok | Naikkan `KP_RANGE`, naikkan `STEER_BOOST`, naikkan `TURN_SMOOTH` |
| Motor berdengung tapi tidak berputar | Naikkan `MIN_PWM` sesuai batas PWM minimum motor Anda |
| Robot melaju terlalu kencang / lambat | Atur `BASE_SPEED` dan `MAX_SPEED` |
| Turn melonjak tiba-tiba | Turunkan `EC_FILTER_ALPHA`, turunkan `KD_RANGE` |
| Robot overshoot saat berhenti | Naikkan `FINISH_TOLERANCE_PX`, turunkan `ACCEL_RATE`, naikkan `BRAKE_RATE` |
| Path terlalu bergelombang | Naikkan `PATH_SMOOTHING` (tetapi path makin menyimpang dari gambar) |
| Respons robot terlambat karena latensi WiFi | Gunakan `PurePursuit.predict_pose()` dan atur `PREDICT_DELAY` |

Kelompok parameter di `config.py`:

1. **Koneksi ESP32** — IP, port, frekuensi kirim, timeout
2. **Kamera & tampilan** — indeks kamera, nama jendela
3. **Deteksi marker** — ID marker, dictionary ArUco
4. **Path** — jarak resample, tingkat smoothing, panjang minimum
5. **Pure Pursuit** — lookahead, kecepatan, perilaku tikungan
6. **Fuzzy PID** — rentang error, rentang gain, bentuk himpunan fuzzy, tabel aturan

> Tips: Tuning bisa dimulai dengan `BASE_SPEED` rendah dan path sederhana (lurus lalu lengkung lembut), kemudian tingkatkan kecepatan bertahap.

## Troubleshooting

| Masalah | Solusi |
|---|---|
| `Gagal membuka kamera` | Ubah `CAM_INDEX` di `config.py`; pastikan kamera tidak dipakai aplikasi lain |
| Marker tidak terdeteksi | Pastikan `MARKER_ID` benar, pencahayaan cukup, tidak ada pantulan/silau; atur exposure kamera di `main.py` |
| Error `cv2.aruco` tidak ditemukan | Install `opencv-contrib-python` (bukan `opencv-python`) |
| `Gagal konek ke ESP32` | Cek koneksi WiFi ke ESP32, serta `ESP32_IP` dan `ESP32_PORT`; coba uji dengan `TestRobot.py` |
| Robot tidak bergerak sama sekali | Uji dengan `TestRobot.py` untuk memisahkan masalah koneksi/motor dari masalah kamera dan path |
| Robot berputar ke arah yang salah | Periksa orientasi marker (sudut 0→1 harus searah depan robot) dan polaritas motor/kabel |
| Robot berhenti tiba-tiba saat jalan | Marker keluar dari pandangan kamera atau terhalang — ini perilaku fail-safe yang disengaja |
| Path tidak mau dijalankan | Path terlalu pendek (minimal `PATH_MIN_POINTS_TO_RUN` titik); gambar ulang dengan goresan lebih panjang |

## Catatan Platform

`main.py` membuka kamera dengan backend `cv2.CAP_DSHOW` (khusus **Windows**) beserta pengaturan exposure manual. Di Linux/macOS, ganti baris berikut:

```python
cap = cv2.VideoCapture(cfg.CAM_INDEX, cv2.CAP_DSHOW)
```

menjadi:

```python
cap = cv2.VideoCapture(cfg.CAM_INDEX)
```

dan sesuaikan atau hapus pengaturan `CAP_PROP_AUTO_EXPOSURE` / `CAP_PROP_EXPOSURE` jika tidak didukung kamera Anda.

`TestRobot.py` memakai Tkinter yang sudah disertakan pada instalasi Python di Windows dan macOS. Di Linux, install dengan `sudo apt install python3-tk`.

## Keselamatan

- Uji pertama kali dengan roda terangkat atau di area yang lapang.
- Siapkan tombol `s` (stop) atau `q` (keluar) selalu dalam jangkauan.
- Implementasikan *watchdog* di firmware ESP32 agar motor berhenti jika koneksi terputus.

## Pengembangan Lanjutan

- Penggunaan `predict_pose()` untuk kompensasi delay nirkabel (fungsi sudah tersedia di `pure_pursuit.py`, namun belum dipanggil di `main.py`)
- Kalibrasi kamera / konversi pixel → satuan metrik
- Penyimpanan dan pemuatan path dari file
- Logging data (error, gain, PWM) untuk analisis dan tuning
- Dukungan multi-robot dengan ID marker berbeda

## Kontribusi

Kontribusi sangat diterima. Ada dua cara utama untuk ikut berkontribusi di GitHub:

### Issue

*Issue* adalah catatan di halaman repositori (tab **Issues**) untuk melaporkan masalah, bertanya, atau mengusulkan fitur. Gunakan issue bila Anda menemukan bug, mengalami error, atau punya ide pengembangan, tanpa perlu mengubah kode.

Saat membuat issue, sertakan sebisa mungkin:

- Deskripsi singkat masalah atau usulan
- Langkah untuk mengulang masalah
- Perilaku yang diharapkan dan yang terjadi
- Sistem operasi, versi Python, dan versi OpenCV
- Potongan log atau pesan error

### Pull Request

*Pull request* (PR) adalah usulan perubahan kode. Anda membuat salinan repositori (*fork*), mengubah kode di salinan tersebut, lalu mengajukan perubahan itu agar digabungkan ke repositori utama setelah ditinjau pemilik repositori.

Langkah umum:

1. *Fork* repositori ini lewat tombol **Fork** di GitHub.
2. Clone hasil fork, lalu buat branch baru:

   ```bash
   git clone https://github.com/<username-anda>/Slambengok-RobotKaro2.git
   cd Slambengok-RobotKaro2
   git checkout -b nama-fitur-atau-perbaikan
   ```

3. Lakukan perubahan, lalu commit dan push:

   ```bash
   git add .
   git commit -m "Jelaskan perubahan secara singkat"
   git push origin nama-fitur-atau-perbaikan
   ```

4. Buka repositori fork Anda di GitHub, klik **Compare & pull request**, jelaskan perubahan, lalu kirim.

Tips: usahakan satu PR hanya untuk satu perubahan, jelaskan alasannya, dan jika ini terkait parameter tuning, ubah lewat `config.py` agar konsisten dengan struktur proyek.

## Lisensi

Proyek ini dirilis di bawah lisensi **MIT**. Lisensi adalah aturan yang menjelaskan apa yang boleh dilakukan orang lain terhadap kode ini (menggunakan, mengubah, menyebarkan). Lisensi MIT mengizinkan siapa pun memakai, memodifikasi, dan menyebarkan kode ini, termasuk untuk keperluan komersial, selama menyertakan pemberitahuan hak cipta dan teks lisensi aslinya. Kode disediakan apa adanya tanpa jaminan apa pun.

Teks lengkap lisensi ada di file [LICENSE](LICENSE).

## Author

Dibuat oleh **M. Zuhayr Ramadhan**. Untuk saran dan perbaikan, silakan buka [issue](https://github.com/Juhers/Slambengok-RobotKaro2/issues) atau ajukan [pull request](https://github.com/Juhers/Slambengok-RobotKaro2/pulls).
