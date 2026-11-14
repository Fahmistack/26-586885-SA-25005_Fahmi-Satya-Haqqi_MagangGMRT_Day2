# Forward & Inverse Kinematics Kaki Robot 3 DoF (3D)

**NAMA:** Fahmi Satya Haqqi

**NIM:** 26/586885/SA/25005

## Deskripsi Singkat

Program ini menghitung posisi ujung kaki robot (end-effector) berdasarkan sudut servo, dan sebaliknya. Kaki robot terdiri dari tiga bagian: Coxa, Femur, dan Tibia. Titik pusat koordinat (0,0,0) berada di sendi pangkal (Coxa).

Studi kasus awalnya adalah Forward Kinematics 2 DoF (Femur dan Tibia, Coxa = 0). Program ini dikembangkan menjadi sistem 3 DoF dalam ruang tiga dimensi (X, Y, Z) dengan panjang Coxa 10 cm. Perhitungan memakai Homogeneous Transformation Matrix 4x4 dengan NumPy. Program juga menyediakan Inverse Kinematics dan visualisasi postur kaki dengan matplotlib.

**Ketentuan variabel dari NIM:**

| Bagian | Sumber | Panjang |
|---|---|---|
| Femur | 2 digit terakhir NIU (586885) | 85 cm |
| Tibia | 2 digit terakhir NIF (25005) | 5 cm |
| Coxa | ditetapkan sendiri | 10 cm |

Arah putar positif = Counter-Clockwise (CCW), negatif = Clockwise (CW). Posisi 0 derajat adalah saat lengan sejajar sumbu-X positif.

**Rumus FK 2D (kasus dasar):**

```
x = a1 cos(q1) + a2 cos(q1 + q2)
y = a1 sin(q1) + a2 sin(q1 + q2)
```

**Test case (Servo 1 = 40 derajat, Servo 2 = 30 derajat):**

- FK 2D (Coxa = 0): end-effector = (66.82, 59.34) cm
- FK 3D (Coxa 10 cm, yaw 0): end-effector = (76.82, 0.00, 59.34) cm

## Fitur yang Diimplementasikan

**Fitur wajib**

| Fitur | Status |
|---|---|
| Program Python interaktif dengan `input()` | ✅ |
| Perhitungan koordinat end-effector dengan rumus FK | ✅ |
| Panjang Femur dan Tibia dari NIU dan NIF | ✅ |
| Arah putar CCW positif, CW negatif | ✅ |
| Test case Servo 1 = 40 dan Servo 2 = 30 | ✅ |

**Fitur bonus**

| Fitur | Status |
|---|---|
| Mathematical Proof (foto coretan tangan) | ✅ |
| N-DoF (3 DoF, Coxa tidak lagi 0) | ✅ |
| Spatial Kinematics (X, Y, Z) | ✅ |
| Homogeneous Transformation (NumPy) | ✅ |
| Visualisasi plot 3D (matplotlib, simpan PNG) | ✅ |
| Inverse Kinematics (target X, Y, Z menjadi sudut) | ✅ |

## Cara Instalasi dan Menjalankan Program

**1. Prasyarat**

Python 3.12.10

```
python --version
```

**2. Pasang library**

```
python -m pip install numpy matplotlib
```

**3. Jalankan program**

Buka terminal di folder program, lalu jalankan:

```
python forward_kinematics_3d.py
```

**4. Pilih mode dan isi input**

*Mode 1 - Forward Kinematics:* masukkan sudut Coxa, Servo 1 (Femur), dan Servo 2 (Tibia) dalam derajat. Contoh test case:

```
Mode (1/2): 1
Rotasi Coxa / yaw (derajat)    : 0
Rotasi Servo 1 / Femur (derajat): 40
Rotasi Servo 2 / Tibia (derajat): 30
```

Output: matriks transformasi homogen, posisi tiap sendi, dan `posture_fk.png`.

*Mode 2 - Inverse Kinematics:* masukkan target X, Y, Z dalam cm dan pilih posisi lutut. Program menampilkan sudut Coxa, Servo 1, dan Servo 2, lalu memverifikasinya dengan FK dan menyimpan `posture_ik.png`.

## Output Test Case

```
    Posisi tiap sendi (x, y, z) cm 
Pangkal               : (0.00, 0.00, 0.00)
Ujung Coxa            : (10.00, 0.00, 0.00)
Lutut (Femur-Tibia)   : (75.11, 0.00, 54.64)
End-Effector          : (76.82, 0.00, 59.34)
```

(Tempelkan screenshot terminal dan `posture_fk.png` di sini.)
