import math
import numpy as np
import matplotlib.pyplot as plt


# Data dari NIM 26/586885/SA/25005

NIM = "26/586885/SA/25005"
bagian = NIM.split("/")
NIU, NIF = bagian[1], bagian[3]

L_FEMUR = float(int(NIU[-2:]))   # 85 cm
L_TIBIA = float(int(NIF[-2:]))   # 5 cm
L_COXA = 10.0                    # cm 


# Matriks Transformasi Homogen 4x4
def rot_z(theta):
    """Rotasi terhadap sumbu-Z (yaw coxa)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]])


def rot_pitch(q):
    """Rotasi di bidang kaki: q positif (CCW) mengangkat ujung ke +Z.
    Ini adalah rotasi terhadap sumbu-Y dengan sudut -q."""
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, 0, -s, 0],
                     [0, 1,  0, 0],
                     [s, 0,  c, 0],
                     [0, 0,  0, 1]])


def trans_x(d):
    """Translasi sepanjang sumbu-X lokal (panjang link)."""
    T = np.eye(4)
    T[0, 3] = d
    return T


# Forward Kinematics
def forward_kinematics(theta1_deg, q1_deg, q2_deg,
                       l_coxa=L_COXA, l_femur=L_FEMUR, l_tibia=L_TIBIA):
    """Kembalikan (daftar titik sendi, matriks T akhir)."""
    th1 = math.radians(theta1_deg)
    q1 = math.radians(q1_deg)
    q2 = math.radians(q2_deg)

    T0 = np.eye(4)                                   # pangkal (0,0,0)
    T1 = T0 @ rot_z(th1) @ trans_x(l_coxa)           # ujung Coxa
    T2 = T1 @ rot_pitch(q1) @ trans_x(l_femur)       # ujung Femur (lutut)
    T3 = T2 @ rot_pitch(q2) @ trans_x(l_tibia)       # ujung Tibia (end-effector)

    titik = [T[:3, 3] for T in (T0, T1, T2, T3)]
    return titik, T3


# Inverse Kinematics
def inverse_kinematics(x, y, z, elbow=+1,
                       l_coxa=L_COXA, l_femur=L_FEMUR, l_tibia=L_TIBIA):
    """Cari (theta1, q1, q2) dalam derajat dari target (x, y, z).
    elbow = +1 / -1 memilih salah satu dari dua solusi lutut.
    Kembalikan None jika target di luar jangkauan."""
    theta1 = math.atan2(y, x)

    r = math.hypot(x, y) - l_coxa    # jarak horizontal dari ujung Coxa
    d2 = r * r + z * z               # kuadrat jarak ke target
    d = math.sqrt(d2)

    if d > l_femur + l_tibia or d < abs(l_femur - l_tibia):
        return None

    c2 = (d2 - l_femur ** 2 - l_tibia ** 2) / (2 * l_femur * l_tibia)
    c2 = max(-1.0, min(1.0, c2))
    q2 = elbow * math.acos(c2)

    q1 = math.atan2(z, r) - math.atan2(l_tibia * math.sin(q2),
                                       l_femur + l_tibia * math.cos(q2))
    return math.degrees(theta1), math.degrees(q1), math.degrees(q2)


# Visualisasi
def plot_robot(titik, nama_file="posture_robot.png", judul="Postur Kaki Robot"):
    P = np.array(titik)
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    warna = ["tab:green", "tab:red", "tab:blue"]
    label = ["Coxa", "Femur", "Tibia"]
    for i in range(3):
        ax.plot(P[i:i + 2, 0], P[i:i + 2, 1], P[i:i + 2, 2],
                color=warna[i], linewidth=5, label=label[i])
    ax.scatter(P[:, 0], P[:, 1], P[:, 2], color="black", s=30)
    ax.scatter(*P[-1], color="gold", s=90, edgecolor="black",
               label="End-effector", zorder=5)
    ax.text(*P[-1], f"  ({P[-1][0]:.2f}, {P[-1][1]:.2f}, {P[-1][2]:.2f})")

    batas = max(np.abs(P).max(), 1) * 1.1
    ax.set_xlim(-batas / 4, batas)
    ax.set_ylim(-batas / 2, batas / 2)
    ax.set_zlim(-batas / 4, batas)
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_zlabel("Z (cm)")
    ax.set_title(judul)
    ax.legend()
    plt.tight_layout()
    plt.savefig(nama_file, dpi=150)
    print(f"Gambar disimpan: {nama_file}")
    try:
        plt.show()
    except Exception:
        pass
    plt.close(fig)


# Program utama
def tampil_hasil_fk(theta1, q1, q2):
    titik, T = forward_kinematics(theta1, q1, q2)
    np.set_printoptions(precision=4, suppress=True)
    print("\n Matriks Transformasi Homogen Akhir (T 0->3) ")
    print(T)
    print("\n Posisi tiap sendi (x, y, z) cm ")
    for nama, p in zip(["Pangkal", "Ujung Coxa", "Lutut (Femur-Tibia)",
                        "End-Effector"], titik):
        print(f"{nama:22s}: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")
    return titik


def main():
    print("Kinematika Kaki Robot 3 DoF (3D)")
    print(f"NIM           : {NIM}")
    print(f"Panjang Coxa  : {L_COXA:g} cm")
    print(f"Panjang Femur : {L_FEMUR:g} cm")
    print(f"Panjang Tibia : {L_TIBIA:g} cm")
    print("(CCW = positif, CW = negatif)\n")

    print("Pilih mode:")
    print("  1. Forward Kinematics (sudut -> posisi)")
    print("  2. Inverse Kinematics (posisi -> sudut)")
    mode = input("Mode (1/2): ").strip()

    if mode == "1":
        theta1 = float(input("Rotasi Coxa / yaw (derajat)    : "))
        q1 = float(input("Rotasi Servo 1 / Femur (derajat): "))
        q2 = float(input("Rotasi Servo 2 / Tibia (derajat): "))
        titik = tampil_hasil_fk(theta1, q1, q2)
        plot_robot(titik, "posture_fk.png",
                   f"FK: Coxa={theta1:g}, Servo1={q1:g}, Servo2={q2:g}")

    elif mode == "2":
        x = float(input("Target X (cm): "))
        y = float(input("Target Y (cm): "))
        z = float(input("Target Z (cm): "))
        elbow = +1 if input("Posisi lutut (1=atas, 2=bawah): ").strip() == "1" else -1
        hasil = inverse_kinematics(x, y, z, elbow)
        if hasil is None:
            print("Target di luar jangkauan kaki robot.")
            return
        theta1, q1, q2 = hasil
        print("\n Hasil IK")
        print(f"Coxa          : {theta1:.2f} derajat")
        print(f"Servo 1 (Femur): {q1:.2f} derajat")
        print(f"Servo 2 (Tibia): {q2:.2f} derajat")
        print("\nVerifikasi dengan FK:")
        titik = tampil_hasil_fk(theta1, q1, q2)
        plot_robot(titik, "posture_ik.png",
                   f"IK target=({x:g}, {y:g}, {z:g})")
    else:
        print("Mode tidak dikenal.")


if __name__ == "__main__":
    main()