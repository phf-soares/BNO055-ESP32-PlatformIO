import serial
import time
import csv
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime
import pandas as pd # pyright: ignore[reportMissingModuleSource]
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt # pyright: ignore[reportMissingImports]
from scipy.fft import rfft, rfftfreq # pyright: ignore[reportMissingImports]

# === CONFIGURAÇÕES ===
PORT = "COM7"           # Porta serial do ESP32
BAUD = 1600000          # Baud rate configurado no ESP32
DURATION = 10           # Tempo de coleta (segundos)

# === PASTAS E NOMES DE SAÍDA ===
os.makedirs("scripts/data", exist_ok=True)
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_FILENAME = f"scripts/data/bno055_data_{timestamp}.csv"

# === ABRIR SERIAL ===
ser = serial.Serial(PORT, BAUD, timeout=0.02)
ser.flushInput()

print(f"📡 Coletando dados por {DURATION}s em {PORT} a {BAUD} bps...")

start = time.time()
data = []

# === COLETAR DADOS ===
while (time.time() - start) < DURATION:
    try:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        parts = line.split(",")
        if len(parts) == 3:
            x, y, z = map(float, parts)
            timestamp = time.time() - start
            data.append([timestamp, x, y, z])
    except Exception:
        pass

ser.close()
print(f"✅ Coleta concluída — {len(data)} amostras capturadas")

# === SALVAR EM CSV ===
with open(CSV_FILENAME, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["tempo_s", "ax", "ay", "az"])
    writer.writerows(data)

print(f"💾 Dados salvos em '{CSV_FILENAME}'")

# === CALCULAR TAXA DE AMOSTRAGEM ===
duration_real = data[-1][0] - data[0][0] if len(data) > 1 else 0
sample_rate = len(data) / duration_real if duration_real > 0 else 0
print(f"📊 Taxa média de amostragem: {sample_rate:.1f} Hz")

# === LER CSV ===
data = pd.read_csv(CSV_FILENAME)

t = data["tempo_s"]
ax = data["ax"]
ay = data["ay"]
az = data["az"]
# estima fs a partir do tempo
fs = 1 / np.mean(np.diff(t))

# === FFT DO SINAL ===
yf = np.abs(rfft(ax))
xf = rfftfreq(len(ax), 1/fs)

plt.semilogy(xf, yf)
plt.xlabel("Frequência (Hz)")
plt.ylabel("|A(f)|")
plt.grid(True)
plt.title("Espectro de aceleração")
plt.show()

# === Remove o pico em 0 Hz (DC) ===
dc_region = xf < 0.5  # ignora abaixo de 0.5 Hz
yf_nodc = yf.copy()
yf_nodc[dc_region] = 0

# === Normalização linear ===
yf_norm = yf_nodc / np.max(yf_nodc)

# === Encontra pico máximo ===
idx_peak = np.argmax(yf_norm)
f_peak = xf[idx_peak]
amp_peak = yf[idx_peak]
amp_norm_peak = yf_norm[idx_peak]

# === Plotar ===
plt.figure(figsize=(10, 5))
plt.plot(xf, yf_norm, label="Espectro normalizado (linear)")
plt.axvline(f_peak, color='r', linestyle='--', label=f"Pico {f_peak:.2f} Hz")
plt.xlabel("Frequência (Hz)")
plt.ylabel("Amplitude normalizada")
plt.title("Espectro normalizado - eixo X")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

print(f"🔍 Frequência dominante: {f_peak:.2f} Hz (amplitude = {amp_norm_peak:.2f})")

# === FILTRAGEM ===

df = pd.read_csv(CSV_FILENAME)
t = df["tempo_s"].values
ax = df["ax"].values; ay = df["ay"].values; az = df["az"].values

fc = f_peak*1.2;  # Hz (ajuste conforme seu caso)
wn = fc / (fs/2) # frequência normalizada (para butter)

b, a = butter(N=4, Wn=wn, btype='low')   # ordem 4
ax_f = filtfilt(b, a, ax)                # zero-fase (sem atraso)
ay_f = filtfilt(b, a, ay)
az_f = filtfilt(b, a, az)

# === CONFIGURAR ESTILO MATLAB ===
plt.style.use('classic')
plt.rcParams.update({
    'font.size': 11,
    'axes.labelweight': 'bold',
    'axes.titlesize': 12,
    'axes.titleweight': 'bold',
    'lines.linewidth': 1.2,
    'grid.color': '0.8',
    'grid.linestyle': '--',
    'grid.linewidth': 0.6
})

# === CRIAR SUBPLOTS ===
fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
fig.suptitle("BNO055 - Aceleração por Eixo", fontsize=14, fontweight='bold')

# Eixo X
axs[0].plot(t, ax_f, color='r')
axs[0].set_ylabel("A_x (m/s²)")
axs[0].grid(True)

# Eixo Y
axs[1].plot(t, ay_f, color='g')
axs[1].set_ylabel("A_y (m/s²)")
axs[1].grid(True)

# Eixo Z
axs[2].plot(t, az_f, color='b')
axs[2].set_ylabel("A_z (m/s²)")
axs[2].set_xlabel("Tempo (s)")
axs[2].grid(True)

plt.tight_layout(rect=[0, 0, 1, 0.97])  # espaço pro título
plt.show()