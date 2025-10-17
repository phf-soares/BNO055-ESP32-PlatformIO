import serial
import time
import csv
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime
import pandas as pd # pyright: ignore[reportMissingModuleSource]

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
axs[0].plot(t, ax, color='r')
axs[0].set_ylabel("A_x (m/s²)")
axs[0].grid(True)

# Eixo Y
axs[1].plot(t, ay, color='g')
axs[1].set_ylabel("A_y (m/s²)")
axs[1].grid(True)

# Eixo Z
axs[2].plot(t, az, color='b')
axs[2].set_ylabel("A_z (m/s²)")
axs[2].set_xlabel("Tempo (s)")
axs[2].grid(True)

plt.tight_layout(rect=[0, 0, 1, 0.97])  # espaço pro título
plt.show()