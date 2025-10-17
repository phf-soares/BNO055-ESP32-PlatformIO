import os
import re
import csv
import time
import json
import serial
import argparse
import subprocess
import configparser
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd # pyright: ignore[reportMissingModuleSource]
from datetime import datetime
from scipy.signal import butter, filtfilt # pyright: ignore[reportMissingImports]
from scipy.fft import rfft, rfftfreq # pyright: ignore[reportMissingImports]

# === DETECTAR PORTA SERIAL, BAUD E DURAÇÃO DE COLETA ===
def get_pio_serial_port():
    """Tenta detectar automaticamente a porta serial usada pelo PlatformIO."""
    try:
        result = subprocess.run(
            ["pio", "device", "list", "--json-output"],
            capture_output=True, text=True, check=True
        )
        devices = json.loads(result.stdout)
        for dev in devices:
            if "port" in dev and ("usb" in dev["port"].lower() or "COM" in dev["port"].upper()):
                return dev["port"]
    except Exception as e:
        print(f"⚠️ Erro ao detectar porta do PlatformIO: {e}")
    return None

def detect_serial_port():
    port = get_pio_serial_port()
    if port:
        return port
    else:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        if ports:
            return ports[0].device
    return None

def read_pio_ini_value(key, default=None):
    """Lê automaticamente um valor do platformio.ini (sem precisar do nome do ambiente)."""
    ini_path = os.path.join(os.path.dirname(__file__), "..", "platformio.ini")

    config = configparser.ConfigParser()
    config.read(ini_path)

    # encontra a primeira seção [env:...]
    env_section = None
    for section in config.sections():
        if re.match(r"env:", section):
            env_section = section
            break

    # tenta ler o valor na seção encontrada
    if env_section and key in config[env_section]:
        return config[env_section][key].strip()
    elif "platformio" in config and key in config["platformio"]:
        return config["platformio"][key].strip()

    return default

# === CONFIGURAÇÕES ===
PORT = detect_serial_port() or "COM3"           # Porta serial do ESP32
BAUD = int(read_pio_ini_value("monitor_speed", 1600000))                  # Baud rate configurado no ESP32
DURATION = float(read_pio_ini_value("custom_data_duration_s", 10))                 # Tempo de coleta (segundos)

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

# === PARÂMETROS DE LINHA DE COMANDO ===
parser = argparse.ArgumentParser(description="Análise de dados do BNO055")
parser.add_argument("--fft", action="store_true", help="Executa FFT do arquivo CSV")
parser.add_argument("--nofilter", action="store_true", help="Sinal original do arquivo CSV")
args = parser.parse_args()

if args.fft:
    # === Plotar ===
    plt.semilogy(xf, yf)
    plt.xlabel("Frequência (Hz)")
    plt.ylabel("|A(f)|")
    plt.grid(True)
    plt.title("Espectro de aceleração")
    plt.show()

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

if args.nofilter:
    fig2, axs2 = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig2.suptitle("BNO055 - Aceleração por Eixo Sem Pós Filtro", fontsize=14, fontweight='bold')

    # Eixo X
    axs2[0].plot(t, ax, color='r')
    axs2[0].set_ylabel("A_x (m/s²)")
    axs2[0].grid(True)

    # Eixo Y
    axs2[1].plot(t, ay, color='g')
    axs2[1].set_ylabel("A_y (m/s²)")
    axs2[1].grid(True)

    # Eixo Z
    axs2[2].plot(t, az, color='b')
    axs2[2].set_ylabel("A_z (m/s²)")
    axs2[2].set_xlabel("Tempo (s)")
    axs2[2].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.97])  # espaço pro título
    plt.show()
