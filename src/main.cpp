#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// === CONFIGURAÇÕES ===
#define CS_PIN 5             // Pino CS do módulo SD (ajuste conforme seu hardware)
#define SAMPLE_PERIOD_MS 3   // 1 ms = 1 kHz
#define QUEUE_LENGTH 256
#define LOG_DURATION_MS 10000  // 10 segundos de gravação

// === VARIÁVEIS GLOBAIS ===
QueueHandle_t dataQueue;
File dataFile;
volatile bool stopLogging = false;

// Estrutura de dados da amostra
typedef struct {
  uint32_t timestamp;
  int value;
} Sample_t;

// === TAREFA 1: LEITURA DO SENSOR ===
void taskSensor(void *pvParameters) {
  Sample_t sample;
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    if (stopLogging) break;

    sample.timestamp = millis();
    sample.value = 12;  // Simula sensor analógico

    // Envia amostra para fila (não bloqueante)
    xQueueSend(dataQueue, &sample, 0);

    // Mantém a taxa de amostragem exata
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }

  Serial.println("taskSensor finalizada.");
  vTaskDelete(NULL);
}

// === TAREFA 2: GRAVAÇÃO NO SD ===
void taskSD(void *pvParameters) {
  Sample_t received;

  dataFile = SD.open("/dados.csv", FILE_WRITE);
  if (!dataFile) {
    Serial.println("Erro ao abrir arquivo para escrita!");
    vTaskDelete(NULL);
  }

  dataFile.println("timestamp_ms,valor");

  for (;;) {
    if (stopLogging && uxQueueMessagesWaiting(dataQueue) == 0) break;

    if (xQueueReceive(dataQueue, &received, pdMS_TO_TICKS(100))) {
      dataFile.printf("%lu,%d\n", received.timestamp, received.value);
    }
  }

  dataFile.flush();
  dataFile.close();
  Serial.println("Arquivo fechado. taskSD finalizada.");

  vTaskDelete(NULL);
}

// === SETUP PRINCIPAL ===
void setup() {
  Serial.begin(BAUD_RATE);
  delay(1000);
  Serial.println("Inicializando...");

  // Inicia SD
  if (!SD.begin(CS_PIN)) {
    Serial.println("Falha ao iniciar SD!");
    while (true);
  }
  Serial.println("SD iniciado com sucesso.");

  // Cria fila
  dataQueue = xQueueCreate(QUEUE_LENGTH, sizeof(Sample_t));
  if (dataQueue == NULL) {
    Serial.println("Erro ao criar a fila!");
    while (true);
  }

  // Cria tarefas
  xTaskCreatePinnedToCore(taskSensor, "SensorTask", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskSD, "SDTask", 8192, NULL, 1, NULL, 1);

  // Aguarda 10 s de gravação
  Serial.println("Gravando dados por 10 segundos...");
  delay(LOG_DURATION_MS);

  // Para a gravação
  stopLogging = true;
  Serial.println("Parando gravação...");
  delay(2000);  // Dá tempo das tarefas fecharem o arquivo

  // === Leitura do arquivo ===
  Serial.println("\n--- LENDO ARQUIVO /dados.csv ---");
  File readFile = SD.open("/dados.csv", FILE_READ);
  if (!readFile) {
    Serial.println("Erro ao abrir arquivo para leitura!");
    return;
  }

  while (readFile.available()) {
    Serial.write(readFile.read());
  }
  readFile.close();

  Serial.println("\n--- FIM DA LEITURA ---");
}

void loop() {
  // Nada — FreeRTOS e setup fazem tudo
}
