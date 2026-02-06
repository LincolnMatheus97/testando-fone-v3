#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>
#include "led_bar.h"

// --- Configurações de Hardware ---
#define I2S_SCK 4
#define I2S_WS  5
#define I2S_SD  6
#define I2S_PORT I2S_NUM_0

// --- Áudio ---
#define SAMPLE_RATE 44100
#define BLOCK_SIZE 1024
#define OFFSET_DB  98.0f 

// --- Coeficientes do Filtro A (Recalculados para 44.1kHz) ---
float b0 = 0.169996, b1_f = 0.280415, b2 = -0.11204, b3 = -0.131557; 
float a1_f = -2.12979, a2 = 1.63772, a3 = -0.463633;                 
float x1_h = 0, x2_h = 0, x3_h = 0, y1_h = 0, y2_h = 0, y3_h = 0;

void setup() {
  delay(2000);
  Serial.begin(115200);
  
  // 1. Inicializa LEDs
  inicializar_ledbar();
  
  // 2. Configura I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  
  Serial.println("Sistema SE:MAPA Iniciado Completo!");
}

float aplicar_filtro_A(float input) {
    float output = (b0 * input) + (b1_f * x1_h) + (b2 * x2_h) + (b3 * x3_h)
                 - (a1_f * y1_h) - (a2 * y2_h) - (a3 * y3_h);
    x3_h = x2_h; x2_h = x1_h; x1_h = input;
    y3_h = y2_h; y2_h = y1_h; y1_h = output;
    return output;
}

void loop() {
  int32_t raw_samples[BLOCK_SIZE];
  size_t bytes_read = 0;
  static float dc_offset = 0.0f;
  static float db_suavizado = 30.0f; // Começa no mínimo

  esp_err_t result = i2s_read(I2S_PORT, &raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY);

  if (result == ESP_OK) {
    double sum_squares = 0.0;
    int samples_read = bytes_read / 4;

    for (int i = 0; i < samples_read; i++) {
        float sample_norm = (float)raw_samples[i] / 2147483648.0f;
        dc_offset = (dc_offset * 0.995f) + (sample_norm * 0.005f);
        sample_norm -= dc_offset;
        float sample_filtered = aplicar_filtro_A(sample_norm);
        sum_squares += (sample_filtered * sample_filtered);
    }

    float rms = sqrt(sum_squares / samples_read);
    float db_instantaneo = 20.0f * log10(rms + 1e-9) + OFFSET_DB;
    if (db_instantaneo < 30.0f) db_instantaneo = 30.0f;

    // Filtro de Suavização (Smoothing)
    db_suavizado = (db_suavizado * 0.85f) + (db_instantaneo * 0.15f);

    // --- ATUALIZA A FITA DE LED ---
    atualizar_ledbar_mapa(db_suavizado);

    // Debug Serial
    Serial.print("Min:30 Max:100 Ref_App:40 dB:");
    Serial.println(db_suavizado);
  }
}