#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>
#include "led_bar.h"
#include "network_ws.h" 

// --- A MÁGICA DO TINYML AQUI ---
// (Lembre-se de verificar se o nome do arquivo abaixo bate com a sua biblioteca)
#include <se-mapa-urban_inferencing.h> 

// --- Configurações de Hardware (Microfone) ---
#define I2S_SCK 4
#define I2S_WS  5
#define I2S_SD  6
#define I2S_PORT I2S_NUM_0

// --- Áudio ---
#define SAMPLE_RATE 44100
#define BLOCK_SIZE 1024
#define OFFSET_DB  98.0f 

// --- Variáveis Globais (Compartilhadas entre os dois núcleos) ---
volatile float global_db_suavizado = 30.0f;
char global_classe_ia[30] = "Aguardando IA..."; // Guardará o texto: "transito", "ruido_fundo", etc.

// --- Buffers da Inteligência Artificial ---
float ml_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]; // Tamanho exato que a IA espera (1 segundo a 16kHz)
volatile int ml_index = 0;
volatile bool ml_pronto = false; // Avisa o Núcleo 0 que pode rodar a IA

// --- Coeficientes do Filtro A ---
float b0 = 0.169996, b1_f = 0.280415, b2 = -0.11204, b3 = -0.131557; 
float a1_f = -2.12979, a2 = 1.63772, a3 = -0.463633;                 
float x1_h = 0, x2_h = 0, x3_h = 0, y1_h = 0, y2_h = 0, y3_h = 0;

float aplicar_filtro_A(float input) {
    float output = (b0 * input) + (b1_f * x1_h) + (b2 * x2_h) + (b3 * x3_h)
                 - (a1_f * y1_h) - (a2 * y2_h) - (a3 * y3_h);
    x3_h = x2_h; x2_h = x1_h; x1_h = input;
    y3_h = y2_h; y2_h = y1_h; y1_h = output;
    return output;
}

// ==========================================
// FUNÇÃO OBRIGATÓRIA DO EDGE IMPULSE
// ==========================================
// Pega os dados do nosso ml_buffer e entrega para a IA empacotado
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, ml_buffer + offset, length * sizeof(float));
    return 0;
}

// ==========================================
// TAREFA DO NÚCLEO 0: REDE + INTELIGÊNCIA ARTIFICIAL
// ==========================================
void taskDeRedeEIA(void * pvParameters) {
    inicializar_wifi_e_websocket();
    unsigned long ultimo_envio = 0;
    const unsigned long INTERVALO_ENVIO_MS = 200; // 200ms para dashboard responsivo

    for(;;) {
        webSocket.loop(); 

        // 1. RODA A INTELIGÊNCIA ARTIFICIAL (SE O ÁUDIO ESTIVER PRONTO)
        if (ml_pronto) {
            signal_t signal;
            signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
            signal.get_data = &raw_feature_get_data;

            ei_impulse_result_t result = { 0 };
            
            // run_classifier faz a matemática pesada. Demora uns ~100ms no ESP32-S3
            EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);

            if (r == EI_IMPULSE_OK) {
                // Acha a classe vencedora (com maior probabilidade de acerto)
                float maior_prob = 0.0;
                int melhor_indice = 0;
                
                for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                    if (result.classification[i].value > maior_prob) {
                        maior_prob = result.classification[i].value;
                        melhor_indice = i;
                    }
                }
                
                // Atualiza a interface apenas se a IA tiver mais de 60% de certeza
                if (maior_prob > 0.60) {
                    strncpy(global_classe_ia, result.classification[melhor_indice].label, sizeof(global_classe_ia));
                    Serial.printf("IA Detectou: %s (%.0f%%)\\n", global_classe_ia, maior_prob * 100);
                }
            }
            
            // Limpa o sinalizador para o Núcleo 1 gravar o próximo segundo de áudio
            ml_index = 0;
            ml_pronto = false; 
        }

        // 2. ENVIA OS DADOS PARA A WEB
        if (millis() - ultimo_envio > INTERVALO_ENVIO_MS) {
            if (WiFi.status() == WL_CONNECTED) {
                char json[128];
                float db_atual = global_db_suavizado; 
                
                // Aqui substituímos a antiga classificação simples ("Moderado", "Alto")
                // pela resposta da nossa Inteligência Artificial!
                snprintf(json, sizeof(json), "{\"niveldB\": %.2f, \"nivelSom\": \"%s\"}", 
                         db_atual, global_classe_ia);
                
                webSocket.sendTXT(json);
            }
            ultimo_envio = millis();
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS); // Watchdog respira
    }
}

// ==========================================
// TAREFA DO NÚCLEO 1 (Loop Principal): ÁUDIO, FILTRO A E LEDS
// ==========================================
void setup() {
  delay(2000);
  Serial.begin(115200);
  
  inicializar_ledbar();
  
  // Inicia a tarefa mista (Rede + IA) no Núcleo 0
  xTaskCreatePinnedToCore(taskDeRedeEIA, "TaskRedeIA", 16384, NULL, 1, NULL, 0);

  // Configura I2S (Fica nativamente no Core 1)
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

  i2s_pin_config_t pin_config = { .bck_io_num = I2S_SCK, .ws_io_num = I2S_WS, .data_out_num = I2S_PIN_NO_CHANGE, .data_in_num = I2S_SD };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void loop() {
  int32_t raw_samples[BLOCK_SIZE];
  size_t bytes_read = 0;
  static float dc_offset = 0.0f;
  static float db_suavizado = 30.0f;

  esp_err_t result = i2s_read(I2S_PORT, &raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY);

  if (result == ESP_OK) {
    double sum_squares = 0.0;
    int samples_read = bytes_read / 4;

    for (int i = 0; i < samples_read; i++) {
        float sample_norm = (float)raw_samples[i] / 2147483648.0f;
        
        // Remove offset DC
        dc_offset = (dc_offset * 0.995f) + (sample_norm * 0.005f);
        sample_norm -= dc_offset;
        
        // --- PREPARAÇÃO PARA A IA (RESAMPLING PARA 16KHz) ---
        // Só guardamos a amostra se a IA estiver livre. 
        if (!ml_pronto) {
            static float phase = 0.0f;
            phase += (16000.0f / 44100.0f); // Passo fracionado
            
            if (phase >= 1.0f) {
                phase -= 1.0f;
                if (ml_index < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                    // Damos um "Ganho" de 8x a 15x para a IA escutar direito o mundo real
                    float ganho_ia = 10.0f; // Ajuste esse valor! Tente 8.0, 10.0 ou 15.0
                    float amostra_amplificada = sample_norm * 32767.0f * ganho_ia;

                    // Trava de segurança para não distorcer o áudio digitalmente (Clipping)
                    if (amostra_amplificada > 32767.0f) amostra_amplificada = 32767.0f;
                    if (amostra_amplificada < -32768.0f) amostra_amplificada = -32768.0f;

                    ml_buffer[ml_index] = amostra_amplificada; 
                    ml_index++;
                }
                if (ml_index >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                    ml_pronto = true; // Avisa o Núcleo 0: "Toma 1 segundo de áudio, pode processar!"
                }
            }
        }

        // --- FILTRO DE DECIBÉIS ---
        float sample_filtered = aplicar_filtro_A(sample_norm);
        sum_squares += (sample_filtered * sample_filtered);
    }

    // Calcula e suaviza os decibéis
    float rms = sqrt(sum_squares / samples_read);
    float db_instantaneo = 20.0f * log10(rms + 1e-9) + OFFSET_DB;
    if (db_instantaneo < 30.0f) db_instantaneo = 30.0f;

    db_suavizado = (db_suavizado * 0.85f) + (db_instantaneo * 0.15f);
    global_db_suavizado = db_suavizado; // Atualiza para o WebSocket ler

    // Atualiza a fita física de LEDs
    atualizar_ledbar_mapa(db_suavizado);
  }
}