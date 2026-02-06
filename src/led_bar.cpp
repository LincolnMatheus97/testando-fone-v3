#include "led_bar.h"

// Instancia o objeto da fita
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void inicializar_ledbar() {
    strip.begin();
    strip.setBrightness(BRIGHTNESS);
    strip.clear();
    strip.show();
}

// Função inteligente que converte dB direto para LEDs acesos
void atualizar_ledbar_mapa(float db_value) {
    // Mapeamento
    int leds_acesos = map((long)db_value, 30, 100, 0, LED_COUNT);

    // Travas de segurança
    if (leds_acesos < 0) leds_acesos = 0;
    if (leds_acesos > LED_COUNT) leds_acesos = LED_COUNT;

    strip.clear(); // Apaga tudo para desenhar o novo estado

    for (int i = 0; i < leds_acesos; i++) {
        // Zona VERDE
        if (i < LEDS_VERDES) {
            strip.setPixelColor(i, strip.Color(0, 255, 0)); // R, G, B
        }
        // Zona AMARELA
        else if (i < (LEDS_VERDES + LEDS_AMARELOS)) {
            strip.setPixelColor(i, strip.Color(255, 200, 0)); 
        }
        // Zona VERMELHA
        else {
            strip.setPixelColor(i, strip.Color(255, 0, 0)); 
        }
    }
    strip.show();
}