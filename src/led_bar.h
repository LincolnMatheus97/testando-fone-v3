#ifndef LED_BAR_H
#define LED_BAR_H

#include <Adafruit_NeoPixel.h>

// Configurações da Fita
#define LED_PIN     7      // Pino de dados 
#define LED_COUNT   12     // Total de LEDs
#define BRIGHTNESS  20     // Brilho (0 a 255)

// Definição das zonas de cor 
#define LEDS_VERDES    4
#define LEDS_AMARELOS  4
#define LEDS_VERMELHOS 4

void inicializar_ledbar();
void atualizar_ledbar_mapa(float db_value);

#endif