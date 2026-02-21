#ifndef NETWORK_WS_H
#define NETWORK_WS_H

#include <WiFi.h>
#include <WebSocketsClient.h>

// --- Configurações da sua Rede ---
#define WIFI_SSID "Larissa Lima"
#define WIFI_PASS "larissalima33840"

// --- Configurações do Servidor (Railway) ---
#define WS_HOST "crossover.proxy.rlwy.net"
#define WS_PORT 52420
#define WS_URL  "/"

extern WebSocketsClient webSocket;

// Função de callback para eventos do WebSocket
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
void inicializar_wifi_e_websocket();

#endif