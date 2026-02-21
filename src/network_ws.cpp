#include "network_ws.h"

WebSocketsClient webSocket;

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("[WS] Desconectado do Servidor!");
            break;
        case WStype_CONNECTED:
            Serial.printf("[WS] Conectado a url: %s\n", payload);
            break;
        case WStype_ERROR:
            Serial.println("[WS] Erro na conexão!");
            break;
        default:
            break;
    }
}

void inicializar_wifi_e_websocket() {
    Serial.print("[WiFi] Conectando a ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    // Tenta conectar, mas não trava o sistema inteiro se falhar
    int tentativas = 0;
    while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
        delay(500);
        Serial.print(".");
        tentativas++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[WiFi] Conectado! IP: ");
        Serial.println(WiFi.localIP());

        // Configura e inicia o WebSocket
        webSocket.begin(WS_HOST, WS_PORT, WS_URL);
        webSocket.onEvent(webSocketEvent);
        webSocket.setReconnectInterval(5000); // Tenta reconectar a cada 5s se cair
    } else {
        Serial.println("\n[WiFi] Falha ao conectar. Trabalhando offline.");
    }
}