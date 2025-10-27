#include "StatusTransmitter.h"
#include <esp_now.h>
#include <WiFi.h>

#define LED_PIN 8

uint8_t receiverMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfoSt;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Status send: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "Failed");
}

void initStatusTransmitter() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    memcpy(peerInfoSt.peer_addr, receiverMAC, 6);
    peerInfoSt.channel = 1;
    peerInfoSt.encrypt = false;
    if (esp_now_add_peer(&peerInfoSt) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
    delay(2000);
    Serial.println("Status transmitter ready");
}

void sendStatusMessage(String message) {
    unsigned long t = millis();
    String msgWithTime = String(t) + " " + message;
    uint8_t messageData[250];
    int msgLen = min((int)msgWithTime.length(), 249);
    strncpy((char*)messageData, msgWithTime.c_str(), msgLen);
    messageData[msgLen] = '\0';
    digitalWrite(LED_PIN, LOW);
    esp_now_send(receiverMAC, messageData, msgLen + 1);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
}

