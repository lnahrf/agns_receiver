#pragma once
#ifndef RECEIVER_H
#define RECEIVER_H
#include <Arduino.h>
#include <esp_now.h>
#include <string>
#include "WiFi.h"
#include "env.h"

#define ESP_INIT_SUCCESS "Initiated ESP"
#define ESP_INIT_FAILURE "Failed to inititate ESP"
#define MOTOR_HIGH "Motor is set to HIGH"
#define MOTOR_LOW "Motor is set to LOW"

const int MOTOR_PIN = 14;

typedef struct state
{
    bool power;
} state;

state incomingState;
unsigned long lastStateTime = 0;

class RECEIVER
{
private:
    static void onMessage(const uint8_t *mac, const uint8_t *incomingData, int len);

public:
    static std::string getMACAddress();
    static void initReceiver();
    static void messageHandler();
    static void powerHandler();
    static void stateTimeHandler();
};

std::string RECEIVER::getMACAddress()
{
    uint8_t address[6];
    esp_efuse_mac_get_default(address);

    char mac[18];

    // Write the address to a buffer
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             address[0], address[1], address[2],
             address[3], address[4], address[5]);

    return std::string(mac);
};

void RECEIVER::initReceiver()
{
    pinMode(MOTOR_PIN, OUTPUT);
    WiFi.mode(WIFI_MODE_STA);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println(ESP_INIT_FAILURE);
    }
    else
    {
        Serial.println(ESP_INIT_SUCCESS);
    }
};

void RECEIVER::onMessage(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&incomingState, incomingData, sizeof(incomingState));

    if (incomingState.power)
    {
        digitalWrite(MOTOR_PIN, HIGH);
        Serial.println(MOTOR_HIGH);
    }

    lastStateTime = millis();
};

void RECEIVER::messageHandler()
{
    esp_now_register_recv_cb(onMessage);
};

void RECEIVER::stateTimeHandler()
{

    if (!incomingState.power)
        return;

    unsigned long currentTime = millis();
    if (currentTime - lastStateTime >= IRRIGATION_SHUTDOWN_THRESHOLD)
    {
        digitalWrite(MOTOR_PIN, LOW);
        lastStateTime = 0;
        incomingState.power = false;
        Serial.println(MOTOR_LOW);
    }
}

#endif