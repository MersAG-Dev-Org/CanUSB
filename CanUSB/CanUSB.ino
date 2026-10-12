#include <Arduino.h>
#include "driver/twai.h"

// ESP32 DevKit V1 + SN65HVD230 Pin Definitions
#define CAN_TX_PIN GPIO_NUM_2
#define CAN_RX_PIN GPIO_NUM_15

String inputBuffer = "";

void sendGridConnect(const twai_message_t& msg) {
  Serial.print(':');
  if (msg.extd) {
    Serial.print('X');
    char idStr[9];
    snprintf(idStr, sizeof(idStr), "%08X", (unsigned int)msg.identifier);
    Serial.print(idStr);
  } else {
    Serial.print('S');
    char idStr[4];
    snprintf(idStr, sizeof(idStr), "%03X", (unsigned int)msg.identifier);
    Serial.print(idStr);
  }
  Serial.print(msg.rtr ? 'R' : 'N');
  for (int i = 0; i < msg.data_length_code; i++) {
    char byteStr[3];
    snprintf(byteStr, sizeof(byteStr), "%02X", msg.data[i]);
    Serial.print(byteStr);
  }
  Serial.print(";\r\n");
}

bool parseGridConnect(const String& str, twai_message_t& msg) {
  int len = str.length();
  if (len < 4 || str.charAt(0) != ':' || str.charAt(len - 1) != ';') return false;

  memset(&msg, 0, sizeof(msg));
  char frameType = str.charAt(1);
  msg.extd = (frameType == 'X' || frameType == 'x') ? 1 : 0;

  int flagPos = str.indexOf('N');
  if (flagPos == -1) flagPos = str.indexOf('n');
  if (flagPos == -1) {
    flagPos = str.indexOf('R');
    if (flagPos == -1) flagPos = str.indexOf('r');
    if (flagPos != -1) msg.rtr = 1;
  }
  if (flagPos == -1) return false;

  String idStr = str.substring(2, flagPos);
  msg.identifier = strtoul(idStr.c_str(), NULL, 16);

  String dataStr = str.substring(flagPos + 1, len - 1);
  msg.data_length_code = dataStr.length() / 2;
  if (msg.data_length_code > 8) msg.data_length_code = 8;

  for (int i = 0; i < msg.data_length_code; i++) {
    String byteStr = dataStr.substring(i * 2, i * 2 + 2);
    msg.data[i] = (uint8_t)strtoul(byteStr.c_str(), NULL, 16);
  }
  return true;
}

void setup() {
  // Brief delay to allow power supplies and strapping pins to stabilize
  delay(500);

  Serial.begin(460800);
  inputBuffer.reserve(64);

  // CAN @ 125 kbit/s
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK ||
      twai_start() != ESP_OK) {
    // If initialization fails, loop here
    while (1) {
      delay(1000);
    }
  }
}

void loop() {
  // CAN → Serial
  twai_message_t rx_msg;
  if (twai_receive(&rx_msg, 0) == ESP_OK) {
    sendGridConnect(rx_msg);
  }

  // Serial → CAN
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;

    inputBuffer += c;

    if (c == ';') {
      twai_message_t tx_msg;
      if (parseGridConnect(inputBuffer, tx_msg)) {
        twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
      }
      inputBuffer = "";
    }
  }
}