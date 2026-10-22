#include <Arduino.h>
#include "driver/twai.h"

// ESP32 DevKit V1 + SN65HVD230 Pin Definitions
#define CAN_TX_PIN GPIO_NUM_2
#define CAN_RX_PIN GPIO_NUM_15

// Task handles for FreeRTOS multi-core setup
TaskHandle_t canToSerialTaskHandle = NULL;
TaskHandle_t serialToCanTaskHandle = NULL;

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

// --- TASK 1: Handles CAN Bus -> Serial (Pinned to Core 1) ---
void canToSerialTask(void *pvParameters) {
  twai_message_t rx_msg;
  while (true) {
    while (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
      sendGridConnect(rx_msg);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// --- TASK 2: Handles Serial (JMRI) -> CAN Bus (Pinned to Core 0) ---
void serialToCanTask(void *pvParameters) {
  String inputBuffer = "";
  inputBuffer.reserve(96);

  while (true) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r' || c == '\n') continue;

      inputBuffer += c;

      if (c == ';') {
        twai_message_t tx_msg;
        if (parseGridConnect(inputBuffer, tx_msg)) {
          twai_transmit(&tx_msg, pdMS_TO_TICKS(2));
        }
        inputBuffer = "";
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  // Brief delay for power supplies and strapping pin stabilization
  delay(500);

  // Expand Arduino Serial RX/TX buffers to prevent overflow at 460800 baud
  Serial.setRxBufferSize(2048);
  Serial.setTxBufferSize(2048);
  Serial.begin(460800);

  // General config with massive queues to absorb JMRI startup traffic
  twai_general_config_t g_config = {
      .mode = TWAI_MODE_NORMAL,
      .tx_io = CAN_TX_PIN,
      .rx_io = CAN_RX_PIN,
      .clkout_io = TWAI_IO_UNUSED,
      .bus_off_io = TWAI_IO_UNUSED,
      .tx_queue_len = 1024, // 1024 frames towards CAN bus
      .rx_queue_len = 128,  // Large RX queue for layout node bursts
      .alerts_enabled = TWAI_ALERT_NONE,
      .intr_flags = ESP_INTR_FLAG_LEVEL1
  };
  
  // Custom timing configuration for 125 kbps with an optimized 75% sample point
  twai_timing_config_t t_config = {
      .brp = 32,
      .tseg_1 = 14,
      .tseg_2 = 5,
      .sjw = 3,
      .triple_sampling = false
  };

  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK ||
      twai_start() != ESP_OK) {
    while (1) {
      delay(1000);
    }
  }

  // Create Dedicated FreeRTOS Tasks across both CPU cores
  xTaskCreatePinnedToCore(
    canToSerialTask,     
    "CanToSerial",       
    4096,                
    NULL,                
    3,                   
    &canToSerialTaskHandle,
    1                    // Core 1
  );

  xTaskCreatePinnedToCore(
    serialToCanTask,     
    "SerialToCan",       
    4096,                
    NULL,                
    2,                   
    &serialToCanTaskHandle,
    0                    // Core 0
  );
}

void loop() {
  // FreeRTOS tasks handle everything concurrently in the background
  vTaskDelay(pdMS_TO_TICKS(1000));
}
