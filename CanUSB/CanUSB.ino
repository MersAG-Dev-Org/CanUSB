#include <Arduino.h>
#include "driver/twai.h"

// ESP32 DevKit V1 + SN65HVD230 Pin Definitions
#define CAN_TX_PIN GPIO_NUM_2
#define CAN_RX_PIN GPIO_NUM_15

// Task handles for FreeRTOS multi-core setup
TaskHandle_t canToSerialTaskHandle = NULL;
TaskHandle_t serialToCanTaskHandle = NULL;

// Fast lookup table for hex-to-ascii conversions
const char HEX_LOOKUP[] = "0123456789ABCDEF";

// Fast byte parsing utility
inline uint8_t parseHexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;
}

// Highly optimized formatting using direct array index operations instead of snprintf
void sendGridConnect(const twai_message_t& msg) {
  char buf[32];
  int idx = 0;
  
  buf[idx++] = ':';
  if (msg.extd) {
    buf[idx++] = 'X';
    buf[idx++] = HEX_LOOKUP[(msg.identifier >> 28) & 0x0F];
    buf[idx++] = HEX_LOOKUP[(msg.identifier >> 24) & 0x0F];
    buf[idx++] = HEX_LOOKUP[(msg.identifier >> 20) & 0x0F];
    buf[idx++] = HEX_LOOKUP[(msg.identifier >> 16) & 0x0F];
    buf[idx++] = HEX_LOOKUP[(msg.identifier >> 12) & 0x0F];
    buf[idx++] = HEX_LOOKUP[(msg.identifier >> 8)  & 0x0F];
    buf[idx++] = HEX_LOOKUP[(msg.identifier >> 4)  & 0x0F];
    buf[idx++] = HEX_LOOKUP[msg.identifier         & 0x0F];
  } else {
    buf[idx++] = 'S';
    buf[idx++] = HEX_LOOKUP[(msg.identifier >> 8)  & 0x0F];
    buf[idx++] = HEX_LOOKUP[(msg.identifier >> 4)  & 0x0F];
    buf[idx++] = HEX_LOOKUP[msg.identifier         & 0x0F];
  }
  
  buf[idx++] = msg.rtr ? 'R' : 'N';
  
  for (int i = 0; i < msg.data_length_code; i++) {
    buf[idx++] = HEX_LOOKUP[(msg.data[i] >> 4) & 0x0F];
    buf[idx++] = HEX_LOOKUP[msg.data[i]        & 0x0F];
  }
  
  buf[idx++] = ';';
  buf[idx++] = '\r';
  buf[idx++] = '\n';
  
  // Single atomic write operation down to the hardware core
  Serial.write((const uint8_t*)buf, idx);
}

// Zero-allocation Gridconnect string parser using basic array indexes
bool parseGridConnectFast(const char* buf, int len, twai_message_t& msg) {
  if (len < 5 || buf[0] != ':' || buf[len - 1] != ';') return false;

  memset(&msg, 0, sizeof(msg));
  char frameType = buf[1];
  msg.extd = (frameType == 'X' || frameType == 'x') ? 1 : 0;

  int idLen = msg.extd ? 8 : 3;
  if (len < 2 + idLen + 1) return false;

  // Parse identifier directly out of the indices
  uint32_t id = 0;
  for (int i = 0; i < idLen; i++) {
    id = (id << 4) | parseHexNibble(buf[2 + i]);
  }
  msg.identifier = id;

  char typeFlag = buf[2 + idLen];
  if (typeFlag == 'R' || typeFlag == 'r') {
    msg.rtr = 1;
  }

  // Parse remaining payload data bytes
  int dataStartIdx = 2 + idLen + 1;
  int dataLenChars = (len - 1) - dataStartIdx;
  msg.data_length_code = dataLenChars / 2;
  if (msg.data_length_code > 8) msg.data_length_code = 8;

  for (int i = 0; i < msg.data_length_code; i++) {
    uint8_t high = parseHexNibble(buf[dataStartIdx + (i * 2)]);
    uint8_t low  = parseHexNibble(buf[dataStartIdx + (i * 2) + 1]);
    msg.data[i] = (high << 4) | low;
  }
  return true;
}

// --- TASK 1: Handles CAN Bus -> Serial (Pinned to Core 1) ---
void canToSerialTask(void *pvParameters) {
  twai_message_t rx_msg;
  while (true) {
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
      sendGridConnect(rx_msg);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// --- TASK 2: Handles Serial (JMRI) -> CAN Bus (Pinned to Core 0) ---
void serialToCanTask(void *pvParameters) {
  static char serialBuffer[64];
  int bufIndex = 0;

  while (true) {
    int availableBytes = Serial.available();
    while (availableBytes > 0) {
      char c = (char)Serial.read();
      availableBytes--;

      if (c == '\r' || c == '\n') continue;

      if (bufIndex < (int)sizeof(serialBuffer) - 1) {
        serialBuffer[bufIndex++] = c;
      }

      if (c == ';') {
        twai_message_t tx_msg;
        if (parseGridConnectFast(serialBuffer, bufIndex, tx_msg)) {
          twai_transmit(&tx_msg, 0); 
        }
        bufIndex = 0; 
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  delay(500);

  // Allocate massive RAM pipelines to absorb PC/JMRI startup bursts
  Serial.setRxBufferSize(4096);
  Serial.setTxBufferSize(4096);
  Serial.begin(460800);

  // General config restored with massive queues to prevent startup node drops
  twai_general_config_t g_config = {
      .mode = TWAI_MODE_NORMAL,
      .tx_io = CAN_TX_PIN,
      .rx_io = CAN_RX_PIN,
      .clkout_io = TWAI_IO_UNUSED,
      .bus_off_io = TWAI_IO_UNUSED,
      .tx_queue_len = 2048, // Restored massive TX queue for startup flood
      .rx_queue_len = 512,  // Restored massive RX queue for node response wave
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

  xTaskCreatePinnedToCore(
    canToSerialTask,     
    "CanToSerial",       
    3072,                
    NULL,                
    3,                   
    &canToSerialTaskHandle,
    1                    // Core 1
  );

  xTaskCreatePinnedToCore(
    serialToCanTask,     
    "SerialToCan",       
    3072,                
    NULL,                
    2,                   
    &serialToCanTaskHandle,
    0                    // Core 0
  );
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
