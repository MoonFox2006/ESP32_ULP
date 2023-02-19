#include <Arduino.h>
#if defined(CONFIG_IDF_TARGET_ESP32)
#include "esp32/ulp.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#include "esp32s2/ulp.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#include "esp32s3/ulp.h"
#else
#error "Unsupported CPU!"
#endif

#if defined(CONFIG_IDF_TARGET_ESP32)
enum { CNT_LOW, CNT_HIGH, RTC_MAX };
#else
enum { CNT, RTC_MAX };
#endif

static esp_err_t ulp_init() {
  enum { LOOP, OVER, WAIT, REPEAT };

  constexpr uint16_t DATA_OFFSET = 0;
  constexpr uint16_t CODE_OFFSET = RTC_MAX;

  const ulp_insn_t program[] = {
    I_MOVI(R2, DATA_OFFSET),

    M_LABEL(LOOP),
#if defined(CONFIG_IDF_TARGET_ESP32)
    I_LD(R0, R2, CNT_LOW), // R0 = CNT_LOW
#else
    I_LDL(R0, R2, CNT), // R0 = CNT[15:0]
#endif
    I_ADDI(R0, R0, 1), // R0++
#if defined(CONFIG_IDF_TARGET_ESP32)
    I_ST(R0, R2, CNT_LOW), // CNT_LOW = R0
#else
    I_STL(R0, R2, CNT), // CNT[15:0] = R0
#endif
    M_BXF(OVER),
    M_BX(WAIT),

    M_LABEL(OVER),
#if defined(CONFIG_IDF_TARGET_ESP32)
    I_LD(R0, R2, CNT_HIGH), // R0 = CNT_HIGH
#else
    I_LDH(R0, R2, CNT), // R0 = CNT[31:16]
#endif
    I_ADDI(R0, R0, 1), // R0++
#if defined(CONFIG_IDF_TARGET_ESP32)
    I_ST(R0, R2, CNT_HIGH), // CNT_HIGH = R0
#else
    I_STH(R0, R2, CNT), // CNT[31:16] = R0
#endif

    M_LABEL(WAIT),
/*
    I_STAGE_RST(),

    M_LABEL(REPEAT),
    I_DELAY(20000),
    I_STAGE_INC(1),
    M_BSLE(REPEAT, 40),
*/
    M_BX(LOOP),
  };

  size_t size = sizeof(program) / sizeof(ulp_insn_t);
  esp_err_t err;

#if defined(CONFIG_IDF_TARGET_ESP32)
  RTC_SLOW_MEM[CNT_LOW] = 0;
  RTC_SLOW_MEM[CNT_HIGH] = 0;
#else
  RTC_SLOW_MEM[CNT] = 0;
#endif
  err = ulp_process_macros_and_load(CODE_OFFSET, program, &size);
  if (err == ESP_OK) {
    err = ulp_run(CODE_OFFSET);
  }
  return err;
}

void setup() {
  Serial.begin(115200);

  esp_err_t err = ulp_init();

  if (err != ESP_OK) {
    Serial.printf("ULP init error 0x%X!\n", err);
    Serial.flush();
    esp_deep_sleep_start();
  }
}

void loop() {
#if defined(CONFIG_IDF_TARGET_ESP32)
  uint32_t cnt = (RTC_SLOW_MEM[CNT_HIGH] << 16) | (RTC_SLOW_MEM[CNT_LOW] & 0xFFFF);

  Serial.printf("\r%-10u", cnt);
#else
  Serial.printf("\r%-10u", RTC_SLOW_MEM[CNT]);
#endif
  delay(100);
}
