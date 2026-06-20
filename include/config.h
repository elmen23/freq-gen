#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define PIN_ENCODER_A     18
#define PIN_ENCODER_B     19
#define PIN_ENCODER_BTN   5

#define PIN_OUTPUT_A      25
#define PIN_OUTPUT_B      26

#define OLED_SDA          21
#define OLED_SCL          22
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     64
#define OLED_ADDR         0x3C

#define FREQ_MIN          10000UL
#define FREQ_MAX          300000UL
#define FREQ_DEFAULT      100000UL
#define DEAD_TIME_MIN_NS  100UL
#define DEAD_TIME_MAX_NS  5000UL
#define DEAD_TIME_DEFAULT 500UL

#define DEBOUNCE_MS       200
#define LONG_PRESS_MS     500

#define TASK_STACK_UI     4096
#define TASK_STACK_INPUT  2048
#define TASK_STACK_SIG    2048

#define TASK_PRIO_UI      2
#define TASK_PRIO_INPUT   3
#define TASK_PRIO_SIG     5

#define EVT_BTN_SHORT     (1 << 0)
#define EVT_BTN_LONG      (1 << 1)

enum Mode : uint8_t { MODE_FREQ = 0, MODE_DEADTIME, MODE_STEP, MODE_COUNT };

extern const uint32_t STEP_VALUES[];
extern const uint8_t  STEP_COUNT;

#endif
