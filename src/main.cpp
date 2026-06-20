#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>
#include "config.h"

const uint32_t STEP_VALUES[] = { 100, 500, 1000, 5000, 10000 };
const uint8_t  STEP_COUNT    = sizeof(STEP_VALUES) / sizeof(STEP_VALUES[0]);

struct SharedState {
    uint32_t frequency;
    uint32_t deadTimeNs;
    uint8_t  stepIndex;
    Mode     mode;
    bool     outputOn;
};

static SharedState        g_state;
static SemaphoreHandle_t  g_stateMutex;
static EventGroupHandle_t g_inputEvents;

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
static ESP32Encoder encoder;

static hw_timer_t *g_timer = NULL;
static volatile uint8_t g_sigState = 0;
static volatile uint64_t g_halfPeriod;
static volatile uint64_t g_deadTicks;

#define APB_CLK 80000000ULL

static void IRAM_ATTR timerISR() {
    switch (g_sigState) {
        case 0:
            GPIO.out_w1ts = (1 << PIN_OUTPUT_A);
            timerAlarmWrite(g_timer, g_halfPeriod - g_deadTicks, true);
            g_sigState = 1;
            break;
        case 1:
            GPIO.out_w1tc = (1 << PIN_OUTPUT_A);
            timerAlarmWrite(g_timer, g_deadTicks, true);
            g_sigState = 2;
            break;
        case 2:
            GPIO.out_w1ts = (1 << PIN_OUTPUT_B);
            timerAlarmWrite(g_timer, g_halfPeriod - g_deadTicks, true);
            g_sigState = 3;
            break;
        case 3:
            GPIO.out_w1tc = (1 << PIN_OUTPUT_B);
            timerAlarmWrite(g_timer, g_deadTicks, true);
            g_sigState = 0;
            break;
    }
}

static void recalcSignal(uint32_t freq, uint32_t dtNs) {
    g_halfPeriod = APB_CLK / (freq * 2);
    g_deadTicks  = (APB_CLK / 1000000000ULL) * dtNs;
    if (g_deadTicks >= g_halfPeriod / 2)
        g_deadTicks = g_halfPeriod / 4;
}

static void signalEnable() {
    SharedState s;
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    s = g_state;
    xSemaphoreGive(g_stateMutex);

    recalcSignal(s.frequency, s.deadTimeNs);
    g_sigState = 0;
    timerWrite(g_timer, 0);
    timerAlarmEnable(g_timer);
}

static void signalDisable() {
    timerAlarmDisable(g_timer);
    GPIO.out_w1tc = (1 << PIN_OUTPUT_A) | (1 << PIN_OUTPUT_B);
}

static void inputTask(void *pv) {
    int64_t lastCount = 0;
    bool lastBtnHigh = true;
    uint32_t pressStart = 0;
    bool longFired = false;

    for (;;) {
        int64_t count = encoder.getCount();
        int64_t delta = count - lastCount;

        if (delta != 0) {
            lastCount = count;

            SharedState s;
            xSemaphoreTake(g_stateMutex, portMAX_DELAY);
            s = g_state;
            xSemaphoreGive(g_stateMutex);

            bool changed = false;
            switch (s.mode) {
                case MODE_FREQ: {
                    int64_t f = (int64_t)s.frequency + delta * (int64_t)STEP_VALUES[s.stepIndex];
                    if (f < (int64_t)FREQ_MIN) f = FREQ_MIN;
                    if (f > (int64_t)FREQ_MAX) f = FREQ_MAX;
                    if ((uint32_t)f != s.frequency) {
                        s.frequency = (uint32_t)f;
                        changed = true;
                    }
                    break;
                }
                case MODE_DEADTIME: {
                    int64_t dt = (int64_t)s.deadTimeNs + delta * 100;
                    if (dt < (int64_t)DEAD_TIME_MIN_NS) dt = DEAD_TIME_MIN_NS;
                    if (dt > (int64_t)DEAD_TIME_MAX_NS) dt = DEAD_TIME_MAX_NS;
                    if ((uint32_t)dt != s.deadTimeNs) {
                        s.deadTimeNs = (uint32_t)dt;
                        changed = true;
                    }
                    break;
                }
                case MODE_STEP: {
                    uint8_t idx = s.stepIndex;
                    if (delta > 0) idx = (idx + 1) % STEP_COUNT;
                    else          idx = (idx + STEP_COUNT - 1) % STEP_COUNT;
                    if (idx != s.stepIndex) {
                        s.stepIndex = idx;
                        changed = true;
                    }
                    break;
                }
                default: break;
            }

            if (changed) {
                xSemaphoreTake(g_stateMutex, portMAX_DELAY);
                g_state = s;
                xSemaphoreGive(g_stateMutex);

                if (s.outputOn) recalcSignal(s.frequency, s.deadTimeNs);
            }
        }

        bool btnHigh = digitalRead(PIN_ENCODER_BTN);
        if (btnHigh != lastBtnHigh) {
            vTaskDelay(pdMS_TO_TICKS(15));
            btnHigh = digitalRead(PIN_ENCODER_BTN);
            if (btnHigh == lastBtnHigh) continue;

            if (!btnHigh) {
                pressStart = millis();
                longFired = false;
            } else {
                uint32_t dur = millis() - pressStart;
                if (dur >= LONG_PRESS_MS) {
                    xEventGroupSetBits(g_inputEvents, EVT_BTN_LONG);
                } else if (dur >= DEBOUNCE_MS) {
                    xEventGroupSetBits(g_inputEvents, EVT_BTN_SHORT);
                }
            }
            lastBtnHigh = btnHigh;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void displayTask(void *pv) {
    for (;;) {
        SharedState s;
        xSemaphoreTake(g_stateMutex, portMAX_DELAY);
        s = g_state;
        xSemaphoreGive(g_stateMutex);

        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);

        display.setTextSize(1);
        display.setCursor(20, 0);
        display.println("FREQUENCY GEN");
        display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

        display.setTextSize(2);
        display.setCursor(0, 14);
        display.print("F:");
        display.setTextSize(1);
        display.setCursor(0, 30);
        if (s.frequency >= 1000000)
            display.printf("%d.%03d MHz", s.frequency / 1000000, (s.frequency % 1000000) / 1000);
        else
            display.printf("%d.%03d kHz", s.frequency / 1000, s.frequency % 1000);

        display.drawLine(0, 40, 127, 40, SSD1306_WHITE);

        display.setTextSize(1);
        display.setCursor(0, 44);
        display.printf("DT:%lu ns", s.deadTimeNs);

        display.setCursor(0, 54);
        uint32_t step = STEP_VALUES[s.stepIndex];
        if (step >= 1000) display.printf("Step:%lu kHz", step / 1000);
        else              display.printf("Step:%lu Hz", step);

        display.drawLine(75, 44, 75, 63, SSD1306_WHITE);

        display.setCursor(80, 44);
        display.printf("Mode:");
        display.setCursor(80, 54);
        static const char *modeNames[] = { "Freq ", "DT   ", "Step " };
        display.print(modeNames[s.mode]);

        display.display();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void signalControlTask(void *pv) {
    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(g_inputEvents,
            EVT_BTN_SHORT | EVT_BTN_LONG, pdTRUE, pdFALSE, portMAX_DELAY);

        SharedState s;
        xSemaphoreTake(g_stateMutex, portMAX_DELAY);
        s = g_state;

        if (bits & EVT_BTN_LONG) {
            s.mode = (Mode)((s.mode + 1) % MODE_COUNT);
        }

        if (bits & EVT_BTN_SHORT) {
            s.outputOn = !s.outputOn;
            if (s.outputOn) signalEnable();
            else            signalDisable();
        }

        g_state = s;
        xSemaphoreGive(g_stateMutex);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Freq Gen - FreeRTOS");

    g_stateMutex    = xSemaphoreCreateMutex();
    g_inputEvents   = xEventGroupCreate();

    g_state.frequency  = FREQ_DEFAULT;
    g_state.deadTimeNs = DEAD_TIME_DEFAULT;
    g_state.stepIndex  = 2;
    g_state.mode       = MODE_FREQ;
    g_state.outputOn   = false;

    pinMode(PIN_OUTPUT_A, OUTPUT);
    pinMode(PIN_OUTPUT_B, OUTPUT);
    digitalWrite(PIN_OUTPUT_A, LOW);
    digitalWrite(PIN_OUTPUT_B, LOW);

    Wire.begin(OLED_SDA, OLED_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("SSD1306 fail");
        while (1) vTaskDelay(portMAX_DELAY);
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(20, 28);
    display.println("Starting...");
    display.display();

    ESP32Encoder::useInternalWeakPullResistors = UP;
    encoder.attachHalfQuad(PIN_ENCODER_A, PIN_ENCODER_B);
    encoder.setCount(0);

    pinMode(PIN_ENCODER_BTN, INPUT_PULLUP);

    g_timer = timerBegin(0, 1, true);
    timerAttachInterrupt(g_timer, &timerISR, true);
    timerAlarmWrite(g_timer, 400, true);

    recalcSignal(g_state.frequency, g_state.deadTimeNs);

    xTaskCreatePinnedToCore(inputTask,    "input",   TASK_STACK_INPUT, NULL, TASK_PRIO_INPUT, NULL, 0);
    xTaskCreatePinnedToCore(displayTask,   "display", TASK_STACK_UI,    NULL, TASK_PRIO_UI,    NULL, 1);
    xTaskCreatePinnedToCore(signalControlTask, "sig",  TASK_STACK_SIG,  NULL, TASK_PRIO_SIG,    NULL, 0);

    Serial.printf("Ready: %lu Hz, DT %lu ns, step %lu\n",
        g_state.frequency, g_state.deadTimeNs, STEP_VALUES[g_state.stepIndex]);

    vTaskDelete(NULL);
}

void loop() {
    vTaskDelete(NULL);
}
