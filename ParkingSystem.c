#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <softTone.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "OutputCtrl.h"

// ================= LCD (I2C 16x2) =================
#define LCD_ADDR 0x27
#define RS 0x01
#define EN 0x04
#define BL 0x08
static int fd;

static void lcd_write_byte(int data) { wiringPiI2CWrite(fd, data); }

static void lcd_pulse(int data) {
    lcd_write_byte(data | EN);
    delayMicroseconds(1);
    lcd_write_byte(data & ~EN);
    delayMicroseconds(50);
}

static void lcd_write4(int nibble, int mode_rs) {
    int data = 0;
    if (mode_rs) data |= RS;
    data |= BL;
    data |= (nibble & 0x0F) << 4;
    lcd_write_byte(data);
    lcd_pulse(data);
}

static void lcd_send(int value, int mode_rs) {
    lcd_write4((value >> 4) & 0x0F, mode_rs);
    lcd_write4(value & 0x0F, mode_rs);
}

static void lcd_cmd(int c) {
    lcd_send(c, 0);
    if (c == 0x01 || c == 0x02) delay(2);
}

static void lcd_data(char c) { lcd_send(c, 1); }

static void lcd_init(void) {
    delay(50);
    lcd_write4(0x03, 0); delay(5);
    lcd_write4(0x03, 0); delayMicroseconds(150);
    lcd_write4(0x03, 0); delayMicroseconds(150);
    lcd_write4(0x02, 0); delayMicroseconds(150);
    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
}

static void lcd_set_cursor(int row, int col) {
    int addr = (row == 0) ? 0x00 : 0x40;
    lcd_cmd(0x80 | (addr + col));
}
static void lcd_print(const char *s) { while (*s) lcd_data(*s++); }
static void lcd_clear(void) { lcd_cmd(0x01); }

// ================= Ultrasonic =================
#define UC_TRIG_PIN 12
#define UC_ECHO_PIN 6

// ================= PIR / IR Sensor =================
#define IR_PIN 9
#define IR_ACTIVE 0
#define IR_HOLD_MS 5000

// ================= Buzzer =================
#define BUZZER_GPIO 21
#define BUZZ_FREQ   2000

// ================= Distance thresholds =================
#define SAFE_DIST     50.0f
#define CAUTION_DIST  20.0f

// ================= RGB LED =================
static void led_all_off(void) {
    OC_WritePin(OC_PIN_EXTLED0, 0); // Red
    OC_WritePin(OC_PIN_EXTLED1, 0); // Yellow
    OC_WritePin(OC_PIN_EXTLED2, 0); // Green
}
static void led_safe(void) {
    led_all_off();
    OC_WritePin(OC_PIN_EXTLED0, 1);
}
static void led_caution(void) {
    led_all_off();
    OC_WritePin(OC_PIN_EXTLED1, 1);
}
static void led_warning_on(void) {
    led_all_off();
    OC_WritePin(OC_PIN_EXTLED2, 1);
}

// ================= TFT Image (fbi) =================
// 상태가 바뀔 때만 이미지 교체 (프로세스 폭주 방지)
static void show_png_once(const char *png) {
    char cmd[256];
    // 1) 기존 fbi 모두 종료 (중복 실행 방지)
    system("sudo killall -q fbi > /dev/null 2>&1");
    // 2) 새 이미지 표시
    snprintf(cmd, sizeof(cmd), "sudo fbi -T 1 -d /dev/fb0 -noverbose -a %s > /dev/null 2>&1", png);
    system(cmd);
}

// ================= State enum =================
typedef enum {
    ST_INIT = 0,
    ST_SAFE,
    ST_CAUTION,
    ST_WARNING,
    ST_DANGER
} State;

int main(void)
{
    float dist = 0.0f;
    unsigned int s = 0, e = 0;

    static float last_valid = 0.0f;
    static float ema = 0.0f;
    const float alpha = 0.25f;

    static int blink = 0;
    static int danger_step = 0;

    unsigned int ir_start_ms = 0;
    static int ir_prev = 1;
    static int danger_printed = 0;

    State cur_state = ST_INIT;
    State prev_state = -1;

    if (wiringPiSetupGpio() == -1) return 1;

    fd = wiringPiI2CSetup(LCD_ADDR);
    if (fd < 0) return 1;
    lcd_init();

    OC_Init();
    OC_SelectIO(1);
    led_all_off();

    if (softToneCreate(BUZZER_GPIO) != 0) return 1;
    softToneWrite(BUZZER_GPIO, 0);

    pinMode(UC_TRIG_PIN, OUTPUT);
    pinMode(UC_ECHO_PIN, INPUT);

    pinMode(IR_PIN, INPUT);
    pullUpDnControl(IR_PIN, PUD_UP);

    printf("=== Parking System Start ===\n");
    printf("[INFO] PIR hold time = %d ms\n", IR_HOLD_MS);

    // 시작 화면: TFT에 00.png 1회
    show_png_once("00.png");

    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_print("Parking System");
    lcd_set_cursor(1,0);
    lcd_print("Ready");
    delay(800);

    while (1) {

        // ================= PIR CHECK =================
        int ir = digitalRead(IR_PIN);

        if (ir == IR_ACTIVE && ir_prev != IR_ACTIVE) {
            printf("[PIR] Motion detected (start)\n");
            ir_start_ms = millis();
            danger_printed = 0;
        }

        if (ir != IR_ACTIVE && ir_prev == IR_ACTIVE) {
            printf("[PIR] Motion cleared\n");
            ir_start_ms = 0;
            danger_printed = 0;
        }

        // 5초 이상 감지면 DANGER
        if (ir_start_ms != 0 && (millis() - ir_start_ms >= IR_HOLD_MS)) {

            cur_state = ST_DANGER;

            // 상태 바뀌면 TFT 이미지 교체
            if (cur_state != prev_state) {
                show_png_once("44.png");
                prev_state = cur_state;
            }

            if (!danger_printed) {
                printf("[PIR] DANGER! Human detected for %d ms\n", IR_HOLD_MS);
                danger_printed = 1;
            }

            lcd_clear();
            lcd_set_cursor(0,0);
            lcd_print("HUMAN");
            lcd_set_cursor(1,0);
            lcd_print("DETECTED!!");

            // LED: 빨-노-초 순환
            led_all_off();
            if (danger_step == 0)      OC_WritePin(OC_PIN_EXTLED0, 1); // Red
            else if (danger_step == 1) OC_WritePin(OC_PIN_EXTLED1, 1); // Yellow
            else                       OC_WritePin(OC_PIN_EXTLED2, 1); // Green
            danger_step = (danger_step + 1) % 3;

            // 부저: 삐삐
            softToneWrite(BUZZER_GPIO, BUZZ_FREQ);
            delay(120);
            softToneWrite(BUZZER_GPIO, 0);

            delay(200);
            ir_prev = ir;
            continue;   // 초음파 주차 로직 차단
        }

        ir_prev = ir;

        // ================= Ultrasonic =================
        digitalWrite(UC_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(UC_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(UC_TRIG_PIN, LOW);

        unsigned int t0 = micros();
        // Echo 신호 미수신 시 무한 루프 방지
        // 일정 시간(30ms) 초과 시 마지막 정상 거리값 유지
        while (digitalRead(UC_ECHO_PIN) == 0) {
            // Timeout 발생 시 이전 유효값 사용
            if (micros() - t0 > 30000) goto use_prev;
            s = micros();
        }

        while (digitalRead(UC_ECHO_PIN) == 1) {
            if (micros() - s > 30000) goto use_prev;
            e = micros();
        }

        dist = (e - s) / 58.0f;
        last_valid = dist;
        goto filter;

    use_prev:
        dist = last_valid;

    filter:
        if (dist < 2.0f || dist > 300.0f) dist = last_valid;

        if (ema == 0.0f) ema = dist;
        else ema = alpha * dist + (1.0f - alpha) * ema;
        dist = ema;

        // ===== LCD + LED + BUZZER + TFT =====
        lcd_clear();

        if (dist >= SAFE_DIST) {
            cur_state = ST_SAFE;

            if (cur_state != prev_state) {
                show_png_once("11.png");
                prev_state = cur_state;
            }

            lcd_set_cursor(0,0);
            lcd_print("SAFE");
            led_safe();
            softToneWrite(BUZZER_GPIO, 0);
        }
        else if (dist >= CAUTION_DIST) {
            cur_state = ST_CAUTION;

            if (cur_state != prev_state) {
                show_png_once("22.png");
                prev_state = cur_state;
            }

            lcd_set_cursor(0,0);
            lcd_print("CAUTION");
            led_caution();
            softToneWrite(BUZZER_GPIO, 0);
        }
        else {
            cur_state = ST_WARNING;

            if (cur_state != prev_state) {
                show_png_once("33.png");
                prev_state = cur_state;
            }

            lcd_set_cursor(0,0);
            lcd_print("WARNING!!");

            blink = !blink;
            if (blink) {
                led_warning_on();
                softToneWrite(BUZZER_GPIO, BUZZ_FREQ);
            } else {
                led_all_off();
                softToneWrite(BUZZER_GPIO, 0);
            }
        }

        char buf[17];
        snprintf(buf, sizeof(buf), "%.1f cm", dist);
        lcd_set_cursor(1,0);
        lcd_print(buf);

        delay(200);
    }
}
