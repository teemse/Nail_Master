#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// =========================== CONFIG ===========================
#define MAX_TARGET_EMF 10000
#define DIV_R1 10000
#define DIV_R2 5100           // Поменяли на 5.1к!
#define PID_DIVIDER 4000.0

#define MIN_TARGET_EMF 200
#define REF1_1 1100
#define MIN_VMOT 6000
#define CTRL_PRD 20
#define WAIT_ADC 800
#define SMOOTH_STEP 200

// Пины
#define BEMF_PIN A0
#define VMOT_PIN A1
#define PWM_PIN  9
#define FAN_PIN  6
#define ENC_A 2
#define ENC_B 3
#define ENC_SW 4
#define BUZZER_PIN 8

// =========================== DATA ===========================
int vref;
int target = 0;
int pwm = 0;
int fanSpeed = 0;

bool isMotorPaused = false;
bool isFanPaused = false;
bool motorDisconnected = false; // Флаг отключения мотора

int savedMotorPos = 0;
int savedFanSpeed = 0;

// Фильтры
template <typename T>
class IntEMA {
   public:
    T filter(T val, uint8_t k2) {
        T sum = (val - _filt) + _err;
        T div = sum >> k2;
        _err = sum - (div << k2);
        return _filt += div;
    }
    void init(T val) { _filt = val; _err = 0; }
    T get() { return _filt; }
    operator T() { return _filt; }
   private:
    T _filt = 0, _err = 0;
};
IntEMA<int> vmot;
IntEMA<int> vemf;

// PI-регулятор
class PIreg {
   public:
    float Kp = 0.05;
    float Ki = 0.005;
    float integral = 0;

    int compute(int input, int setpoint, int feedforward, float dt) {
        float error = setpoint - input;
        float nextIntegral = integral + error * Ki * dt;
        float output = feedforward + error * Kp + nextIntegral;

        if (!((output > 255 && error > 0) || (output < 0 && error < 0))) {
            integral = nextIntegral;
        }
        return constrain(output, 0, 255);
    }
};
PIreg pi;

// Энкодер
volatile int32_t encoderPos = 0;
volatile int8_t encState = 0;
int8_t encTable[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// UI
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
enum ScreenMode { SCREEN_MOTOR, SCREEN_FAN, SCREEN_SETTINGS };
ScreenMode currentScreen = SCREEN_MOTOR;

int stepSize = 100;
bool btnPressed = false;
bool btnLongPressed = false;
uint32_t btnTimer = 0;
bool lastBtnState = HIGH;
uint8_t animFrame = 0;

// =========================== FUNC ===========================

void initMotorPWM() {
    pinMode(PWM_PIN, OUTPUT);
    TCCR1A = _BV(COM1A1) | _BV(WGM10); 
    TCCR1B = _BV(WGM12) | _BV(CS10); 
}

void setMotorPWM(uint8_t duty) {
    if (duty == 0) {
        bitClear(TCCR1A, COM1A1);
        digitalWrite(PWM_PIN, LOW);
    } else {
        bitSet(TCCR1A, COM1A1);
        OCR1A = duty;
    }
}

uint16_t readVCC(uint16_t ref1v1) {
    uint8_t muxt = ADMUX;
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC));
    uint16_t vcc = ref1v1 * 1023ul / ADC;
    ADMUX = muxt;
    return vcc;
}

uint16_t readDivider(uint8_t apin) {
    uint32_t pinMv = (uint32_t)analogRead(apin) * vref / 1024;
    return pinMv * (DIV_R1 + DIV_R2) / DIV_R2;
}

void encoderISR() {
    int8_t a = digitalRead(ENC_A);
    int8_t b = digitalRead(ENC_B);
    encState = (encState << 2) | (a << 1) | b;
    encoderPos += encTable[encState & 0x0F];
}

void playTone(int freq, int duration) {
    tone(BUZZER_PIN, freq, duration);
    delay(duration); 
    noTone(BUZZER_PIN);
}

int getMotorDisplayValue() {
    return map(constrain(encoderPos * stepSize, 0, MAX_TARGET_EMF), 0, MAX_TARGET_EMF, 0, 9999);
}

void updateFromEncoder() {
    static int32_t lastPos = 0;
    if (encoderPos == lastPos) return;
    
    int32_t diff = encoderPos - lastPos;
    lastPos = encoderPos;

    if (currentScreen == SCREEN_MOTOR) {
        int newTarget = constrain(encoderPos * stepSize, 0, MAX_TARGET_EMF);
        encoderPos = newTarget / stepSize;
    } 
    else if (currentScreen == SCREEN_FAN) {
        fanSpeed = constrain(fanSpeed + diff * 5, 0, 255);
    }
    else if (currentScreen == SCREEN_SETTINGS) {
        // В меню меняем Kp и Ki одновременно для простоты
        if (diff > 0) { pi.Kp += 0.005; pi.Ki += 0.001; }
        else          { pi.Kp -= 0.005; pi.Ki -= 0.001; }
        pi.Kp = constrain(pi.Kp, 0.0, 1.0);
        pi.Ki = constrain(pi.Ki, 0.0, 0.5);
    }
}

void control() {
    static uint32_t tmr;
    if (millis() - tmr < CTRL_PRD) return;
    tmr = millis();

    setMotorPWM(0);
    delayMicroseconds(WAIT_ADC);

    int rawVmot = readDivider(VMOT_PIN);
    int rawMotor = readDivider(BEMF_PIN);
    int rawBemf = rawVmot - rawMotor;

    // Проверка на обрыв мотора (если ШИМ был высокий, а BEMF нет)
    // Если ручка отключена, ток не течет, и BEMF ≈ Vmot
    if (pwm > 50 && rawBemf < 300) {
        motorDisconnected = true;
    } else if (rawBemf > 500) {
        motorDisconnected = false;
    }

    if (rawVmot > MIN_VMOT) setMotorPWM(pwm);

    vmot.filter(rawVmot, 2);
    vemf.filter(rawBemf, 2);

    int desiredTarget = 0;
    if (!isMotorPaused && !motorDisconnected && currentScreen == SCREEN_MOTOR) {
        desiredTarget = constrain(encoderPos * stepSize, 0, MAX_TARGET_EMF);
    }
    
    if (isMotorPaused || motorDisconnected || rawVmot <= MIN_VMOT || desiredTarget < MIN_TARGET_EMF) {
        pi.integral = 0;
        target = 0;
        pwm = 0;
        setMotorPWM(0);
        return;
    }

    target += constrain(desiredTarget - target, -SMOOTH_STEP, SMOOTH_STEP);
    int feedforward = (long)target * 255 / vmot.get();
    pwm = pi.compute(vemf.get(), target, feedforward, CTRL_PRD / 1000.0f);
    setMotorPWM(pwm);
}

void handleButton() {
    bool btnState = digitalRead(ENC_SW);
    
    if (btnState == LOW && lastBtnState == HIGH) {
        btnTimer = millis();
        btnPressed = true;
    }
    
    if (btnState == LOW && btnPressed) {
        if (millis() - btnTimer > 1000 && !btnLongPressed) {
            btnLongPressed = true;
            btnPressed = false;
            playTone(800, 50);
            if (currentScreen == SCREEN_SETTINGS) {
                currentScreen = SCREEN_MOTOR;
                encoderPos = savedMotorPos / stepSize;
            } else {
                savedMotorPos = encoderPos * stepSize;
                currentScreen = SCREEN_SETTINGS;
                encoderPos = 0;
            }
        }
    }
    
    if (btnState == HIGH && lastBtnState == LOW && btnPressed) {
        if (!btnLongPressed && millis() - btnTimer > 50) {
            playTone(1500, 30);
            
            if (currentScreen == SCREEN_SETTINGS) {
                currentScreen = SCREEN_MOTOR;
                encoderPos = savedMotorPos / stepSize;
            } 
            else if (currentScreen == SCREEN_MOTOR) {
                isMotorPaused = !isMotorPaused;
                if (isMotorPaused) {
                    savedMotorPos = encoderPos * stepSize;
                    encoderPos = 0;
                } else {
                    encoderPos = savedMotorPos / stepSize;
                }
            }
            else if (currentScreen == SCREEN_FAN) {
                isFanPaused = !isFanPaused;
                if (isFanPaused) {
                    savedFanSpeed = fanSpeed;
                    fanSpeed = 0;
                } else {
                    fanSpeed = savedFanSpeed;
                }
            }
        }
        btnPressed = false;
        btnLongPressed = false;
    }
    
    lastBtnState = btnState;
}

void drawDisplay() {
    static uint32_t tmr;
    if (millis() - tmr < 100) return;
    tmr = millis();
    
    animFrame++;

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);

    if (currentScreen == SCREEN_MOTOR) {
        u8g2.setCursor(0, 10);
        if (motorDisconnected) {
            u8g2.print("NO MOTOR!");
        } else {
            u8g2.print("MOTOR");
            if (isMotorPaused) u8g2.print(" [PAUSE]");
        }
        
        u8g2.setFont(u8g2_font_logisoso28_tn);
        char buf[10];
        if (motorDisconnected) {
            u8g2.drawStr(0, 45, "----");
        } else {
            sprintf(buf, "%04d", getMotorDisplayValue());
            u8g2.drawStr(0, 45, buf);
        }
        
        // Иконка
        u8g2.drawCircle(100, 30, 15);
        if (!isMotorPaused && !motorDisconnected && getMotorDisplayValue() > 0) {
            int ang = (animFrame * 15) % 360;
            int x = 100 + 12 * cos(ang * DEG_TO_RAD);
            int y = 30 + 12 * sin(ang * DEG_TO_RAD);
            u8g2.drawDisc(x, y, 3);
        } else {
            u8g2.drawDisc(100, 30, 3);
        }
    }
    else if (currentScreen == SCREEN_FAN) {
        u8g2.setCursor(0, 10);
        u8g2.print("FAN");
        if (isFanPaused) u8g2.print(" [PAUSE]");
        
        u8g2.setFont(u8g2_font_logisoso28_tn);
        char buf[10];
        sprintf(buf, "%03d%%", map(fanSpeed, 0, 255, 0, 100));
        u8g2.drawStr(0, 45, buf);
        
        int cx = 100, cy = 30;
        for (int i = 0; i < 4; i++) {
            int ang = i * 90 + (animFrame * 15);
            if (isFanPaused || fanSpeed == 0) ang = i * 90;
            int x = cx + 12 * cos(ang * DEG_TO_RAD);
            int y = cy + 12 * sin(ang * DEG_TO_RAD);
            u8g2.drawDisc(x, y, 5);
        }
        u8g2.drawDisc(cx, cy, 4);
    }
    else if (currentScreen == SCREEN_SETTINGS) {
        u8g2.setCursor(0, 10);
        u8g2.print("SETTINGS");
        
        u8g2.setCursor(0, 25);
        u8g2.print("Vmot: "); u8g2.print(vmot.get() / 1000.0, 1); u8g2.print("V");
        u8g2.setCursor(0, 35);
        u8g2.print("BEMF: "); u8g2.print(vemf.get()); u8g2.print("mV");
        u8g2.setCursor(0, 45);
        u8g2.print("Tar: "); u8g2.print(target); u8g2.print("mV");
        u8g2.setCursor(0, 55);
        u8g2.print("PWM: "); u8g2.print(pwm);
        
        u8g2.setCursor(60, 25);
        u8g2.print("Kp:"); u8g2.print(pi.Kp, 3);
        u8g2.setCursor(60, 35);
        u8g2.print("Ki:"); u8g2.print(pi.Ki, 3);
    }

    u8g2.sendBuffer();
}

void setup() {
    Serial.begin(115200);
    
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

    initMotorPWM();
    u8g2.begin();
    
    vref = readVCC(REF1_1);
    vmot.init(readDivider(VMOT_PIN));
    vemf.init(0);
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(5, 30, "NailMaster");
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(15, 50, "v1.3 Ready");
    u8g2.sendBuffer();
    playTone(1000, 100);
    delay(50);
    playTone(1500, 150);
    delay(500);
}

void loop() {
    //control();
    handleButton();
    updateFromEncoder();
    drawDisplay();
    
    // static int lastFan = -1;
    // int currentFan = isFanPaused ? 0 : fanSpeed;
    // if (currentFan != lastFan) {
    //     analogWrite(FAN_PIN, currentFan);
    //     lastFan = currentFan;
    // }
}