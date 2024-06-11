#define SPEED_BOTTOM_RIGHT     5
#define DIR_BOTTOM_RIGHT       4
#define SPEED_TOP_RIGHT        6
#define DIR_TOP_RIGHT          7
#define SPEED_BOTTOM_LEFT      10
#define DIR_BOTTOM_LEFT        11
#define SPEED_TOP_LEFT         9
#define DIR_TOP_LEFT           8

#define ENCODER_A_TOP_LEFT     2
#define ENCODER_B_TOP_LEFT     23
#define ENCODER_A_TOP_RIGHT    18
#define ENCODER_B_TOP_RIGHT    25
#define ENCODER_A_BOTTOM_LEFT  20
#define ENCODER_B_BOTTOM_LEFT  27
#define ENCODER_A_BOTTOM_RIGHT 21
#define ENCODER_B_BOTTOM_RIGHT 29

#define PPM_INTERRUPT          3

#define _Kp                    10
#define _Ki                    3
#define _Kd                    5

#include <GyverPID.h>

GyverPID regulatorBL(_Kp, _Ki, _Kd);
GyverPID regulatorTL(_Kp, _Ki, _Kd);
GyverPID regulatorBR(_Kp, _Ki, _Kd);
GyverPID regulatorTR(_Kp, _Ki, _Kd);

class MotorController {
  public:
  int8_t EncoderPinA;
  int8_t EncoderPinB;
  volatile int EncoderCount;
  long CurrentPosition;
  long PreviousPosition;
  long CurrentTime;
  long PreviousTime;
  long CurrentTimeforError;
  long PreviousTimeforError;
  float rpmFilt;
  float rpmPrev;

  MotorController(int8_t EncoderA, int8_t EncoderB) {
    this->EncoderPinA = EncoderA;
    this->EncoderPinB = EncoderB;
    pinMode(EncoderPinA, INPUT);
    pinMode(EncoderPinB, INPUT);
  }

  int tick = 34;

  float getRpm() {
    CurrentPosition = EncoderCount;
    CurrentTime = millis();
    float delta1 = ((float) CurrentTime - PreviousTime) / 1.0e3;
    float velocity = ((float) CurrentPosition - PreviousPosition) / delta1;
    float rpm = (velocity / tick) * 60;
    rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev;
    float rpmPrev = rpm;
    PreviousPosition = CurrentPosition;
    PreviousTime = CurrentTime;
    return rpmFilt;
  }
};

MotorController topLeftWheel(ENCODER_A_TOP_LEFT, ENCODER_B_TOP_LEFT);
MotorController topRightWheel(ENCODER_A_TOP_RIGHT, ENCODER_B_TOP_RIGHT);
MotorController bottomLeftWheel(ENCODER_A_BOTTOM_LEFT, ENCODER_B_BOTTOM_LEFT);
MotorController bottomRightWheel(ENCODER_A_BOTTOM_RIGHT, ENCODER_B_BOTTOM_RIGHT);

// for ppm
int channelAmount = 6;
int ch[6];
unsigned long minChannelValue = 1000;
unsigned long maxChannelValue = 2000;
unsigned long channelValueMaxError = 10;
unsigned long blankTime = 2100;
volatile unsigned long *validValues = new unsigned long[channelAmount];
volatile byte pulseCounter = 0;
volatile unsigned long microsAtLastPulse = 0;

// for speed
int speed_t_l, speed_t_r, speed_b_l, speed_b_r;
int speed_str = 0;
int speed_turn = 0;
int speed_side = 0;
char mode = 0;

// for encoder
volatile char stateTL = 0;
volatile char stateTR = 0;
volatile char stateBL = 0;
volatile char stateBR = 0;

// получение данных с пульта управления
void ppm() {
  unsigned long previousMicros = microsAtLastPulse;
  microsAtLastPulse = micros();
  unsigned long time = microsAtLastPulse - previousMicros;

  if (time > blankTime)
    pulseCounter = 0;
  else {
    if (pulseCounter < channelAmount) {
      if (time >= minChannelValue - channelValueMaxError && time <= maxChannelValue + channelValueMaxError)
        validValues[pulseCounter] = constrain(time, minChannelValue, maxChannelValue);
    }
    ++pulseCounter;
  }
}

// Возвращает последнее значение с пульта управления
unsigned long latestValidChannelValue(byte channel, unsigned long defaultValue) {
  unsigned long value = defaultValue;
  if (channel >= 1 && channel <= channelAmount) {
    noInterrupts();
    value = validValues[channel - 1];
    interrupts();
  }
  return value;
}

// Запись новых значений с пульта управления по всем каналам
void readPPMData() {
  for (int channel = 1; channel <= channelAmount; ++channel) {
    unsigned long value = latestValidChannelValue(channel, 0);
    ch[channel - 1] = value;
  }
}

// Расчёт желаемой скорости с пульта управления и режима езды робота
void getSpeed() {
  speed_str = (1504 - ch[1]) * 0.51;
  speed_turn = (1488 - ch[3]) * 0.51;
  speed_side = (1488 - ch[0]) * 0.51;

  if (speed_side < -5 || speed_side > 5) mode = 3;
  else if (speed_turn < -5 || speed_turn > 5) mode = 2;
  else if (speed_str < -5 || speed_str > 5) mode = 1;
  else mode = 0;
}

// Расчёт направления и скорости для каждого колеса в соответствии с режимом езды
void calculateSpeed() {
  int sign = 1;
  if (speed_str > 0) sign = 1;
  else sign = -1;
  switch (mode) {
    case 0:
      speed_t_l = 0;
      speed_t_r = 0;
      speed_b_l = 0;
      speed_b_r = 0;
      break;
    case 1:
      if (speed_str > 0) {
        digitalWrite(DIR_TOP_RIGHT, HIGH);
        digitalWrite(DIR_TOP_LEFT, LOW);
        digitalWrite(DIR_BOTTOM_RIGHT, HIGH);
        digitalWrite(DIR_BOTTOM_LEFT, LOW);

        speed_t_l = speed_str;
        speed_t_r = speed_str;
        speed_b_l = speed_str;
        speed_b_r = speed_str;
      }
      else {
        digitalWrite(DIR_TOP_RIGHT, LOW);
        digitalWrite(DIR_TOP_LEFT, HIGH);
        digitalWrite(DIR_BOTTOM_RIGHT, LOW);
        digitalWrite(DIR_BOTTOM_LEFT, HIGH);

        speed_t_l = -speed_str;
        speed_t_r = -speed_str;
        speed_b_l = -speed_str;
        speed_b_r = -speed_str;
      }
      break;
    case 2:
      if (speed_str < -5 || speed_str > 5) {
        if (speed_str > 0) {
          digitalWrite(DIR_TOP_RIGHT, HIGH);
          digitalWrite(DIR_TOP_LEFT, LOW);
          digitalWrite(DIR_BOTTOM_RIGHT, HIGH);
          digitalWrite(DIR_BOTTOM_LEFT, LOW);
        }
        else {
          digitalWrite(DIR_TOP_RIGHT, LOW);
          digitalWrite(DIR_TOP_LEFT, HIGH);
          digitalWrite(DIR_BOTTOM_RIGHT, LOW);
          digitalWrite(DIR_BOTTOM_LEFT, HIGH);
        }
        if (speed_turn < 0) {
          speed_t_r = sign * speed_str - speed_turn;
          if (speed_t_r > 255) speed_t_r = 255;
          speed_b_r = sign * speed_str - speed_turn;
          if (speed_b_r > 255) speed_b_r = 255;
          speed_t_l = sign * speed_str + speed_turn;
          if (speed_t_l < 0) speed_t_l = 0;
          speed_b_l = sign * speed_str + speed_turn;
          if (speed_b_l < 0) speed_b_l = 0;
        }
        else {
          if (speed_str > 0) sign = 1;
          else sign = -1;
          speed_t_l = sign * speed_str + speed_turn;
          if (speed_t_l > 255) speed_t_l = 255;
          speed_b_l = sign * speed_str + speed_turn;
          if (speed_b_l > 255) speed_b_l = 255;
          speed_t_r = sign * speed_str - speed_turn;
          if (speed_t_r < 0) speed_t_r = 0;
          speed_b_r = sign * speed_str - speed_turn;
          if (speed_b_r < 0) speed_b_r = 0;
        }
      }
      break;
    case 3:
      if (speed_side < 0) {
        digitalWrite(DIR_TOP_RIGHT, HIGH);
        digitalWrite(DIR_BOTTOM_RIGHT, LOW);
        digitalWrite(DIR_TOP_LEFT, HIGH);
        digitalWrite(DIR_BOTTOM_LEFT, LOW);

        if (speed_str < 0) {
          speed_t_l = -speed_side;
          speed_b_r = -speed_side;
          speed_t_r = sign * speed_str;
          speed_b_l = sign * speed_str;
        }
        else if (speed_str >= -5 && speed_str <= 5) {
          speed_t_l = -speed_side;
          speed_b_r = -speed_side;
          speed_t_r = -speed_side;
          speed_b_l = -speed_side;
        }
        else {
          speed_t_l = sign * speed_str;
          speed_b_r = sign * speed_str;
          speed_t_r = -speed_side;
          speed_b_l = -speed_side;
        }
      }
      else {
        digitalWrite(DIR_TOP_RIGHT, LOW);
        digitalWrite(DIR_BOTTOM_RIGHT, HIGH);
        digitalWrite(DIR_TOP_LEFT, LOW);
        digitalWrite(DIR_BOTTOM_LEFT, HIGH);

        if (speed_str < 0) {
          speed_t_l = sign * speed_str;
          speed_b_r = sign * speed_str;
          speed_t_r = speed_side;
          speed_b_l = speed_side;
        }
        else if (speed_str >= -5 && speed_str <= 5) {
          speed_t_l = speed_side;
          speed_b_r = speed_side;
          speed_t_r = speed_side;
          speed_b_l = speed_side;
        }
        else {
          speed_t_l = speed_side;
          speed_b_r = speed_side;
          speed_t_r = sign * speed_str;
          speed_b_l = sign * speed_str;
        }
      }
      break;
  }
}

// Установка значений желаемой и текущей скоростей в ПИД - регуляторы для каждого колеса
void setSpeed() {
  regulatorBL.setpoint = speed_b_l;
  regulatorBL.input = bottomLeftWheel.getRpm();
  analogWrite(SPEED_BOTTOM_LEFT, regulatorBL.getResultTimer());

  regulatorBR.setpoint = speed_b_r;
  regulatorBR.input = bottomRightWheel.getRpm();
  analogWrite(SPEED_BOTTOM_RIGHT, regulatorBR.getResultTimer());

  regulatorTL.setpoint = speed_t_l;
  regulatorTL.input = topLeftWheel.getRpm();
  analogWrite(SPEED_TOP_LEFT, regulatorTL.getResultTimer());

  regulatorTR.setpoint = speed_t_r;
  regulatorTR.input = topRightWheel.getRpm();
  analogWrite(SPEED_TOP_RIGHT, regulatorTR.getResultTimer());
}

// Обработка сигналов энкодера для переднего левого колеса
void updateEncoderTL() {
  char s = digitalRead(topLeftWheel.EncoderPinA) | (digitalRead(topLeftWheel.EncoderPinB) << 1);
  switch(stateTL) {
    case 0:
      if (s == 1) {topLeftWheel.EncoderCount++; stateTL = s; break;}
      if (s == 2) {topLeftWheel.EncoderCount--; stateTL = s; break;}
      break;
    case 1:
      if (s == 3) {topLeftWheel.EncoderCount++; stateTL = s; break;}
      if (s == 0) {topLeftWheel.EncoderCount--; stateTL = s; break;}
      break;
    case 2:
      if (s == 0) {topLeftWheel.EncoderCount++; stateTL = s; break;}
      if (s == 3) {topLeftWheel.EncoderCount--; stateTL = s; break;}
      break;
    case 3:
      if (s == 2) {topLeftWheel.EncoderCount++; stateTL = s; break;}
      if (s == 1) {topLeftWheel.EncoderCount--; stateTL = s; break;}
      break;
  }
}

// Обработка сигналов энкодера для переднего правого колеса
void updateEncoderTR() {
  char s = digitalRead(topRightWheel.EncoderPinA) | (digitalRead(topRightWheel.EncoderPinB) << 1);
  switch(stateTR) {
    case 0:
      if (s == 1) {topRightWheel.EncoderCount++; stateTR = s; break;}
      if (s == 2) {topRightWheel.EncoderCount--; stateTR = s; break;}
      break;
    case 1:
      if (s == 3) {topRightWheel.EncoderCount++; stateTR = s; break;}
      if (s == 0) {topRightWheel.EncoderCount--; stateTR = s; break;}
      break;
    case 2:
      if (s == 0) {topRightWheel.EncoderCount++; stateTR = s; break;}
      if (s == 3) {topRightWheel.EncoderCount--; stateTR = s; break;}
      break;
    case 3:
      if (s == 2) {topRightWheel.EncoderCount++; stateTR = s; break;}
      if (s == 1) {topRightWheel.EncoderCount--; stateTR = s; break;}
      break;
  }
}

// Обработка сигналов энкодера для заднего левого колеса
void updateEncoderBL() {
  char s = digitalRead(bottomLeftWheel.EncoderPinA) | (digitalRead(bottomLeftWheel.EncoderPinB) << 1);
  switch(stateBL) {
    case 0:
      if (s == 1) {bottomLeftWheel.EncoderCount++; stateBL = s; break;}
      if (s == 2) {bottomLeftWheel.EncoderCount--; stateBL = s; break;}
      break;
    case 1:
      if (s == 3) {bottomLeftWheel.EncoderCount++; stateBL = s; break;}
      if (s == 0) {bottomLeftWheel.EncoderCount--; stateBL = s; break;}
      break;
    case 2:
      if (s == 0) {bottomLeftWheel.EncoderCount++; stateBL = s; break;}
      if (s == 3) {bottomLeftWheel.EncoderCount--; stateBL = s; break;}
      break;
    case 3:
      if (s == 2) {bottomLeftWheel.EncoderCount++; stateBL = s; break;}
      if (s == 1) {bottomLeftWheel.EncoderCount--; stateBL = s; break;}
      break;
  }
}

// Обработка сигналов энкодера для заднего правого колеса
void updateEncoderBR() {
  char s = digitalRead(bottomRightWheel.EncoderPinA) | (digitalRead(bottomRightWheel.EncoderPinB) << 1);
  switch(stateBR) {
    case 0:
      if (s == 1) {bottomRightWheel.EncoderCount++; stateBR = s; break;}
      if (s == 2) {bottomRightWheel.EncoderCount--; stateBR = s; break;}
      break;
    case 1:
      if (s == 3) {bottomRightWheel.EncoderCount++; stateBR = s; break;}
      if (s == 0) {bottomRightWheel.EncoderCount--; stateBR = s; break;}
      break;
    case 2:
      if (s == 0) {bottomRightWheel.EncoderCount++; stateBR = s; break;}
      if (s == 3) {bottomRightWheel.EncoderCount--; stateBR = s; break;}
      break;
    case 3:
      if (s == 2) {bottomRightWheel.EncoderCount++; stateBR = s; break;}
      if (s == 1) {bottomRightWheel.EncoderCount--; stateBR = s; break;}
      break;
  }
}

void setup() {
  for (int i = 0; i < channelAmount; ++i)
    validValues[i] = 0;

  for (int i = 4; i < 12; i++)
    pinMode(i, OUTPUT);
  
  pinMode(PPM_INTERRUPT, INPUT);

  // encoder_bottom_left
  pinMode(51, INPUT_PULLUP);
  digitalWrite(51, LOW);
  pinMode(53, INPUT_PULLUP);
  digitalWrite(53, HIGH);
  // encoder_top_left
  pinMode(45, INPUT_PULLUP);
  digitalWrite(45, LOW);
  pinMode(47, INPUT_PULLUP);
  digitalWrite(47, HIGH);
  // encoder_top_right
  pinMode(39, INPUT_PULLUP);
  digitalWrite(39, LOW);
  pinMode(41, INPUT_PULLUP);
  digitalWrite(41, HIGH);
  // encoder_bottom_right
  pinMode(33, INPUT_PULLUP);
  digitalWrite(33, LOW);
  pinMode(35, INPUT_PULLUP);
  digitalWrite(35, HIGH);

  attachInterrupt(digitalPinToInterrupt(PPM_INTERRUPT), ppm, RISING);

  attachInterrupt(digitalPinToInterrupt(topLeftWheel.EncoderPinA), updateEncoderTL, RISING);
  attachInterrupt(digitalPinToInterrupt(topRightWheel.EncoderPinA), updateEncoderTR, RISING);
  attachInterrupt(digitalPinToInterrupt(bottomLeftWheel.EncoderPinA), updateEncoderBL, RISING);
  attachInterrupt(digitalPinToInterrupt(bottomRightWheel.EncoderPinA), updateEncoderBR, RISING);

  regulatorTL.setDirection(NORMAL);
  regulatorTL.setLimits(0, 255);
  regulatorTR.setDirection(NORMAL);
  regulatorTR.setLimits(0, 255);
  regulatorBL.setDirection(NORMAL);
  regulatorBL.setLimits(0, 255);
  regulatorBR.setDirection(NORMAL);
  regulatorBR.setLimits(0, 255);
}

void loop() {
  readPPMData();
  getSpeed();
  calculateSpeed();
  setSpeed();
}
