#ifndef IR_CAR_CPP
#define IR_CAR_CPP

#include "IR_Car.h"

IRCar::IRCar() {
}
IRCar::~IRCar() {
}

void IRCar::_Pins_Init(void) {
  /* HC595 */
  pinMode(HC595_CLK_PIN, OUTPUT);
  pinMode(HC595_DATA_PIN, OUTPUT);
  pinMode(HC595_LATCH_PIN, OUTPUT);
  /* Motor PWM */
  pinMode(MOTOR_PWM_A_PIN, OUTPUT);
  pinMode(MOTOR_PWM_B_PIN, OUTPUT);
  /* Servo PWM */
  pinMode(SERVO_PIN, OUTPUT);
  /* Button */
  pinMode(BT_1_PIN, INPUT_PULLUP);
  pinMode(BT_2_PIN, INPUT_PULLUP);
  pinMode(BT_3_PIN, INPUT_PULLUP);
  pinMode(BT_4_PIN, INPUT_PULLUP);
}
void IRCar::_Sterring_Init(void) {
  /* Timer 1 */
  TCCR1A = 0;
  TCCR1B = 0;
  DDRB |= (1 << PB1);
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  TCCR1A |= (1 << COM1A1);
  TCCR1B |= (1 << CS11);
  // P_clock=16mhz/8=2mhz
  OCR1A = ANGLE_BASE;
  // 40000 ~ 20ms
  ICR1 = 40000;
}
void IRCar::_Perph_Init(void) {
  SysLog_Begin(115200);
  EEPROM.begin();
}
void IRCar::Init(void){
  _Pins_Init();
  _Perph_Init();
  /* steering */
  _Sterring_Init();
}

void IRCar::Set_LedAll(uint8_t val) {
  this->_ledControlVal = val;
}
void IRCar::setLed(uint8_t bit, bool state) {
  if(state == true) {
    sbi(this->_ledControlVal, bit);
  }
  else {
    cbi(this->_ledControlVal, bit);
  }
}

void IRCar::HC595_Update(uint8_t led, uint8_t motor){
  digitalWrite(HC595_LATCH_PIN, LOW);
  //shift out the bits;
  shiftOut(HC595_DATA_PIN, HC595_CLK_PIN, MSBFIRST, motor);
  shiftOut(HC595_DATA_PIN, HC595_CLK_PIN, LSBFIRST, led);
  //take the latch pin high so the LEDs will light up;
  digitalWrite(HC595_LATCH_PIN,HIGH);
}

void IRCar::Led_Display(uint8_t filted) {
  static uint8_t pre = 0x00;
  if(pre != filted) {
    Set_LedAll(filted);
    HC595_Update(_ledControlVal, _motorControlVal);
    pre = filted;
  }
}

unsigned int* IRCar::IRLed_GetAllAnalog(bool forceReadHW) {
  /* if read from hardware is required */
  if (forceReadHW == true) {
    for (uint8_t iLed = 0; iLed < NO_IRLED; iLed++) 
    {
      uint8_t iLedHW = iLed + IRLED_PIN_BASE;
      analogRead(iLedHW);
      this->_IRLed_AnalogVal[iLed] = 0;
      for(uint8_t i = 0; i < 16; i++)
      {
        this->_IRLed_AnalogVal[iLed] += analogRead(iLedHW);
      }
      this->_IRLed_AnalogVal[iLed] /= 16;
//      Serial.print(this->_IRLed_AnalogVal[iLed]);
//      Serial.print("\t");
    }
  }
//  Serial.println();
  return this->_IRLed_AnalogVal;
}

unsigned int IRCar::IRLed_GetEachAnalog(uint8_t LedIndex, bool forceReadHW) {
  /* if read from hardware is required */
  if (forceReadHW == true) 
  {
    uint8_t iLedHW = LedIndex + IRLED_PIN_BASE;
    analogRead(iLedHW);
    this->_IRLed_AnalogVal[LedIndex] = 0;
    for(uint8_t i = 0; i < 16; i++)
    {
      this->_IRLed_AnalogVal[LedIndex] += analogRead(iLedHW);
    }
    this->_IRLed_AnalogVal[LedIndex] /= 16;
  }
  return this->_IRLed_AnalogVal[LedIndex];
}
bool IRCar::IRLed_SetAllThreshold(unsigned int thresholdArray[]) {
  for (uint8_t iTh = 0; iTh < NO_IRLED; iTh++) {
    /* if input value is incorrect */
    if (thresholdArray[iTh] > 1024) return false;
    this->_IRLed_thresholdVal[iTh] = thresholdArray[iTh];
  }
  return true;
}
bool IRCar::IRLed_SetEachThreshold(uint8_t LedIndex, unsigned int threshold) {
  /* if input value is incorrect */
  if (threshold > 1024) return false;
  this->_IRLed_thresholdVal[LedIndex] = threshold;
  return true;
}
uint8_t IRCar::IRLed_GetAllFilted() {
  uint8_t filted = 0x00;
  for (uint8_t iLed = 0; iLed < NO_IRLED; iLed++) {
    /* if analog val below the coresponse threshold */
    if (this->_IRLed_AnalogVal[iLed] < this->_IRLed_thresholdVal[iLed]) {
      /* set bit */
      filted |= (0x01 << (NO_IRLED - iLed - 1));
    }
  }
  return filted;
}
bool IRCar::IRLed_GetEachFilted(uint8_t LedIndex) {
  if (this->_IRLed_AnalogVal[LedIndex] < this->_IRLed_thresholdVal[LedIndex]) {
    return true;
  }
  return false;
}

void IRCar::IRLed_SerialPrintFilted(uint8_t filted) {
  for(uint8_t iData=0; iData<NO_IRLED; iData++) {
    if(bit_is_set(filted, NO_IRLED-iData-1)) {
      SysLog_Print('1');
    }
    else {
      SysLog_Print('0');
    }
  }
  SysLog_Println();
}

uint8_t IRCar::Buttons_Check(void) {
  if(digitalRead(BT_1_PIN) == LOW) {
    while(digitalRead(BT_1_PIN) == LOW) {}
    delay(500);
    return BT_1;
  }
  if(digitalRead(BT_2_PIN) == LOW) {
    while(digitalRead(BT_2_PIN) == LOW) {}
    delay(500);
    return BT_2;
  }
  if(digitalRead(BT_3_PIN) == LOW) {
    while(digitalRead(BT_3_PIN) == LOW) {}
    delay(500);
    return BT_3;
  }
  if(digitalRead(BT_4_PIN) == LOW) {
    while(digitalRead(BT_4_PIN) == LOW) {}
    delay(500);
    return BT_4;
  }

  return UNPRESS;
}

bool IRCar::getDistance(void) 
{
//  bool distance =  !digitalRead(DISTANCE_PIN);
////  Serial.print(distance);
////  Serial.println('\t');
//  return distance;
  return !digitalRead(DISTANCE_PIN);
}

void IRCar::run(int16_t left, int16_t right) {
  /* stop motor */
  if(left == 0 && right == 0) {
    stop();
    return;
  }
  /* start motor */
  sbi(_motorControlVal, STBY_BIT);
  /* limit motor (dont edit anything here) */
  left += left > 0 ? 10 : - 10;
  if(left > PWM_MAX) {
    left = PWM_MAX;
  }
  if(left < -PWM_MAX) {
    left = -PWM_MAX;
  }
  if(right > PWM_MAX) {
    right = PWM_MAX;
  }
  if(right < -PWM_MAX) {
    right = -PWM_MAX;
  }
  /* direction */
  /* left spin backwards */
  if(left < 0) {
    cbi(_motorControlVal, BIN1_BIT);
    sbi(_motorControlVal, BIN2_BIT);
    left = -left;
  }
  /* left spin forwards */
  else {
    sbi(_motorControlVal, BIN1_BIT);
    cbi(_motorControlVal, BIN2_BIT);
  }
  /* right spin backwards */
  if(right < 0) {
    cbi(_motorControlVal, AIN1_BIT);
    sbi(_motorControlVal, AIN2_BIT);
    right = -right;
  }
  /* right spin forward */
  else {
    sbi(_motorControlVal, AIN1_BIT);
    cbi(_motorControlVal, AIN2_BIT);
  }
  /* update direction */
  HC595_Update(_ledControlVal, _motorControlVal);
  /* speed */
  analogWrite(MOTOR_PWM_A_PIN, right);
  analogWrite(MOTOR_PWM_B_PIN, left);
}
void IRCar::stop(void) {
  cbi(_motorControlVal, STBY_BIT);
  HC595_Update(_ledControlVal, _motorControlVal);
}
void IRCar::turn(int16_t angle) {
  angle = map(angle, -60, 60, -1500, 1500);
  if(angle>ANGLE_MAX) {
    angle = ANGLE_MAX;
  }
  if(angle<-ANGLE_MAX) {
    angle = -ANGLE_MAX;
  }
  OCR1A = (ANGLE_BASE + angle);
}

#endif /* IR_CAR_CPP */
