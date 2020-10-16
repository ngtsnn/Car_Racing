#ifndef IR_CAR_H
#define IR_CAR_H

#include <stdint.h>
#include "Arduino.h"
#include "Support_Macro.h"
#include "EEPROM.h"

#define BT_1_PIN                  13
#define BT_2_PIN                  10
#define BT_3_PIN                  11
#define BT_4_PIN                  12

#define BT_1                      1
#define BT_2                      2
#define BT_3                      3
#define BT_4                      4
#define UNPRESS                   0

#define NO_IRLED                  7 /* Number of IR LEDs */
#define IRLED_PIN_BASE            A1 /* start position of analog IR led pin */

#define DISTANCE_PIN              A0 /* distance pin */
#define DISTANCE_THRESHOLD        1024/2 /* threshold analog distance pin */

#define HC595_CLK_PIN             2
#define HC595_DATA_PIN            7
#define HC595_LATCH_PIN           8

#define PWM_MAX                   255
#define MOTOR_PWM_A_PIN           5
#define MOTOR_PWM_B_PIN           6
#define STBY_BIT                  2
#define AIN1_BIT                  1
#define AIN2_BIT                  0
#define BIN1_BIT                  3
#define BIN2_BIT                  4

#define SERVO_PIN                 9
#define ANGLE_BASE                2900
#define ANGLE_MAX                 1300

#define SysLog_Begin(baudrate)    Serial.begin(baudrate)
#define SysLog_Print(content)     Serial.print(content)
#define SysLog_Println(content)   Serial.println(content)

class IRCar {
public:
  IRCar();
  ~IRCar();

  void Init(void);
  unsigned int* IRLed_GetAllAnalog(bool forceReadHW=true);
  unsigned int IRLed_GetEachAnalog(uint8_t LedIndex, bool forceReadHW=true);
  bool IRLed_SetAllThreshold(unsigned int thresholdArray[]);
  bool IRLed_SetEachThreshold(uint8_t LedIndex, unsigned int threshold);
  uint8_t IRLed_GetAllFilted(void);
  bool IRLed_GetEachFilted(uint8_t LedIndex);
  void IRLed_SerialPrintFilted(uint8_t filted);
  void Led_Display(uint8_t filted);

  uint8_t Buttons_Check(void);

  bool Distance_Get(void);

  void Run(int16_t left, int16_t right);
  void Stop(void);

  void Turn(int16_t angle);

  void Set_LedAll(uint8_t val);
  void Set_Led(uint8_t bit, bool state);
  void HC595_Update(uint8_t led, uint8_t motor);
protected:
  void _Pins_Init(void);
  void _Sterring_Init(void);
  void _Perph_Init(void);
  
  /* reserve */
  void _Set_MotorAll(uint8_t val);
  void _Set_Motor(uint8_t bit, bool state);

  unsigned int _IRLed_AnalogVal[NO_IRLED]={0,0,0,0,0,0,0};
  unsigned int _IRLed_thresholdVal[NO_IRLED]={0,0,0,0,0,0,0};
  uint8_t _IRLed_filtedVal=0;

  uint8_t _ledControlVal=0;
  uint8_t _motorControlVal=0;
};

#endif /* IR_CAR_H */
