#ifndef BUZZER_H
#define BUZZER_H
#include "bsp_pwm.h"
#define BUZZER_DEVICE_CNT 5

#define  DoFreq  523
#define  ReFreq  587
#define  MiFreq  659
#define  FaFreq  698
#define  SoFreq  784
#define  LaFreq  880
#define  SiFreq  988
// 高八度音(高音 do/re/mi): 频率为中音的两倍
#define  HiDoFreq 1046
#define  HiReFreq 1174
#define  HiMiFreq 1318

typedef enum
{
    OCTAVE_1 = 0, // 中音 do
    OCTAVE_2,     // 中音 re
    OCTAVE_3,     // 中音 mi
    OCTAVE_4,     // 中音 fa
    OCTAVE_5,     // 中音 sol
    OCTAVE_6,     // 中音 la
    OCTAVE_7,     // 中音 si
    OCTAVE_8,     // 高音 do
    OCTAVE_9,     // 高音 re
    OCTAVE_10,    // 高音 mi
}octave_e;

typedef enum
{
    ALARM_LEVEL_HIGH = 0,
    ALARM_LEVEL_ABOVE_MEDIUM,
    ALARM_LEVEL_MEDIUM,
    ALARM_LEVEL_BELOW_MEDIUM,
    ALARM_LEVEL_LOW,
}AlarmLevel_e;

typedef enum
{
    ALARM_OFF = 0,
    ALARM_ON,
}AlarmState_e;
typedef struct
{
    AlarmLevel_e alarm_level;
    octave_e octave;
    float loudness;
}Buzzer_config_s;

typedef struct
{
    float loudness;
    octave_e octave;
    AlarmLevel_e alarm_level;
    AlarmState_e alarm_state;
}BuzzzerInstance;


void BuzzerInit();
void BuzzerTask();
BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config);
void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state);
#endif // !BUZZER_H
