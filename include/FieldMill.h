#ifndef FM_include
#define FM_include
#include <stdint.h>
#include "driver/timer.h"

#define FM_INTERRUPTER_PIN 27
#define FM_Motor_PIN 4
#define FM_ADC_CLK_PIN 25
#define FM_ADC_DIN_PIN 33
#define FM_ADC_CS_PIN 32

#define FM_CONF_DEADTIME (TIMER_BASE_CLK / 4000000) * 1500 //in 25ns increments
#define FM_CONF_MOTORSPEED_GAIN 0.01f //1/( d(out/outN0)/(N0-))

/*
    Motor speed calibration maths:
    target rotor speed := n0
    FM_CONF_MOTORSPEED_GAIN := y
    speed difference = n-no := dn
    proportional value change = out@n0/out (dn) := r(dn)

    r(dn) = 1/(1 + dn * y)  //at n0 out = 1 * out@n0
<=> 1/r(dn) = 1 + dn * y
<=> (1/r(dn)) - 1) / dn = y

    to measure y:
1.  apply a static field to the mill
2.  log value at n0 (normally 3600 rpm) := out@n0
3.  log value at known offset (for example 3300rpm = 3600rpm - dn) := out
4.  use y = ((out@n0/out) - 1) / dn

    to apply cal to a value
    out@n0 = out * r(dn) = out * 1/(1 + dn*y)
*/

//#define FM_motorTuneP 50.0f

void FM_init();
void FM_loadSettings();
unsigned FM_isSampleUsable(uint64_t time);
int32_t FM_getRaw();
uint32_t FM_getMotorRPM();
float FM_getField();

extern unsigned FM_rotorPos;
#endif