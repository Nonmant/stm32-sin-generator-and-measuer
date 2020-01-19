#include <math.h>
#include "stm32f1xx_hal.h"

#ifndef __SinGener_H__
#define __SinGener_H__

#define max(x, y) ((x) > (y) ? (x) : (y))

struct SinGener;
enum SinGener_ModeOutput{
        SG_SINGLEOUT,
        SG_ALTERNATEOUT
};
struct SinGener* SinGener_new(TIM_TypeDef *waveGenTimer, TIM_TypeDef *PWMGenTimer);
void sinGener_autoSetVals(volatile struct SinGener* sinGen);
_Bool sinGener_autoSetWaveParams(volatile struct SinGener* sinGen);
int sinGener_tick(volatile struct SinGener* sinGen, _Bool *valChanged, _Bool *signChanged, _Bool *signPositive);
_Bool sinGener_setSinFreqRad(volatile struct SinGener* sinGen, double freqIn);
void sinGener_setPWM(volatile struct SinGener* sinGen, int16_t pulse);
void sinGener_setModeOut(volatile struct SinGener* sinGen, enum SinGener_ModeOutput mode);
enum SinGener_ModeOutput sinGener_getModeOut(volatile struct SinGener* sinGen);
void sinGener_pause(volatile struct SinGener* sinGen);
void sinGener_resume(volatile struct SinGener* sinGen);
void sinGener_setCusSignal(volatile struct SinGener* sinGen, double *arr, unsigned int len);
void sinGener_setDefSignal(volatile struct SinGener* sinGen);
void sinGener_setPulseMin(volatile struct SinGener* sinGen, double percent);
#endif
