#include <stdlib.h>
#include <math.h>

#ifndef __DiscrFilt_H__
#define __DiscrFilt_H__
struct DiscrFilt;
struct DiscrFilt* discrFilt_new(void);
void discrFilt_setLens(struct DiscrFilt *filt ,unsigned int aLenIn, unsigned int bLenIn);
void discrFilt_setLen(struct DiscrFilt *filt ,unsigned int lenIn, char aOrbOrBoth);
_Bool discrFilt_setKoef(struct DiscrFilt *filt, unsigned int i, double val, char aOrb);
double discrFilt_getKoef(struct DiscrFilt *filt,unsigned int i, char aOrb);
double discrFilt_filt(struct DiscrFilt *filt, double xIn);
//void discrFilt_reduceKoefs(struct DiscrFilt *filt);
//void discrFilt_restoreKoefs(struct DiscrFilt *filt);
_Bool discrFilt_isReady(struct DiscrFilt *filt);
_Bool discrFilt_stop(struct DiscrFilt *filt);
_Bool discrFilt_start(struct DiscrFilt *filt);

unsigned int discFilt_dispLen(struct DiscrFilt *filt);
_Bool discFilt_disp(struct DiscrFilt *filt, char ans[], unsigned int *lenMax);
#endif
