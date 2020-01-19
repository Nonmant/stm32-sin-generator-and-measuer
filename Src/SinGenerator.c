#include "SinGenerator.h"
struct SinGener{
    int tim_clk;
    double sinFreqRad, waveFreqHz, sinFreqHz;
    TIM_TypeDef *waveGenTimer, *PWMGenTimer;
    
    int *vals, indSin, indSinLen, amp;
    char indQuad;//0:[0 pi/2] 1:[pi/2 pi] 2:[pi 3pi/4] 3:[3pi/4 2pi]
    unsigned long int indTick, indTickMax;
    
    enum SinGener_ModeOutput modeOutput;
    int pulseMin;//would be instead of 0 at pwm output
    _Bool flagPause;
    _Bool flagQuad;
    _Bool flagDefSignal;
};

struct SinGener* SinGener_new(TIM_TypeDef *waveGenTimer, TIM_TypeDef *PWMGenTimer){
    static struct SinGener sinGen;
    sinGen.flagPause=1;
    sinGen.PWMGenTimer=PWMGenTimer;
    sinGen.waveGenTimer=waveGenTimer;
    
    sinGen.amp=PWMGenTimer->ARR;// + 1;
    sinGen.indSinLen=25;
    sinGen.tim_clk   = 72e6;
    sinGen.indSin=0;
    sinGen.indTick=0;
    sinGen.indQuad=0;
    sinGen.pulseMin=max(sinGen.amp/10,1);
    sinGen.flagQuad=1;
    sinGen.flagDefSignal=1;
    
    sinGener_autoSetVals(&sinGen);//vals[25]
    
    sinGener_setSinFreqRad(&sinGen, 1);//sinFreqRad, waveFreqHz, indTickMax
    
    sinGener_setModeOut(&sinGen,SG_SINGLEOUT);
    
    sinGen.flagPause=0;
    
    return &sinGen;
}

void sinGener_autoSetVals(volatile struct SinGener* sinGen){
    sinGen->vals=(int*)calloc(sinGen->indSinLen, sizeof(int));
    double step=acos(-1)*0.5/(sinGen->indSinLen-1);// (pi/2)/(len-1) 
    int amp=sinGen->amp-sinGen->pulseMin;
    for(int i=0; i<sinGen->indSinLen;++i){
        sinGen->vals[i]=round(amp*sin(step*i)+sinGen->pulseMin);
        sinGen->vals[i]=abs(sinGen->vals[i]);
    }
}

_Bool sinGener_autoSetWaveParams(volatile struct SinGener* sinGen){
    sinGen->sinFreqHz=sinGen->sinFreqRad/(2*acos(-1));// v=w/(2pi)
    if(sinGen->flagQuad){
        sinGen->waveFreqHz=4*sinGen->indSinLen*sinGen->sinFreqHz;//4*len*v
    }else{
        sinGen->waveFreqHz=sinGen->indSinLen*sinGen->sinFreqHz;//len*v
    }
    
    double timerFreqHz=((double)sinGen->tim_clk)/
        (
            (sinGen->waveGenTimer->PSC+1)*
            (sinGen->waveGenTimer->ARR+1)*
            (sinGen->waveGenTimer->RCR+1)
        );
    
    sinGen->indTickMax=round(timerFreqHz/sinGen->waveFreqHz);
    if(sinGen->indTickMax==0){
        return 0;
    }
    return 1;
}

int sinGener_tick(volatile struct SinGener* sinGen, _Bool *valChanged, _Bool *signChanged, _Bool *signPositive){
    *valChanged=0;
    *signChanged=0;
    if(!sinGen->flagPause){
        ++sinGen->indTick;
    }

    if(sinGen->indTick>=sinGen->indTickMax){//sin value should change
        sinGen->indTick=0;
        *valChanged=1;
        
        ++sinGen->indSin;
        if(sinGen->indSin>=sinGen->indSinLen){//sin quadrant should change
            sinGen->indSin=0;
            
            ++sinGen->indQuad;
            
            if(sinGen->indQuad>=4){//quadtant index should be restarted from 0
                sinGen->indQuad=0;
            }
            
            *signChanged=!(sinGen->indQuad%2);
        }
    }
    *signPositive=sinGen->indQuad<2;
    
    int sinVal;
    
    if(sinGen->flagQuad){
        if(sinGen->indQuad%2){//1,3
            sinVal=sinGen->vals[sinGen->indSinLen-sinGen->indSin-1];
        }else{
            sinVal=sinGen->vals[sinGen->indSin];
        }
        
        sinVal*=(sinGen->indQuad<=1?1:-1);//0,1 - pos, 2,3 - neg
    }else{
        sinVal=sinGen->vals[sinGen->indSin];
    }
    sinGener_setPWM(sinGen, sinVal);
    
    return sinVal;
}


_Bool sinGener_setSinFreqRad(volatile struct SinGener* sinGen, double freqIn){
    sinGen->sinFreqRad=freqIn;
    return sinGener_autoSetWaveParams(sinGen);
}

void sinGen_info(volatile struct SinGener* sinGen, char *ans, unsigned int *ansLen){
    *ansLen=11;
    char indTickMax[11]="indTickMax=";
    for(int i=0; i<11; ++i){
        ans[i]=indTickMax[i];
    }
    ///Дописать!!!
}

void sinGener_setPWM(volatile struct SinGener* sinGen, int16_t pulse){
    switch(sinGen->modeOutput){
        pulse=max(pulse, sinGen->pulseMin);
        case SG_SINGLEOUT:
            sinGen->PWMGenTimer->CCR1=abs(pulse);
            break;
        
        case SG_ALTERNATEOUT:
            if(pulse>=0){
                sinGen->PWMGenTimer->CCR1=pulse;
                sinGen->PWMGenTimer->CCR2=sinGen->pulseMin;
            }
            else{
                sinGen->PWMGenTimer->CCR2=abs(pulse);
                sinGen->PWMGenTimer->CCR1=sinGen->pulseMin;
            }
            break;
        
    }
}

void sinGener_setModeOut(volatile struct SinGener* sinGen, enum SinGener_ModeOutput mode){
    sinGen->modeOutput=mode;
    int16_t pulse = max(sinGen->PWMGenTimer->CCR1, sinGen->PWMGenTimer->CCR2);
    sinGen->PWMGenTimer->CCR1=0;
    sinGen->PWMGenTimer->CCR2=0;
    sinGener_setPWM(sinGen, pulse);
}

enum SinGener_ModeOutput sinGener_getModeOut(volatile struct SinGener* sinGen){
    return sinGen->modeOutput;
}

void sinGener_pause(volatile struct SinGener* sinGen){
    sinGen->flagPause=1;
}
void sinGener_resume(volatile struct SinGener* sinGen){
    sinGen->flagPause=0;
}

void sinGener_setCusSignal(volatile struct SinGener* sinGen, double *arr, unsigned int len){
    sinGener_pause(sinGen);
    
    free(sinGen->vals);
    sinGen->flagDefSignal=0;
    sinGen->flagQuad=0;
    sinGen->indTick=0;
    sinGen->indSinLen=len;
    sinGen->vals=(int*)calloc(sinGen->indSinLen, sizeof(int));
    
    for(int i=0; i<len; ++i){
        sinGen->vals[i]=round(sinGen->amp*arr[i]);
    }
    sinGener_autoSetWaveParams(sinGen);
    sinGener_resume(sinGen);
}

void sinGener_setDefSignal(volatile struct SinGener* sinGen){
    sinGener_pause(sinGen);
    
    free(sinGen->vals);
    sinGen->flagQuad=1;
    sinGen->flagDefSignal=1;
    sinGen->indSin=0;
    sinGen->indTick=0;
    sinGen->indQuad=0;
    sinGen->indSinLen=25;
    
    sinGener_autoSetVals(sinGen);
    
    sinGener_resume(sinGen);
}

void sinGener_setPulseMin(volatile struct SinGener* sinGen, double percent){
    sinGener_pause(sinGen);
    sinGen->pulseMin=(sinGen->amp*percent)/100;
    if(sinGen->flagDefSignal){
        sinGener_autoSetVals(sinGen);
    }
    sinGener_resume(sinGen);
}