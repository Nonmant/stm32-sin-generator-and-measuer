#include "DiscreteFilter.h"

struct DiscrFilt{
    double *a, *b;
    double *yArr,//выходные значения
        *xArr;//входные значения
    unsigned int aLen, bLen, iX, iY;
    _Bool koefsReduced;
};

struct DiscrFilt* discrFilt_new(void){
    static struct DiscrFilt filt;
    filt.aLen=0;
    filt.bLen=0;
    filt.koefsReduced=0;
    return &filt;
}

void discrFilt_mallocArr(struct DiscrFilt *filt, char aOrbOrBoth){//internal func
    
    if(aOrbOrBoth=='a'||aOrbOrBoth=='&'){
        free(filt->a);
        filt->a=(double*)calloc(filt->aLen, sizeof(double));
        filt->yArr=(double*)calloc(filt->aLen, sizeof(double));
    }
    
    if(aOrbOrBoth=='b'||aOrbOrBoth=='&'){
        free(filt->b);
        filt->b=(double*)calloc(filt->bLen, sizeof(double));
        filt->xArr=(double*)calloc(filt->bLen, sizeof(double));
    }
}

void discrFilt_setLens(struct DiscrFilt *filt , unsigned int aLenIn, unsigned int bLenIn){
    filt->aLen=aLenIn;
    filt->iY=aLenIn;
    filt->bLen=bLenIn;
    filt->iX=bLenIn;
    discrFilt_mallocArr(filt,'&');
}

void discrFilt_setLen(struct DiscrFilt *filt ,unsigned int lenIn, char aOrbOrBoth){
    
    if(aOrbOrBoth=='a'||aOrbOrBoth=='&'){
        filt->aLen=lenIn;
        filt->iY=lenIn;
    }
    
    if(aOrbOrBoth=='b'||aOrbOrBoth=='&'){
        filt->bLen=lenIn;
        filt->iX=lenIn;
    }
    
    discrFilt_mallocArr(filt, aOrbOrBoth);
}

_Bool discrFilt_setKoef(struct DiscrFilt *filt, unsigned int i, double val, char aOrb){
    if(aOrb=='a'){
        if(i>=filt->aLen) return 0;
        filt->a[i]=val;
        return 1;
    }
    if(aOrb=='b'){
        if(i>=filt->bLen) return 0;
        filt->b[i]=val;
        return 1;
    }
    return 0;
}
double discrFilt_getKoef(struct DiscrFilt *filt,unsigned int i, char aOrb){
    if(aOrb=='a'){
        if(i>=filt->aLen)return -1;
        return filt->a[i];
    }
    if(aOrb=='b'){
        if(i>=filt->bLen)return -1;
        return filt->b[i];
    }
    return -1;
}

double discrFilt_filt(struct DiscrFilt *filt, double xIn){
    //add x to xArr
    double *xArr=filt->xArr,
        *yArr=filt->yArr,
        *a=filt->a,
        *b=filt->b;
    unsigned int aLen=filt->aLen,
        bLen=filt->bLen,
        *iX=&filt->iX,
        *iY=&filt->iY;
    
    
    xArr[(*iX)%bLen]=xIn;
        
    double yOut=0;
    int i=bLen;
    while(i){
        --i;
        yOut+=b[i]*xArr[ (*iX - i)%bLen ];
    }
    
    ++*iX;
    if(*iX==bLen*2){
        *iX=bLen;
    }
    
    i=aLen;
    while(i>1){//presuming a0=1 - see _reduceKoefs
        --i;
        yOut-=a[i]*yArr[ (*iY-i)%aLen ];
    }
    
    yArr[ (*iY)%aLen ]=yOut;
    ++*iY;
    if(*iY==aLen*2){
        *iY=aLen;
    }
    return yOut;
}


void discrFilt_reduceKoefs(struct DiscrFilt *filt){
    //makes a[0]=1, so there is no need to divide result by it.
    //Koefs can be restored by _expandKoefs
    if(filt->koefsReduced)return;
    for(int i=0;i<filt->bLen;++i){
        filt->b[i]/=filt->a[0];
    }
    for(int i=1;i<filt->aLen;++i){
        filt->a[i]/=filt->a[0];
    }
    filt->koefsReduced=1;
}

void discrFilt_expandKoefs(struct DiscrFilt *filt){
    if(!filt->koefsReduced)return;
    for(int i=0;i<filt->bLen;++i){
        filt->b[i]*=filt->a[0];
    }
    for(int i=1;i<filt->aLen;++i){
        filt->a[i]*=filt->a[0];
    }
    filt->koefsReduced=0;
}

_Bool discrFilt_start(struct DiscrFilt *filt){
    if(!discrFilt_isReady(filt)) return 0;
    discrFilt_reduceKoefs(filt);
    return 1;
}

_Bool discrFilt_stop(struct DiscrFilt *filt){
    if(!discrFilt_isReady(filt)) return 0;
    discrFilt_expandKoefs(filt);
    return 1;
}

_Bool discrFilt_isReady(struct DiscrFilt *filt){
    return filt->aLen||filt->bLen;
}

unsigned int discFilt_dispLen(struct DiscrFilt *filt){
    unsigned int len=0;
    int temp;
    double tempTemp;
    _Bool comma, digitCounts;
    for(int i=0;i<filt->aLen;++i){
        //supposed tolerance 0.001
        tempTemp=filt->a[i];
        temp=filt->a[i];
        
        //before comma
        if(temp<0){//minus sign
            ++len;
            temp=-temp;
        }
        
        do{
            ++len;
            temp/=10.0;
        }while(temp>0);

        //after comma
        temp=((int)(round(1000*filt->a[i])))%1000;
        temp=temp>0?temp:-temp;
        
        if(temp){
            ++len;//comma
            digitCounts=0;
            for(int j=0;j<3;++j){//3 for 10^3
                if(temp%10){
                    digitCounts=1;
                }
                temp=temp/10;
                if(digitCounts)++len;
            }
            
        }
        ++len;//space after digit
        
    }
    
    for(int i=0;i<filt->bLen;++i){
        //supposed tolerance 0.001
        tempTemp=filt->b[i];
        temp=filt->b[i];
        
        //before comma
        if(temp<0){//minus sign
            ++len;
            temp=-temp;
        }
        
        do{
            ++len;
            temp/=10.0;
        }while(temp>0);

        //after comma
        temp=((int)(round(1000*filt->b[i])))%1000;
        
        if(temp){
            ++len;//comma
            digitCounts=0;
            for(int j=0;j<3;++j){//3 for 10^3
                if(temp%10){
                    digitCounts=1;
                }
                temp=temp/10;
                if(digitCounts)++len;
            }
            
        }
        ++len;//space after digit
        
    }
    return len;
}

_Bool discFilt_disp(struct DiscrFilt *filt, char ans[], unsigned int *lenMax){
    unsigned int i_ans=0;
    
    double tol=10000; // (1/1000)
    
    double temp, tempTemp;
    _Bool comma;
    double divider;
    int removedPart;
    
    for(int i=0;i<filt->aLen;++i){
        temp=filt->a[i];
        divider=1;
        removedPart=0;
        comma=0;
        
        if(temp<0){
            ans[i_ans]='-';
            ++i_ans;
            temp=-temp;
        }
        
        while(temp/divider>1){
            divider*=10;
        }
        if(divider!=1)divider/=10;
        
        while(round(temp*tol)/tol>(1/tol)){
            tempTemp=round(temp*tol);//
            tempTemp=round(temp*tol)/tol;//
            if(!comma&&temp<1&&divider!=1){
                ans[i_ans]='.';
                ++i_ans;
                comma=1;
            }  
            
            removedPart=(int)(round(temp/divider));
            ans[i_ans]=removedPart + '0';
            ++i_ans;
            temp-=removedPart*divider;
            divider/=10;
        };
        ans[i_ans]=' ';
        ++i_ans;
    }
    ans[i_ans-1]='\n';//instead of last space
    //b arr
    for(int i=0;i<filt->bLen;++i){
        temp=filt->b[i];
        divider=1;
        removedPart=0;
        comma=0;
        
        if(temp<0){
            ans[i_ans]='-';
            ++i_ans;
            temp=-temp;
        }
        
        while(temp/divider>1){
            divider*=10;
        }
        if(divider!=1)divider/=10;
        
        while(round(temp*1000)/1000>0.001){
            if(!comma&&temp<1&&divider!=1){
                ans[i_ans]='.';
                ++i_ans;
                comma=1;
            }  
            
            removedPart=(int)(round(temp/divider));
            ans[i_ans]=removedPart + '0';
            ++i_ans;
            temp-=removedPart*divider;
            divider/=10;
        };
        ans[i_ans]=' ';
        ++i_ans;
    }
    ans[i_ans-1]='\n';//instead of last space
    if(i_ans>=*lenMax){
        *lenMax=i_ans;
        return 0;
    }
    return 1;
}
