#include "MyFunctions.h"

const char digits[] = "0123456789";

double char2double(uint8_t* Buf, uint32_t *Len){ 
    double sign=1, K=0;
    uint32_t i=0;
    while(Buf[i]==' ')++i; //ignoring spaces before number
    if (Buf[i]=='-'){
        i++;
        sign=-1.0;
    }
    for(;(i<*Len)&&(Buf[i]!='.')&&(Buf[i]!=',');i++){
        if((Buf[i]<'0')||(Buf[i]>'9')){
            break;//not a digit!
        }
        K=K*10;
        K+=Buf[i]-'0';
    }
    if((i<*Len)&&( (Buf[i]=='.')||(Buf[i]==',')) ){//then Buf[i]=='.'||','
        uint32_t iBase=i;
        double val;
        i++;
        for(;i<*Len;i++){
            if((Buf[i]<'0')||(Buf[i]>'9')){
                break;//not a digit!
            }
            val=Buf[i]-'0';
            for(uint32_t j=0;j<i-iBase;j++){
                val/=10;
            }
            K+=val;
        }
    }
    K*=sign;
    return K;
}

unsigned int char2uint(uint8_t* Buf, uint32_t *Len){
    unsigned int ans=0;
    
    for(uint32_t i=0;i<*Len;++i){
        ans*=10;
        ans+=Buf[i]-'0';
    }
    return ans;
}

int char2int(uint8_t* Buf, uint32_t *Len){

    int ans=0;
    
    bool sign=false;//false for +, true for -

    uint32_t len=*Len, i=0;
    
    sign=(Buf[i]=='-');//checking for negative
    
    i+=(sign||Buf[i]=='+')?1:0;//increasing counter if there were any sign
    len-=i;
    
    
    ans=char2uint(&Buf[i],&len);
    
    ans*=sign?(-1):1;
    
    return ans;
}
/*
void double2char3x3(double val, char *ans){
    ans="000.000";
    val*=1000.0;
    
    int len=strlen(ans);
    
    for(int i=len-2;i>0;i--){
        if(ans[i]!='.'){
            ans[i]=digits[((int)val)%10];
            val/=10.0;
        }
    }
}*/
