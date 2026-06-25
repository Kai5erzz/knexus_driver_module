/**
  ******************************************************************************
  * @file    filter32.h
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2020/3/17
  * @brief   
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */
#ifndef __FILTER32_H
#define __FILTER32_H

#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "FreeRTOS.h"


#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

//#if (__CORTEX_M == (7U))

typedef struct
{
    float Input;        //杈撳叆鏁版嵁
    float Output;       //婊ゆ尝杈撳嚭鐨勬暟鎹?
    float RC;           //婊ゆ尝鍙傛暟 RC = 1/omegac
    float Frame_Period; //婊ゆ尝鐨勬椂闂撮棿闅?鍗曚綅 s
} __attribute__((__packed__)) First_Order_Filter_t;

typedef struct window_filter
{
    float Input;         //杈撳叆鏁版嵁
    float Output;        //婊ゆ尝杈撳嚭鐨勬暟鎹?
    uint8_t WindowSize;  //绐楀彛澶у皬
    uint8_t WindowNum;   //闇€瑕佹洿鏂扮殑绐楀彛鍊?
    float *WindowBuffer; //绐楀彛鏁版嵁缂撳啿鍖?
} __attribute__((__packed__)) Window_Filter_t;

typedef struct
{
    float Input;   //杈撳叆鏁版嵁
    float Output;  //婊ゆ尝杈撳嚭鐨勬暟鎹?
    uint8_t Order; //婊ゆ尝鍣ㄩ樁鏁?
    float *Num;    //Numerator
    float *Den;    //Denominator
    float *xbuf;
    float *ybuf;
} __attribute__((__packed__)) IIR_Filter_t;



/** 骞冲潎婊ゆ尝鍣?*/
#define ave_filter_times_max 10

typedef struct
{
    int16_t index;
    float value[ave_filter_times_max];
    float value_ave;
    float filter_times;
}ave_filter_t;

void ave_fil_init(ave_filter_t *ave_fil);
float ave_fil_update(ave_filter_t *ave_fil, float value, uint16_t max);
/** 骞冲潎婊ゆ尝鍣?*/



void First_Order_Filter_Init(First_Order_Filter_t *first_order_filter, float frame_period, float num);
float First_Order_Filter_Calculate(First_Order_Filter_t *first_order_filter, float input);
void Window_Filter_Init(Window_Filter_t *window_filter, uint8_t windowSize);
float Window_Filter_Calculate(Window_Filter_t *window_filter, float input);
void IIR_Filter_Init(IIR_Filter_t *iir_filter, float *num, float *den, uint8_t order);
float IIR_Filter_Calculate(IIR_Filter_t *iir_filter, float input);
//#endif

#endif
