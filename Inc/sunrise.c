#include <stdio.h>
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "sunrise.h"

int lightLevel = 0;
int maxPower = 65535;
int oneSec = 10000000;
char* itoa(int i, char b[]);

void handleBtCommand() {
  
  int i = 100000;
  while(i--);
  
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
  
  uint8_t key = receiveBuffer[0];
  
  if (key=='s') {
    setTime();
  } 
  if (key=='n') {
    toggleLight(1);
  } 
  if (key=='m') {
    toggleLight(0);
  }
  
  sendTime();
  
}

void startSunrise() {
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  
  int seconds = 1;
  int lightSteps = 65535;
  
  int wholeCycle;
  int step;
  
  int cycles = 10;
  int cycle=0;
  
  while (cycle < cycles) {
    
    cycle++;
    
    seconds = cycle * 1;
    wholeCycle = seconds * oneSec;
    step = wholeCycle / lightSteps;
    
    uint32_t i = 0;
    uint16_t j = 0;
    
    while(i < seconds * oneSec) {
      i++;
      if (i % step == 0) {
        j++;
        if (j <= maxPower) TIM2->CCR1 = j;
        else TIM2->CCR1 = maxPower;
      }
    }
    
    j = maxPower;
    
    while(i--) {
      if (i % step == 0) {
        j--;
        if (j >= 100) TIM2->CCR1 = j;
        else TIM2->CCR1 = 100;
      }
    }
    
    TIM2->CCR1 = 100;
    
  }
  
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  
}

void setTime() {
  
  int i = 10000;
  while(i--);
  
  RTC_TimeTypeDef sTime;
  
  uint8_t h1 = receiveBuffer[1];
  uint8_t h2 = receiveBuffer[2];
  
  uint8_t m1 = receiveBuffer[4];
  uint8_t m2 = receiveBuffer[5];
  
  uint8_t s1 = receiveBuffer[7];
  uint8_t s2 = receiveBuffer[8];
  
  
  if (h1 && h2 && m1 && m2 && s1 && s2) {
    
    uint8_t h;
    uint8_t m;
    uint8_t s;
    
    h = (h1 - '0') * 10 + (h2 - '0');
    m = (m1 - '0') * 10 + (m2 - '0');
    s = (s1 - '0') * 10 + (s2 - '0');
    
    sTime.Hours = h;
    sTime.Minutes = m;
    sTime.Seconds = s;
    
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    
  }
  
}

void sendTime() {
  
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  
  uint8_t h = sTime.Hours;
  uint8_t m = sTime.Minutes;
  uint8_t s = sTime.Seconds;
  
  sendTimeIn(h, m, s);
  
}

void sendTimeIn(uint8_t h, uint8_t m, uint8_t s) {
  
  char hh[2];
  sprintf(hh, "%d", h);
  if (h<10) { 
    hh[1] = hh[0]; 
    hh[0] = '0'; 
  }
  transmitBuffer[0] = hh[0];
  transmitBuffer[1] = hh[1];
  transmitBuffer[2] = '-';
  
  char mm[2];
  sprintf(mm, "%d", m);
  if (m<10) {
    mm[1] = mm[0];
    mm[0] = '0';
  }
  transmitBuffer[3] = mm[0];
  transmitBuffer[4] = mm[1];
  transmitBuffer[5] = '-';
  
  char ss[2];
  sprintf(ss, "%d", s);
  if (s<10) {
    ss[1] = ss[0];
    ss[0] = '0';
  }
  transmitBuffer[6] = ss[0];
  transmitBuffer[7] = ss[1];
  transmitBuffer[8] = '\n';
  
  int i = 10000;
  while(i--);
  
  HAL_UART_Transmit(&huart3, transmitBuffer, 9, 100);
  
  i = 1000;
  while(i--);
  
  cleanTransmitBuffer();
  
}

void sendCount(uint8_t count) {
  
  char hh[2];
  sprintf(hh, "%d", count);
  transmitBuffer[0] = hh[0];
  transmitBuffer[1] = hh[1];
  transmitBuffer[2] = '\n';
  
  HAL_UART_Transmit(&huart3, transmitBuffer, 32, 50);
  
  cleanTransmitBuffer();
  
}

void sendBuff() {
  
  int i = 1000;
  while(i--);
  
  for (int i=0; i<32; i++) {
    transmitBuffer[i]=receiveBuffer[i]; 
  }
  
  transmitBuffer[31] = '\n';
  HAL_UART_Transmit(&huart3, transmitBuffer, 32, 100);
  
  cleanTransmitBuffer();
  
}

void cleanTransmitBuffer() {
  for (int i=0; i<32; i++) {
    transmitBuffer[i]=NULL; 
  }
}

void cleanReceiveBuffer() {
  for (int i=0; i<32; i++) {
    receiveBuffer[i]=NULL; 
  }
}

void charToBCD() {
}

void toggleLight(uint8_t flag) {
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  
    uint8_t p1 = receiveBuffer[1];
    uint8_t p2 = receiveBuffer[2];    
    uint8_t p;
    uint8_t isUp = 1;
    
    if (!flag) {
      p1 = '0';
      p2 = '0';
    }

    int newLevel;
    int diffLevel;
    int step;
    int cycle = oneSec / 2;

    char b[7];
    
    if (p1 && p2) p = (p1 - '0') * 10 + (p2 - '0');
    newLevel = (maxPower*p)/100;
    
    if (newLevel < lightLevel) {
      isUp = 0;
      diffLevel = lightLevel - newLevel;
    } else if (newLevel > lightLevel) {
      isUp = 1;
      diffLevel = newLevel - lightLevel;
    } else {
      return;
    }

    step = cycle / diffLevel;
    
    int i = 0;
    int j = lightLevel;
    
    if (isUp) {
      while(i < cycle) {
        i++;
        if (i % step == 0) {
          j++;
          if (j <= newLevel) TIM2->CCR1 = j;
          else TIM2->CCR1 = newLevel;
        }
      }
    }

    else {
      i = cycle;
      while(i--) {
        if (i % step == 0) {
          j--;
          if (j >= newLevel) TIM2->CCR1 = j;
          else TIM2->CCR1 = newLevel;
        }
      }
    }
    
    lightLevel = newLevel;
    TIM2->CCR1 = lightLevel;
    
    itoa(lightLevel, b);
    b[6] = '\n';
    
    HAL_UART_Transmit(&huart3, b, 7, 50);
    
    if (!flag) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    }
  
}

char* itoa(int i, char b[]){
    char const digit[] = "0123456789";
    char* p = b;
    if(i<0){
        *p++ = '-';
        i *= -1;
    }
    int shifter = i;
    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);
    return b;
}
