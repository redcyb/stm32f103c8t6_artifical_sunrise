extern uint8_t transmitBuffer[32];
extern uint8_t receiveBuffer[32];
extern int lightLevel;
extern int maxPower;
extern int oneSec;

extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern RTC_AlarmTypeDef sAlarm;

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;

extern void startSunrise();
extern void setTime();
extern void toggleLight(uint8_t flag);

extern void handleBtCommand();

extern void sendTime();
extern void sendTimeIn(uint8_t h, uint8_t m, uint8_t s);
extern void sendCount(uint8_t count);
extern void sendBuff();

extern void cleanTransmitBuffer();
extern void cleanReceiveBuffer();
