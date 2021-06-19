/*
 * @FileName: start.ino
 * @Author: Tuya
 * @Email: 
 * @LastEditors: Tuya
 * @Date: 2021-04-10 11:24:27
 * @LastEditTime: 2021-04-28 19:48:31
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Description: This demo is based on the Arduino UNO, and the LEDs on the UNO board are controlled by the Tuya Smart App. 
 *               Enter network connection mode when Pin7 to GND.
 * @Github:https://github.com/tuya/tuya-wifi-mcu-sdk-arduino-library
 * 
 * ESP32    tuyaWifi
 * 3.3      3.3
 * gnd      gnd
 * tx       Rx
 * rx       Tx//很奇怪 live mini esp32这个位置竟然不是交叉线。ESP32模块却是交叉线~
 * 
 */

#include "audio_reactive.h"//INMP441声音处理模块
#include <FastLED.h>       //FastLED库
#include <LEDMatrix.h>     //FastMatrix库
#include <LEDText.h>       //LEDTest库
#include <FontMatrise.h>   //FontMatrix库
#include <EEPROM.h>        //EEPROM库：关键的控制信息写入EEPROM中

#include "TuyaWifi.h"//因为用的是ESP32所以不能用软串口，所以更改了tuyawifi.h中涉及软串口的代码，此处用的是双引号
//#include <SoftwareSerial.h>//ESP32没有软串口

//板载灯、配网引脚
#define LED_BUILDIN  5 //LIVE MINI ESP32板载灯对应的引脚
#define KEY_PIN 17//针对ESP32将引脚变为GPIO16:将该引脚接地时涂鸦模块开始进行配网

//LedMatrix相关
#define EEPROM_SIZE 5  //
#define LED_PIN     2  //Led的Din
#define M_WIDTH     16 //矩阵宽度
#define M_HEIGHT    16  //矩阵高度
#define NUM_LEDS    (M_WIDTH * M_HEIGHT)

//EEPROM相关
#define EEPROM_BRIGHTNESS   0 //亮度
#define EEPROM_GAIN         1 //
#define EEPROM_SQUELCH      2 //
#define EEPROM_PATTERN      3 //样式
#define EEPROM_DISPLAY_TIME 4 //显示时间

//涂鸦DP点相关
#define DPID_SWITCH 20 //涂鸦官方的DP定义 用来切换自动模式和手动模式 0 自动模式；1：手动模式
#define DPID_MODE 21   //不同显示效果的切换 一共6中显示模式
#define DPID_BRIGHT 22 //LED的亮度调节 取值范围0-30

//LedMatrix相关
uint8_t numBands;
uint8_t barWidth;
uint8_t pattern;
uint8_t brightness;
uint16_t displayTime;

TuyaWifi my_device;   //新建涂鸦设备实例

//标志位相关
unsigned char led_state = 0;//当前led灯的状态 涂鸦配网时控制板载灯的闪烁
bool autoChangePatterns = false;//自动切换样式标志位
/* last time */
unsigned long last_time = 0;

/* Stores all DPs and their types. PS: array[][0]:dpid, array[][1]:dp type. 
 *                                     dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
 *存放DP值和类型的数组
*/
unsigned char dp_array[][2] =
{
  {DPID_SWITCH, DP_TYPE_BOOL},
  {DPID_MODE, DP_TYPE_ENUM},
  {DPID_BRIGHT, DP_TYPE_VALUE},
};

unsigned char pid[] = {"nwe6q********"};//炫彩灯带对应的项目PID
unsigned char mcu_ver[] = {"1.0.0"};       //后续OTA功能可能会用到的版本号

//LedMatrix相关
cLEDMatrix<M_WIDTH, M_HEIGHT, HORIZONTAL_ZIGZAG_MATRIX> leds;
cLEDText ScrollingMsg;

uint8_t peak[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t prevFFTValue[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t barHeights[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// Colors and palettes
DEFINE_GRADIENT_PALETTE( purple_gp ) {
  0,   0, 212, 255,   //blue
255, 179,   0, 255 }; //purple
DEFINE_GRADIENT_PALETTE( outrun_gp ) {
  0, 141,   0, 100,   //purple
127, 255, 192,   0,   //yellow
255,   0,   5, 255 };  //blue
DEFINE_GRADIENT_PALETTE( greenblue_gp ) {
  0,   0, 255,  60,   //green
 64,   0, 236, 255,   //cyan
128,   0,   5, 255,   //blue
192,   0, 236, 255,   //cyan
255,   0, 255,  60 }; //green
DEFINE_GRADIENT_PALETTE( redyellow_gp ) {
  0,   200, 200,  200,   //white
 64,   255, 218,    0,   //yellow
128,   231,   0,    0,   //red
192,   255, 218,    0,   //yellow
255,   200, 200,  200 }; //white
CRGBPalette16 purplePal = purple_gp;
CRGBPalette16 outrunPal = outrun_gp;
CRGBPalette16 greenbluePal = greenblue_gp;
CRGBPalette16 heatPal = redyellow_gp;
uint8_t colorTimer = 0;



void setup() 
{
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds[0], NUM_LEDS);//初始化
  Serial.begin(9600);//开启串口
  setupAudio();    //开启声音

  if (M_WIDTH == 8) numBands = 8;
  else numBands = 16;  //宽度只能是8或者16
  barWidth = M_WIDTH / numBands;
  
  EEPROM.begin(EEPROM_SIZE);
  
  // It should not normally be possible to set the gain to 255
  // If this has happened, the EEPROM has probably never been written to
  // (new board?) so reset the values to something sane.
  //将led矩阵的初始状态值存入EEProm中
  if (EEPROM.read(EEPROM_GAIN) == 255) {
    EEPROM.write(EEPROM_BRIGHTNESS, 25);
    EEPROM.write(EEPROM_GAIN, 0);
    EEPROM.write(EEPROM_SQUELCH, 0);
    EEPROM.write(EEPROM_PATTERN, 0);
    EEPROM.write(EEPROM_DISPLAY_TIME, 10);
    EEPROM.commit();
  }

  // Read saved values from EEPROM将EEPROM中保存的数据读取出来~
  FastLED.setBrightness( EEPROM.read(EEPROM_BRIGHTNESS));//读取eprom中的亮度值
  brightness = FastLED.getBrightness();//将之前的亮度值写入bright变量中
  gain = EEPROM.read(EEPROM_GAIN);
  squelch = EEPROM.read(EEPROM_SQUELCH);
  pattern = EEPROM.read(EEPROM_PATTERN);
  displayTime = EEPROM.read(EEPROM_DISPLAY_TIME);


  //Initialize networking keys.配网引脚选择上拉电阻模式
  pinMode(KEY_PIN, INPUT_PULLUP);
  pinMode(LED_BUILDIN,OUTPUT);//这句不加则无法正常控制板载灯亮灭

  my_device.init(pid, mcu_ver);//初始化设备
  my_device.set_dp_cmd_total(dp_array, 3);//上报所有的dp命令信息
  my_device.dp_process_func_register(dp_process);//注册DP下报处理函数
  my_device.dp_update_all_func_register(dp_update_all);//注册DP上报函数

  last_time = millis();
}

void loop() 
{
  my_device.uart_service();//涂鸦配网模块开始通信

  //检测配网
  if (digitalRead(KEY_PIN) == LOW) {//ESP3的16引脚接地开启模块联网操作，在需要重新配网时选择该操作
    delay(80);
    if (digitalRead(KEY_PIN) == LOW) {
      my_device.mcu_set_wifi_mode(SMART_CONFIG);
    }
  }
  /*配网期间控制板载灯进行闪烁*/
  if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW)) {
    if (millis()- last_time >= 500) {
      last_time = millis();

      if (led_state == LOW) {
        led_state = HIGH;
      } else {
        led_state = LOW;
      }
      digitalWrite(LED_BUILDIN, led_state);
    }
  }

  //delay(10);

  //LedMatrix显示

  if (pattern != 5) FastLED.clear();
  
  uint8_t divisor = 1;                                                    // If 8 bands, we need to divide things by 2
  if (numBands == 8) divisor = 2;                                         // and average each pair of bands together
  
  for (int i = 0; i < 16; i += divisor) {
    uint8_t fftValue;
    
    if (numBands == 8) fftValue = (fftResult[i] + fftResult[i+1]) / 2;    // Average every two bands if numBands = 8
    else fftValue = fftResult[i];

    fftValue = ((prevFFTValue[i/divisor] * 3) + fftValue) / 4;            // Dirty rolling average between frames to reduce flicker
    barHeights[i/divisor] = fftValue / (255 / M_HEIGHT);                  // Scale bar height
    
    if (barHeights[i/divisor] > peak[i/divisor])                          // Move peak up
      peak[i/divisor] = min(M_HEIGHT, (int)barHeights[i/divisor]);
      
    prevFFTValue[i/divisor] = fftValue;                                   // Save prevFFTValue for averaging later
    
  }

  // Draw the patterns
  for (int band = 0; band < numBands; band++) {
    drawPatterns(band);
  }

  // Decay peak
  EVERY_N_MILLISECONDS(60) {
    for (uint8_t band = 0; band < numBands; band++)
      if (peak[band] > 0) peak[band] -= 1;
  }

  EVERY_N_SECONDS(30) {
    // Save values in EEPROM. Will only be commited if values have changed.
    EEPROM.write(EEPROM_BRIGHTNESS, brightness);
    EEPROM.write(EEPROM_GAIN, gain);
    EEPROM.write(EEPROM_SQUELCH, squelch);
    EEPROM.write(EEPROM_PATTERN, pattern);
    EEPROM.write(EEPROM_DISPLAY_TIME, displayTime);
    EEPROM.commit();
  }
  
  EVERY_N_SECONDS_I(timingObj, displayTime) {
    timingObj.setPeriod(displayTime);
    if (autoChangePatterns) pattern = (pattern + 1) % 6;
  }
  
  FastLED.setBrightness(brightness);
  FastLED.show();
}

/**
 * @description: DP download callback function.DP下报处理函数
 * @param {unsigned char} dpid 
 * @param {const unsigned char} value
 * @param {unsigned short} length
 * @return {unsigned char}
 */
unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length)
{
  switch(dpid) {
    case DPID_SWITCH:
      led_state = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
      if (led_state) {
        //Turn on
        digitalWrite(LED_BUILDIN, HIGH);//点亮板载灯
      } else {
        //Turn off
        digitalWrite(LED_BUILDIN, LOW);//关闭板载灯
      }
      //Status changes should be reported.
      my_device.mcu_dp_update(dpid, led_state, 1);//上报板载灯状态
    break;
    case DPID_MODE:
      pattern = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
      //pattern = (pattern + 1) % 6;
      my_device.mcu_dp_update(dpid, pattern, 1);//上报板载灯状态
    break;

    case DPID_BRIGHT:
      brightness = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
      //Status changes should be reported.
      my_device.mcu_dp_update(dpid, brightness, 1);//上报当前亮度值
    break;

    default:break;
  }
  return SUCCESS;
}

/**
 * @description: Upload all DP status of the current device.
 * @param {*}
 * @return {*}
 */
void dp_update_all(void)
{
  my_device.mcu_dp_update(DPID_SWITCH, led_state, 1);
  my_device.mcu_dp_update(DPID_MODE, pattern, 1);
  my_device.mcu_dp_update(DPID_BRIGHT, brightness, 1);
  
}

void drawPatterns(uint8_t band) {
  
  uint8_t barHeight = barHeights[band];
  
  // Draw bars
  switch (pattern) {
    case 0:
      rainbowBars(band, barHeight);
      break;
    case 1:
      // No bars on this one
      break;
    case 2:
      purpleBars(band, barHeight);
      break;
    case 3:
      centerBars(band, barHeight);
      break;
    case 4:
      changingBars(band, barHeight);
      EVERY_N_MILLISECONDS(10) { colorTimer++; }
      break;
    case 5:
      createWaterfall(band);
      EVERY_N_MILLISECONDS(30) { moveWaterfall(); }
      break;
  }

  // Draw peaks
  switch (pattern) {
    case 0:
      whitePeak(band);
      break;
    case 1:
      outrunPeak(band);
      break;
    case 2:
      whitePeak(band);
      break;
    case 3:
      // No peaks
      break;
    case 4:
      // No peaks
      break;
    case 5:
      // No peaks
      break;
  }
}

//////////// Patterns ////////////

void rainbowBars(uint8_t band, uint8_t barHeight) {
  int xStart = barWidth * band;
  for (int x = xStart; x < xStart + barWidth; x++) {
    for (int y = 0; y <= barHeight; y++) {
      leds(x,y) = CHSV((x / barWidth) * (255 / numBands), 255, 255);
    }
  }
}

void purpleBars(int band, int barHeight) {
  int xStart = barWidth * band;
  for (int x = xStart; x < xStart + barWidth; x++) {
    for (int y = 0; y < barHeight; y++) {
      leds(x,y) = ColorFromPalette(purplePal, y * (255 / barHeight));
    }
  }
}

void changingBars(int band, int barHeight) {
  int xStart = barWidth * band;
  for (int x = xStart; x < xStart + barWidth; x++) {
    for (int y = 0; y < barHeight; y++) {
      leds(x,y) = CHSV(y * (255 / M_HEIGHT) + colorTimer, 255, 255); 
    }
  }
}

void centerBars(int band, int barHeight) {
  int xStart = barWidth * band;
  for (int x = xStart; x < xStart + barWidth; x++) {
    if (barHeight % 2 == 0) barHeight--;
    int yStart = ((M_HEIGHT - barHeight) / 2 );
    for (int y = yStart; y <= (yStart + barHeight); y++) {
      int colorIndex = constrain((y - yStart) * (255 / barHeight), 0, 255);
      leds(x,y) = ColorFromPalette(heatPal, colorIndex);
    }
  }
}

void whitePeak(int band) {
  int xStart = barWidth * band;
  int peakHeight = peak[band];
  for (int x = xStart; x < xStart + barWidth; x++) {
    leds(x,peakHeight) = CRGB::White;
  }
}

void outrunPeak(int band) {
  int xStart = barWidth * band;
  int peakHeight = peak[band];
  for (int x = xStart; x < xStart + barWidth; x++) {
    leds(x,peakHeight) = ColorFromPalette(outrunPal, peakHeight * (255 / M_HEIGHT));
  }
}

void createWaterfall(int band) {
  int xStart = barWidth * band;
  // Draw bottom line
  for (int x = xStart; x < xStart + barWidth; x++) {
    leds(x,0) = CHSV(constrain(map(fftResult[band],0,254,160,0),0,160), 255, 255);
  }
}

void moveWaterfall() {
  // Move screen up starting at 2nd row from top
  for (int y = M_HEIGHT - 2; y >= 0; y--) {
    for (int x = 0; x < M_WIDTH; x++) {
      leds(x,y+1) = leds(x,y);
    }
  }
}
