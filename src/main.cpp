#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "WiFi.h"
#include <Wire.h>
#include "esp_adc_cal.h"


#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif

#define TFT_MOSI            19
#define TFT_SCLK            18
#define TFT_CS              5
#define TFT_DC              16
#define TFT_RST             23

#define TFT_BL          4   // Display backlight control pin
#define ADC_EN          14  //ADC_EN is the ADC detection enable port
#define ADC_PIN         36
#define ADC_PIN2         37
#define BUTTON_1        35
#define BUTTON_2        0

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library


char buff[512];
int vref = 1100;
const int ledPin = 32;  // 16 corresponds to GPIO16

// setting PWM properties
const int freq = 34;
const int ledChannel = 0;
const int resolution = 8;

int duty = 127;


void showVoltage()
{
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
        uint16_t v = analogRead(ADC_PIN);
        float battery_voltage = ((float)v / 4095.0) * 3.3 * (vref / 1000.0);
        String voltage = String(battery_voltage) + "V";
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(TR_DATUM);
        tft.fillRect(131,12,6,15,TFT_YELLOW);
        tft.drawString(voltage,  128, 12);
        v = analogRead(ADC_PIN2);
        battery_voltage = ((float)v / 4095.0) * 3.3 * (vref / 1000.0);
        voltage = String(battery_voltage) + "V";
        tft.fillRect(131,34,6,15,TFT_WHITE);
        tft.drawString(voltage,  128, 34);
        tft.fillRect(131,110,6,15,TFT_BLUE);
        tft.drawString("PWM "+ String(duty),  128, 110);
    }
}


void setup()
{
    Serial.begin(115200);
    Serial.println("Start");

    /*
    ADC_EN is the ADC detection enable port
    If the USB port is used for power supply, it is turned on by default.
    If it is powered by battery, it needs to be set to high level
    */
    // pinMode(ADC_EN, OUTPUT);
    // digitalWrite(ADC_EN, HIGH);

    tft.init();
    tft.setRotation(0);
    tft.setTextColor(TFT_GREEN);
    tft.setCursor(0, 0);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(2);

    if (TFT_BL > 0) { // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
        pinMode(TFT_BL, OUTPUT); // Set backlight pin to output mode
        digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    }

    tft.setSwapBytes(true);

    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        Serial.println("Default Vref: 1100mV");
    }

    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);

    pinMode(0,INPUT_PULLUP);
    pinMode(35,INPUT_PULLUP);

    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(ledPin, ledChannel);

}



void loop()
{
  uint16_t v = analogRead(ADC_PIN);
  float battery_voltage = ((float)v / 4095.0) * 3.3 * (vref / 1000.0);
  v = analogRead(ADC_PIN2);
  float battery_voltage2 = ((float)v / 4095.0) * 3.3 * (vref / 1000.0);
  Serial.print(battery_voltage);
  Serial.print(",");
  Serial.println(battery_voltage2);

  if(digitalRead(0)==0)
  {
    if(duty<255)
    {
        duty++;  
        ledcWrite(ledChannel, duty); 
        delay(100); 
    }  
    
  }  
  if(digitalRead(35)==0)
  {
    if(duty>0)
    {  
    duty--;  
    ledcWrite(ledChannel, duty);  
    delay(100);
    }
  }  

  showVoltage();
}