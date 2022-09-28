#include "Arduino.h"  
#include <SeedlingMonitoringWifiManager.h>
#include <Timer.h>
#include <PCF8563TimeManager.h>
#include <Esp32SecretManager.h>
#include <Arduino_JSON.h>
#include <FastLED.h>
#include <TM1637Display.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <PanchoTankFlowData.h>
#define UI_CLK 23
#define UI1_DAT 26
#define UI2_DAT 25
#define LED_PIN 19
#define NUM_LEDS 8 
#define OP_MODE 34
#define RTC_BATT_VOLT 36
#define LED_PIN 19
#define RELAY_PIN 32

uint8_t secondsSinceLastDataSampling=0;
PCF8563TimeManager  timeManager( Serial);
GeneralFunctions generalFunctions;
Esp32SecretManager secretManager(timeManager);

PanchoConfigData panchoConfigData;
PanchoTankFlowData panchoTankFlowData;
SeedlingMonitoringWifiManager wifiManager(Serial, timeManager, secretManager, panchoTankFlowData,panchoConfigData);
bool wifiActive=false;
bool apActive=false;
long requestTempTime=0;
JSONVar jsonData;
TM1637Display display1(UI_CLK, UI1_DAT);
TM1637Display display2(UI_CLK, UI2_DAT);
CRGB leds[NUM_LEDS];
long lastTimeUpdateMillis=0;
RTCInfoRecord currentTimerRecord;
#define TIME_RECORD_REFRESH_SECONDS 3
volatile bool clockTicked=false;
#define UNIQUE_ID_SIZE 8
#define RTC_CLK_OUT 18
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
int soilMoistureValue;
//String display1TempURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Indoor%20Temperature:Indoor%20Temperature%20Data";
String display1TempURL = "http://192.168.1.117/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Indoor%20Temperature:Indoor%20Temperature%20Data";

/********************************************************************/
// Data wire is plugged into pin 2 on the Arduino 
//#define ONE_WIRE_BUS 2 

#define SENSOR_INPUT_1 5
#define SENSOR_INPUT_3 35

/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(SENSOR_INPUT_1); 
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


//
// interrupt functions
//

struct DisplayData{
      int value;
      int dp;
    } displayData;

void IRAM_ATTR clockTick() {
	portENTER_CRITICAL_ISR(&mux);
	clockTicked=true;
	portEXIT_CRITICAL_ISR(&mux);
}

//
// end of interrupt functions
//

int processDisplayValue(String displayURL,struct DisplayData *displayData ){
  int value=0;
  bool debug=false;
 // Serial.print("getting data for ");
//  Serial.println(displayURL);
  
   String displayValue = wifiManager.getTeleonomeData(displayURL, debug);
 //   Serial.print("received ");
  //  Serial.print(displayValue);
      if(displayValue.indexOf("Error") > 0){
         value=9999;
      }else{
         jsonData = JSON.parse(displayValue);
        if(jsonData["Value Type"]==JSONVar("int")){
          auto val=(const char*)jsonData["Value"];
          if(val==NULL){
            value = (int)jsonData["Value"];
          }else{
            String s((const char*)jsonData["Value"]);
            value=s.toInt();
          }
           displayData->dp=-1;
            Serial.print("int value= ");
            Serial.println(value);
       }else if(jsonData["Value Type"]==JSONVar("double")){  
          auto val=(const char*)jsonData["Value"];
          if(val==NULL){
             double valueF = (double)jsonData["Value"];
              if (valueF == (int)valueF) {
                value=(int)valueF;
                displayData->dp=-1;
              }else{
                value=(int)(100*valueF);
                displayData->dp=1;
              }
          }else{
            String s((const char*)jsonData["Value"]);
            float valueF=s.toFloat();
            if (valueF == (int)valueF) {
                value=(int)valueF;
                displayData->dp=-1;
            }else{
                value=(int)(100*valueF);
                displayData->dp=1;
             }
          }
        }else{
          value=9997;
          displayData->dp=-1;
        }
      }
      displayData->value=value;

      return value;
}

void setup() {
  Serial.begin(115200 );

FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
 for(int i=0;i<NUM_LEDS;i++){
     leds[i] = CRGB(255, 255, 0);
  }
   FastLED.show();
  // put your setup code here, to run once:
  display1.setBrightness(0x0f);
  display2.setBrightness(0x0f);
  display1.clear();
  display2.clear();
  pinMode(RTC_CLK_OUT, INPUT_PULLUP);        // set up interrupt%20Pin
	digitalWrite(RTC_CLK_OUT, HIGH);    // turn on pullup resistors
	// attach interrupt%20To set_tick_tock callback on rising edge of INT0
	attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);
  timeManager.start();
	timeManager.PCF8563osc1Hz();

  sensors.begin();


wifiManager.start();
  String ssid = wifiManager.getSSID();
  String ipAddress="";
  if(ssid==""){
      for(int i=0;i<NUM_LEDS;i++){
			leds[i] = CRGB(255, 0, 0);
		}
		FastLED.show();
    wifiActive=false;
  }else{
    ipAddress = wifiManager.getIpAddress();  
    if(ipAddress=="0.0.0.0"){
      for(int i=0;i<NUM_LEDS;i++){
        leds[i] = CRGB(0, 255, 255);
      }
      FastLED.show();
      wifiManager.restartWifi();
      ipAddress = wifiManager.getIpAddress();
      if(ipAddress=="0.0.0.0"){
        for(int i=0;i<NUM_LEDS;i++){
          leds[i] = CRGB(255, 0, 0);
        }
      }else{
        for(int i=0;i<NUM_LEDS;i++){
          leds[i] = CRGB(0, 0, 0);
        }
        leds[0] = CRGB(0, 0, 255);
        wifiActive=true;
      }
      FastLED.show();
    }else{
      for(int i=0;i<NUM_LEDS;i++){
        leds[i] = CRGB(0, 0, 0);
      }
      leds[0] = CRGB(0, 0, 255);
      wifiActive=true;
      FastLED.show();
    }
  }
  


    uint8_t ipi;
    for(int i=0;i<4;i++){
      ipi = GeneralFunctions::getValue(ipAddress, '.', i).toInt();
      display1.showNumberDec(ipi, false); 
      delay(1000);
    }

	pinMode(RTC_BATT_VOLT, INPUT);
	pinMode(OP_MODE, INPUT_PULLUP);
	pinMode(SENSOR_INPUT_3, INPUT);
	pinMode(RELAY_PIN,OUTPUT);


  display1.showNumberDec(0, false);
  display2.showNumberDec(0, false);
  requestTempTime = millis(); 
}

void loop() {
  // put your main code here, to run repeatedly:
  if(clockTicked){
		portENTER_CRITICAL(&mux);
		clockTicked=false;
		portEXIT_CRITICAL(&mux);
		currentTimerRecord  = timeManager.now();
  }

    
  if(currentTimerRecord.second==4 || currentTimerRecord.second==19|| currentTimerRecord.second==34|| currentTimerRecord.second==49){
    leds[1] = CRGB(0, 255, 0);
    leds[2] = CRGB(0, 0, 0);
    leds[3] = CRGB(0, 0, 0);
    FastLED.show();
    const uint8_t te[] = {
     SEG_D | SEG_E | SEG_F | SEG_G,  // t
      SEG_A | SEG_D | SEG_E | SEG_F| SEG_G  // E
    };

     sensors.requestTemperatures(); // Send the command to get temperature readings
    requestTempTime = millis(); 
    delay(100);
    /********************************************************************/
    //Serial.print("Temperature is: "); 
   

  display1.setSegments(te, 2, 0);
 // if(millis()-requestTempTime>800){
    float temp = sensors.getTempCByIndex(0);
    if(temp>0){
      panchoTankFlowData.flowRate=temp;
      //Serial.println(temp);
      int tempi = (int)(temp*100);
      display2.showNumberDecEx(tempi, (0x80 >> 1), false);
    }
 // }

  }else if(currentTimerRecord.second==8 || currentTimerRecord.second==23|| currentTimerRecord.second==38 || currentTimerRecord.second==52){
     leds[1] = CRGB(0, 255, 0);
     leds[2] = CRGB(0, 255, 0);
     leds[3] = CRGB(0, 0, 0);
    FastLED.show();
  //  digitalWrite(RELAY_PIN,HIGH);

  }else if(currentTimerRecord.second==12 || currentTimerRecord.second==27|| currentTimerRecord.second==41 || currentTimerRecord.second==56){
     leds[1] = CRGB(0, 255, 0);
     leds[2] = CRGB(0, 255, 0);
     leds[3] = CRGB(0, 255, 0);
     FastLED.show();
      const uint8_t so[] = {
        SEG_A | SEG_F| SEG_G | SEG_C | SEG_D,  // S
          SEG_G | SEG_C | SEG_D | SEG_E  // o
      };
      soilMoistureValue = analogRead(SENSOR_INPUT_3);
      display1.setSegments(so, 2, 0);
      display2.showNumberDecEx(soilMoistureValue, false);
   //   digitalWrite(RELAY_PIN,LOW);
  }
  if(currentTimerRecord.second==15 || currentTimerRecord.second==30|| currentTimerRecord.second==45){
     

    for(int i=1;i<NUM_LEDS;i++){
        leds[i] = CRGB(255, 0, 255);
      }
      FastLED.show();

    int value1 = processDisplayValue(display1TempURL,&displayData);
    panchoTankFlowData.flowRate2=value1/100.0;
    if(displayData.dp>0){
      display2.showNumberDecEx(value1, (0x80 >> displayData.dp), false);
    }else{
      display2.showNumberDec(value1, false);
    }
    delay(100);
    

    
    const uint8_t tr[] = {
     SEG_F | SEG_E| SEG_D| SEG_G ,  // t
      SEG_G | SEG_E
      }; // r
  
   
    display1.setSegments(tr, 2, 0);

    
    for(int i=1;i<NUM_LEDS;i++){
        leds[i] = CRGB(0, 0, 0);
      }
      FastLED.show();
  }

  if( Serial.available() != 0) {
		String command = Serial.readString();
    Serial.print(F("command="));
    Serial.println(command);
		if(command.startsWith("Ping")){
			Serial.println(F("Ok-Ping"));
    }else if(command.startsWith("ConfigWifiSTA")){
      //ConfigWifiSTA#ssid#password
      //ConfigWifiSTA#MainRouter24##Build4SolarPowerVisualizer#
      String ssid = generalFunctions.getValue(command, '#', 1);
      String password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);
      bool staok = wifiManager.configWifiSTA(ssid,password, hostname);
      if(staok){
          leds[0] = CRGB(0, 0, 255); 
      }else{
        leds[0] = CRGB(255, 0, 0); 
      }
      FastLED.show();
      Serial.println("Ok-ConfigWifiSTA");

		}
  }
}
