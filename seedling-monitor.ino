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
#include "DHTesp.h" 
#include <SeedlingMonitoringData.h>



#define dhtPin 4     
#define UI_CLK 23
#define UI1_DAT 26
#define UI2_DAT 25
#define LED_PIN 19
#define NUM_LEDS 8 
#define OP_MODE 34
#define RTC_BATT_VOLT 36
#define LED_PIN 19
#define RELAY_PIN 32

bool loraActive = false;
bool opmode = false;

DHTesp dht;
uint8_t secondsSinceLastDataSampling=0;
PCF8563TimeManager  timeManager( Serial);
GeneralFunctions generalFunctions;
Esp32SecretManager secretManager(timeManager);

SeedlingMonitorData seedlingMonitorData;


SeedlingMonitoringWifiManager wifiManager(Serial, timeManager, secretManager,seedlingMonitorData);
bool readDHT=false;

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






struct DisplayData{
      int value;
      int dp;
    } displayData;
    
//
// interrupt functions
//



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



bool getTemperature() {
	// Reading temperature for humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht.getTempAndHumidity();
	// Check if any reads failed and exit early (to try again).
	if (dht.getStatus() != 0) {
		Serial.println("DHT11 error status: " + String(dht.getStatusString()));
		return false;
	}

  seedlingMonitorData.greenhouseTemp=newValues.temperature;
  seedlingMonitorData.greenhouseHum=newValues.humidity;
  seedlingMonitorData.heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
  seedlingMonitorData.dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
  Serial.println(" T:" + String(newValues.temperature) + " H:" + String(newValues.humidity) + " I:" + String(seedlingMonitorData.heatIndex) + " D:" + String(seedlingMonitorData.dewPoint) );
	return true;
}
void setup() {
  Serial.begin(115200 );
  dht.setup(dhtPin, DHTesp::DHT22);

	Serial.println("DHT initiated");
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


 const uint8_t lora[] = {
    SEG_F | SEG_E | SEG_D,          // L
    SEG_E | SEG_G | SEG_C | SEG_D , // o
    SEG_E | SEG_G  , // r
    SEG_A | SEG_B | SEG_C | SEG_E | SEG_F| SEG_G  // A
  };

  const uint8_t on[] = {
    SEG_E | SEG_G | SEG_C | SEG_D,  // o
    SEG_C | SEG_E | SEG_G          // n    
  };
  
   const uint8_t off[] = {
    SEG_E | SEG_G | SEG_C | SEG_D,  // o
     SEG_A | SEG_G | SEG_E | SEG_F,  // F
     SEG_A | SEG_G | SEG_E | SEG_F  // F
  };

   display1.setSegments(lora, 4, 0);

for(int i=0;i<NUM_LEDS;i++){
     leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();

  if (true){//!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    
    leds[1] = CRGB(255, 0, 0);
    display2.setSegments(off, 3, 0);
  } else {
    Serial.println("Starting LoRa worked!");
    leds[1] = CRGB(0, 0, 255);
    display2.setSegments(on, 2, 0);
    loraActive = true;
  }
  delay(2000);
  FastLED.show();

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


  opmode = digitalRead(OP_MODE);
  seedlingMonitorData.opMode = opmode;
  if(loraActive){
    leds[1] = CRGB(0, 0, 255);
  }else{
    leds[1] = CRGB(0, 0,0);    
  }

  display1.showNumberDec(0, false);
  display2.showNumberDec(0, false);
  requestTempTime = millis(); 
  getTemperature();
  readDHT=true;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(clockTicked){
		portENTER_CRITICAL(&mux);
		clockTicked=false;
		portEXIT_CRITICAL(&mux);
		currentTimerRecord  = timeManager.now();
  }

  seedlingMonitorData.secondsTime=timeManager.getCurrentTimeInSeconds(currentTimerRecord);
if(currentTimerRecord.second==0){
  for(int i=2;i<NUM_LEDS;i++){
      leds[i] = CRGB(0, 0, 0);
    }
    readDHT=false;
    leds[2] = CRGB(0, 255, 0);
    FastLED.show();
    const uint8_t st[] = {
       SEG_A | SEG_F | SEG_G | SEG_C| SEG_D,//S
     SEG_D | SEG_E | SEG_F | SEG_G  // t
    };
     sensors.requestTemperatures(); // Send the command to get temperature readings
    requestTempTime = millis(); 
    delay(100);
   
    display1.setSegments(st, 2, 0);
    seedlingMonitorData.soilTemperature = sensors.getTempCByIndex(0);
    if(seedlingMonitorData.soilTemperature>0){
      int tempi = (int)(seedlingMonitorData.soilTemperature*100);
      display2.showNumberDecEx(tempi, (0x80 >> 1), false);
    }
 
  }else if(currentTimerRecord.second==10){
     
     leds[3] = CRGB(0, 255, 0);
     FastLED.show();
      const uint8_t sh[] = {
        SEG_A | SEG_F| SEG_G | SEG_C | SEG_D,  // S
          SEG_G | SEG_B | SEG_C | SEG_E | SEG_F // H
      };
      seedlingMonitorData.soilMoisture = analogRead(SENSOR_INPUT_3);
      display1.setSegments(sh, 2, 0);
      display2.showNumberDecEx(seedlingMonitorData.soilMoisture, false);
      readDHT=false;
  }else if(currentTimerRecord.second==20 && !readDHT ){
    leds[4] = CRGB(0, 255, 0);
    FastLED.show();
    if( !readDHT){
        getTemperature();
        readDHT=true;
    }
  
    const uint8_t t[] = {
        SEG_D | SEG_E | SEG_F | SEG_G
      };
      
      display1.setSegments(t, 1, 0);
      
      int tempi = (int)(seedlingMonitorData.greenhouseTemp*100);
      display2.showNumberDecEx(tempi, (0x80 >> 1), false);

  }else if(currentTimerRecord.second==30){
    leds[5] = CRGB(0, 255, 0);
    FastLED.show();
    
    const uint8_t hu[] = {
        SEG_B| SEG_C | SEG_E | SEG_F | SEG_G,
        SEG_B| SEG_C | SEG_E | SEG_F | SEG_D
      };
      display1.setSegments(hu, 2, 0);
      int hi = (int)(seedlingMonitorData.greenhouseHum*100);
      display2.showNumberDecEx(hi, (0x80 >> 1), false);
      
      
  }else if(currentTimerRecord.second==40){
    leds[6] = CRGB(0, 255, 0);
    FastLED.show();
    const uint8_t tr[] = {
     SEG_F | SEG_E| SEG_D| SEG_G ,  // t
      SEG_G | SEG_E //r
      }; 
  
    display1.setSegments(tr, 2, 0);
    int value1 = processDisplayValue(display1TempURL,&displayData);
    seedlingMonitorData.roomTemperature=value1/100.0;
    if(displayData.dp>0){
      display2.showNumberDecEx(value1, (0x80 >> displayData.dp), false);
    }else{
      display2.showNumberDec(value1, false);
    }
    delay(100);
  }else if(currentTimerRecord.second==50){
    leds[7] = CRGB(0, 255, 0);
    FastLED.show();

    
    const uint8_t hi[] = {
     SEG_F | SEG_E| SEG_B| SEG_C | SEG_G ,  // H
      SEG_F | SEG_E  //////////////// I
      }; // r
    display1.setSegments(hi, 2, 0);

    int hei = (int)(seedlingMonitorData.heatIndex*100);
    display2.showNumberDecEx(hei, (0x80 >> 1), false);
  }else if(currentTimerRecord.second==55){
    //leds[7] = CRGB(0, 255, 0);
   // FastLED.show();
    const uint8_t de[] = {
     SEG_B | SEG_C| SEG_D| SEG_E | SEG_G ,  // d
      SEG_F | SEG_E | SEG_A| SEG_D | SEG_G     //// E

      }; // r
    display1.setSegments(de, 2, 0);

    int dei = (int)(seedlingMonitorData.dewPoint*100);
    display2.showNumberDecEx(dei, (0x80 >> 1), false);
  }

  if( Serial.available() != 0) {
		String command = Serial.readString();
    Serial.print(F("command="));
    Serial.println(command);
		if(command.startsWith("Ping")){
			Serial.println(F("Ok-Ping"));
    }else if(command.startsWith("ConfigWifiSTA")){
      //ConfigWifiSTA#ssid#password
      //ConfigWifiSTA#MainRouter24##SeedlingMoinitor#
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
