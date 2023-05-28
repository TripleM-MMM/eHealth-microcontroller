#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "DFRobot_BloodOxygen_S.h"

#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#define I2C_ADDRESS    0x57
DFRobot_BloodOxygen_S_I2C MAX30102(&Wire ,I2C_ADDRESS);

// BLE server name
#define bleServerName "ESP32_BLE_SERVER"

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 10000;

// Sensor measures variables
float temperature; // from LM35
float saturation; // from MAX30100
float pulse; // from MAX30100
float alcohol; // from MQ3

// LM35 variables
const int LM35analogIn = A6;
int LM35RawValue= 0;
float LM35Voltage = 0;
float LM35tempC = 0;

// Callback routine is executed when a pulse is detected
void onBeatDetected() {
    Serial.println("♥ Beat!");
}

// MQ3 variables
const int MQ3analogIn = A7;

// Pushbutton to start measuring alcohol
const int pushButtonPin = 15;
unsigned long lastTimeAlcoholMeasured = 0;
unsigned long timeToMeasureAlcohol = 5000;

// Led to show measuring alcohol
const int ledPin = 2;
bool measuringAlcohol = false;
int alcoMeasures = 0;
float alcoholSum = 0;

bool deviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"

// Temperature Characteristic and Descriptor
BLECharacteristic temperatureCelsiusCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor temperatureCelsiusDescriptor(BLEUUID((uint16_t)0x2902));

// Saturation Characteristic and Descriptor
BLECharacteristic saturationCharacteristics("ca73b3ba-39f6-4ab3-91ae-186dc9577d99", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor saturationDescriptor(BLEUUID((uint16_t)0x2903));

// Pulse Characteristic and Descriptor
BLECharacteristic pulseCharacteristics("8d8c66b5-efc7-4cd0-8c13-04f13f19858b", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor pulseDescriptor(BLEUUID((uint16_t)0x2904));

// Alcohol Characteristic and Descriptor
BLECharacteristic alcoholCharacteristics("ad2be602-2e39-440c-8777-708d62125347", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor alcoholDescriptor(BLEUUID((uint16_t)0x2905));

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected!");
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected!");
  }
};

void initSensors(){
  Serial.println("Initializing pulse oximeter...");
  while (false == MAX30102.begin())
  {
    Serial.println("init fail!");
    delay(1000);
  }
  Serial.println("init success!");
  Serial.println("start measuring...");
  MAX30102.sensorStartCollect();

  Serial.println("MQ3 is warming up...");
  //delay(120000);  //2 min warm up time

  // Declare pushButtonPin as digital input 
  pinMode(pushButtonPin, INPUT);
  pinMode(ledPin, OUTPUT);
}

///////////////////////////////// SETUP BEGIN ////////////////////////////////////////
void setup() {
    Serial.begin(115200);
    Serial.println("SET UP!");

    initSensors();

    // Create the BLE Device
    BLEDevice::init(bleServerName);

    // Create the BLE Server
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *service = pServer->createService(SERVICE_UUID);

    // Create BLE Characteristics and Create a BLE Descriptor
    // Temperature
    service->addCharacteristic(&temperatureCelsiusCharacteristics);
    temperatureCelsiusDescriptor.setValue("Temperature Celsius");
    temperatureCelsiusCharacteristics.addDescriptor(&temperatureCelsiusDescriptor);

    // Saturation
    service->addCharacteristic(&saturationCharacteristics);
    saturationDescriptor.setValue("Saturation");
    saturationCharacteristics.addDescriptor(new BLE2902());

    // Pulse
    service->addCharacteristic(&pulseCharacteristics);
    pulseDescriptor.setValue("Pulse");
    pulseCharacteristics.addDescriptor(new BLE2902());

    // Alcohol
    service->addCharacteristic(&alcoholCharacteristics);
    alcoholDescriptor.setValue("Alcohol");
    alcoholCharacteristics.addDescriptor(new BLE2902());
    
    // Start the service
    service->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");
}
///////////////////////////////// SETUP END //////////////////////////////////////////

///////////////////////////////// LOOP BEGIN /////////////////////////////////////////
void loop() {
    if (deviceConnected) {
      if ((millis() - lastTime) > timerDelay) {
        //////////////// TEMPERATURE ////////////////
        //temperature = getRandomNumber(34, 40); // TODO: remove this line
        temperature = readTemperatureFromLM35(); // TODO: uncomment this line
        static char temperatureCTemp[6];
        dtostrf(temperature, 6, 2, temperatureCTemp);
        //Set temperature Characteristic value and notify connected client
        temperatureCelsiusCharacteristics.setValue(temperatureCTemp);
        temperatureCelsiusCharacteristics.notify();

        //////////////// SATURATION and PULSE ////////////////
        MAX30102.getHeartbeatSPO2();

        //saturation = getRandomNumber(90, 100); // TODO: remove this line
        saturation = MAX30102._sHeartbeatSPO2.SPO2;
        if (saturation != -1) {
          static char saturationTemp[6];
          dtostrf(saturation, 6, 2, saturationTemp);
          //Set saturation Characteristic value and notify connected client
          saturationCharacteristics.setValue(saturationTemp);
          saturationCharacteristics.notify();
        }

        //pulse = getRandomNumber(60, 100); // TODO: remove this line
        pulse = MAX30102._sHeartbeatSPO2.Heartbeat;
        if (pulse != -1) {
          static char pulseTemp[6];
          dtostrf(pulse, 6, 2, pulseTemp);
          //Set pulse Characteristic value and notify connected client
          pulseCharacteristics.setValue(pulseTemp);
          pulseCharacteristics.notify();
        }

        // Print the sensors values
        Serial.println("==================================");
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" ºC");
        Serial.print("Saturation (SPO2): ");
        Serial.print(saturation);
        Serial.println(" %");
        Serial.print("Pulse: ");
        Serial.print(pulse);
        Serial.println(" BPM");
        Serial.println("==================================");  
        
        lastTime = millis();
      }
      //////////////// ALCOHOL ////////////////
      int pushButtonState = digitalRead(pushButtonPin);
      if (pushButtonState == HIGH && millis() - lastTimeAlcoholMeasured > timeToMeasureAlcohol) {
        lastTimeAlcoholMeasured = millis();     
        digitalWrite(ledPin, HIGH);
        measuringAlcohol = true;
        Serial.println("Now measuring alcohol until the LED is ON");     
      }
      if (millis() - lastTimeAlcoholMeasured <= timeToMeasureAlcohol && measuringAlcohol == true) {
        //alcohol = getRandomNumber(0, 1); // TODO: remove this line
        float tmpAlcohol = analogRead(MQ3analogIn); // TODO: uncomment this line
        Serial.println("===============");  
        Serial.print("Alcohol: ");
        Serial.println(tmpAlcohol);
        // Serial.println(" BAC");
        Serial.println("===============");
        alcoholSum += tmpAlcohol;
        alcoMeasures++;        
      } else {
        digitalWrite(ledPin, LOW);
        measuringAlcohol = false;        
        if (alcoMeasures != 0) {
          alcohol = alcoholSum / alcoMeasures;
          static char alcoholTemp[6];
          dtostrf(alcohol, 6, 2, alcoholTemp);
          //Set alcohol Characteristic value and notify connected client
          alcoholCharacteristics.setValue(alcoholTemp);
          alcoholCharacteristics.notify(); 

          Serial.println("==================================");  
          Serial.print("Average Alcohol: ");
          Serial.println(alcohol);
          // Serial.println(" BAC");
          Serial.println("==================================");
          alcoholSum = 0;
          alcoMeasures = 0;
        }
      }
  }
  delay(1000);
}
///////////////////////////////// LOOP END ///////////////////////////////////////////

float readTemperatureFromLM35() {
  LM35RawValue = analogRead(LM35analogIn);
  LM35Voltage = (LM35RawValue / 2048.0) * 3300;
  LM35tempC = LM35Voltage * 0.1;
  return LM35tempC;
}

float getRandomNumber(int min, int max) {
    return (float)rand() / (float)RAND_MAX * (max - min) + min;
}
