#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "MAX30105.h"

#include "heartRate.h"


// BLE server name
#define bleServerName "ESP32_BLE_SERVER"

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

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

// MAX30102 variables
MAX30105 particleSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

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
  // if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
    // {
    //   Serial.println("MAX30102 was not found. Please check wiring/power. ");
    //   while (1);
    // }
    // Serial.println("Place your index finger on the sensor with steady pressure.");

    // particleSensor.setup(); //Configure sensor with default settings
    // particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
    // particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
    //

  Serial.println("MQ3 is warming up...");
  delay(120000);  //2 min warm up time

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
        saturation = getRandomNumber(90, 100); // TODO: remove this line
        static char saturationTemp[6];
        dtostrf(saturation, 6, 2, saturationTemp);
        //Set saturation Characteristic value and notify connected client
        saturationCharacteristics.setValue(saturationTemp);
        saturationCharacteristics.notify();

        pulse = getRandomNumber(60, 100); // TODO: remove this line
        static char pulseTemp[6];
        dtostrf(pulse, 6, 2, pulseTemp);
        //Set pulse Characteristic value and notify connected client
        pulseCharacteristics.setValue(pulseTemp);
        pulseCharacteristics.notify();

        // Print the sensors values
        Serial.println("==================================");
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" ºC");
        Serial.print("Saturation: ");
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
        alcohol = analogRead(MQ3analogIn); // TODO: uncomment this line
        static char alcoholTemp[6];
        dtostrf(alcohol, 6, 2, alcoholTemp);
        //Set alcohol Characteristic value and notify connected client
        alcoholCharacteristics.setValue(alcoholTemp);
        alcoholCharacteristics.notify(); 

        Serial.println("==================================");  
        Serial.print("Alcohol: ");
        Serial.print(alcohol);
        // Serial.println(" BAC");
        Serial.println("==================================");
      } else {
        digitalWrite(ledPin, LOW);
        measuringAlcohol = false;        
      }
      //
      // long irValue = particleSensor.getIR();

      // if (checkForBeat(irValue) == true)
      // {
      //   //We sensed a beat!
      //   long delta = millis() - lastBeat;
      //   lastBeat = millis();

      //   beatsPerMinute = 60 / (delta / 1000.0);

      //   if (beatsPerMinute < 255 && beatsPerMinute > 20)
      //   {
      //     rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      //     rateSpot %= RATE_SIZE; //Wrap variable

      //     //Take average of readings
      //     beatAvg = 0;
      //     for (byte x = 0 ; x < RATE_SIZE ; x++)
      //       beatAvg += rates[x];
      //     beatAvg /= RATE_SIZE;
      //   }
      // }

      // Serial.print("IR=");
      // Serial.print(irValue);
      // Serial.print(", BPM=");
      // Serial.print(beatsPerMinute);
      // Serial.print(", Avg BPM=");
      // Serial.print(beatAvg);

      // if (irValue < 50000)
      //   Serial.print(" No finger?");

      // Serial.println();
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
