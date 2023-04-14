#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>


//BLE server name
#define bleServerName "ESP32_BLE_SERVER"


// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

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
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

// TODO:
void initSensors(){
  // if (!bme.begin(0x76)) {
  //   Serial.println("Could not find a valid BME280 sensor, check wiring!");
  //   while (1);
  // }
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
        // TODO: Read the sensors (get random number for now)
        float temperature = getRandomNumber(34, 40); // from LM35
        float saturation = getRandomNumber(90, 100); // from MAX30100
        float pulse = getRandomNumber(60, 100); // from MAX30100
        // TODO: Handle reading from alcohol sensor separately ?!
        float alcohol = getRandomNumber(0, 1); // from MQ-3

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
        Serial.print("Alcohol: ");
        Serial.print(alcohol);
        Serial.println(" BAC");
        Serial.println("==================================");
    
        //Notify temperature reading from sensor
        static char temperatureCTemp[6];
        dtostrf(temperature, 6, 2, temperatureCTemp);
        //Set temperature Characteristic value and notify connected client
        temperatureCelsiusCharacteristics.setValue(temperatureCTemp);
        temperatureCelsiusCharacteristics.notify();
        
        //Notify saturation reading from sensor
        static char saturationTemp[6];
        dtostrf(saturation, 6, 2, saturationTemp);
        //Set saturation Characteristic value and notify connected client
        saturationCharacteristics.setValue(saturationTemp);
        saturationCharacteristics.notify();

        //Notify pulse reading from sensor
        static char pulseTemp[6];
        dtostrf(pulse, 6, 2, pulseTemp);
        //Set pulse Characteristic value and notify connected client
        pulseCharacteristics.setValue(pulseTemp);
        pulseCharacteristics.notify();   

        //Notify alcohol reading from sensor
        static char alcoholTemp[6];
        dtostrf(alcohol, 6, 2, alcoholTemp);
        //Set alcohol Characteristic value and notify connected client
        alcoholCharacteristics.setValue(alcoholTemp);
        alcoholCharacteristics.notify();   
        
        lastTime = millis();
      }
  }
}
///////////////////////////////// LOOP END ///////////////////////////////////////////


// Function to get random number from given range
float getRandomNumber(int min, int max) {
    return (float)rand() / (float)RAND_MAX * (max - min) + min;
}
