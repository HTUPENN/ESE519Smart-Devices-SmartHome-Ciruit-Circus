#include <PubSubClient.h>
#include "NetworkHelpers.h"
#include <SPI.h>
#include <ArduinoJson.h> // JSON message pack
#include <string.h>

#define SS    33
#define SCK   36
#define MOSI  35
#define MISO  37

// (re)connect to the Blynk Cloud
void mqtt_connected(){
  // Publish some data
  mqtt.publish("ds/terminal", "Device connected\n");
  Serial.println("Device connected\n");
}

// Handle incoming datastream changes (remote-control)
void mqtt_handler(const String& topic, const String& value){
  Serial.print("Got ");       Serial.print(topic);
  Serial.print(", value: ");  Serial.println(value);

  if (topic == "downlink/ds/terminal") {
    String reply = String("Your command: ") + value;
    mqtt.publish("ds/terminal", reply.c_str());
  }
  // else{
  //   transmitJSON(topic, value);
  // }
}

// SPI transmit
// void transmitJSON(const String& sensor, const String& value) {

//   StaticJsonDocument<200> doc;

//   float parsedValue = value.toFloat();

//   doc["sensor"] = sensor;
//   doc["value"] = parsedValue;
    
//   char jsonBuffer[64];
//   serializeJson(doc, jsonBuffer);
    
//   // Start transaction
//   SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//   digitalWrite(SS, LOW);
//   delay(1);
    
//   // Send each byte of the JSON string
//   char* ptr = jsonBuffer;
//   while(*ptr) {
//     SPI.transfer(*ptr++);
//     delayMicroseconds(100);
//   }
    
//   // Send null terminator
//   SPI.transfer('\0');
    
//   digitalWrite(SS, HIGH);
//   SPI.endTransaction();
    
//   Serial.print("Sent: ");
//   Serial.println(jsonBuffer);
//   delay(100);
// }

// SPI receive
void receiveJSON(){ 
  static char buffer[64];
  int idx = 0;
  bool messageComplete = false;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SS, LOW);
  delay(1);
  // Read until complete JSON message is received
  while(idx < sizeof(buffer) - 1) {
    char received = SPI.transfer(0x00);
        
    // Start collecting at opening brace
    if(received == '{') {
        idx = 0;
        buffer[idx++] = received;
    }
    else if(idx > 0) {
      buffer[idx++] = received;
            
      // Check for end of JSON message
      if(received == '}') {
        messageComplete = true;
        break;
      }
    }
        
    delayMicroseconds(100);  // Small delay between bytes
  }
    
  digitalWrite(SS, HIGH);
  SPI.endTransaction();
    
  if(messageComplete) {
    buffer[idx] = '\0';
    Serial.print("Complete JSON: ");
    Serial.println(buffer);
        
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, buffer);
        
    if(!error) {
      String sensor = doc["sensor"];
      float value = doc["value"];
      //Serial.printf("Parsed - Sensor: %s, Value: %.2f\n", sensor, value);

      // Publish value
      String data = String(value);
      mqtt.publish(sensor.c_str(), data.c_str());
    } else {
      Serial.println("Failed to parse JSON");
    }
  }
    
  delay(100);  // Delay between transactions
}


void setup() {
  Serial.begin(115200);
  // Wait for serial monitor, up to 3 seconds
  while (!Serial && (millis() < 3000)) { delay(10); }
  delay(100);

  systemShowDeviceInfo();
    
  // SPI 
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);
  SPI.begin(SCK, MISO, MOSI, SS);
  digitalWrite(SS, HIGH);
  Serial.println("SPI Master Ready");
}

void loop() {
  EVERY_N_MILLIS(1000) {
    String uptime = String(millis() / 1000);
    mqtt.publish("ds/uptime", uptime.c_str());
  }

  EVERY_N_MILLIS(15000) {
    mqtt.publish("ds/rssi", String(WiFi.RSSI()).c_str());
  }

  // Keep WiFi and MQTT connection
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  } else if (!mqtt.connected()) {
    connectMQTT();
  } else {
    mqtt.loop();
  }

  receiveJSON();
  delay(200);
}