/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

/* Fill-in your Template ID (only if using Blynk.Cloud) */
#define BLYNK_TEMPLATE_ID           "TMPL2A1J8oFVM" // FILL THIS OUT
#define BLYNK_TEMPLATE_NAME         "MQTT Demo" // FILL THIS OUT
#define BLYNK_AUTH_TOKEN            "sXekAEwjlrlkeX8_PcLVPeIevhL-kYyo" // FILL THIS OUT


// Your WiFi credentials.
// Set password to "" for open networks.
#define MY_SSID                     "LAPTOP-KEMSQLIL 3120" // FILL THIS OUT
#define MY_PASSWORD                 "32n507[C" // FILL THIS OUT

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = BLYNK_AUTH_TOKEN;

#define Window 33
#define Curtain 38
#define Fan 1
#define Door 3

void setup()
{
  // Debug console
  Serial.begin(9600);
  Blynk.begin(auth, MY_SSID, MY_PASSWORD);
  pinMode(Window, OUTPUT);
  pinMode(Curtain, OUTPUT);
  pinMode(Fan, OUTPUT);
  pinMode(Door, OUTPUT);

  digitalWrite(Window,LOW); 
  digitalWrite(Curtain,LOW); 
  digitalWrite(Fan,LOW); 
  digitalWrite(Door,LOW); 
}

void loop()
{
  Blynk.run();
}

BLYNK_WRITE(V3){
  if(param.asInt() == 0)
  {
    digitalWrite(Curtain,LOW); 
  }
  else if (param.asInt() == 1)
  {
    digitalWrite(Curtain,HIGH); 
  }
}

BLYNK_WRITE(V4){
  if(param.asInt() == 0)
  {
    digitalWrite(Window,LOW); 
  }
  else if (param.asInt() == 1)
  {
    digitalWrite(Window,HIGH); 
  }
}

BLYNK_WRITE(V6){
  if(param.asInt() == 0)
  {
    digitalWrite(Fan,LOW); 
  }
  else if (param.asInt() == 1)
  {
    digitalWrite(Fan,HIGH); 
  }
}

BLYNK_WRITE(V7){
  if(param.asInt() == 0)
  {
    digitalWrite(Door,LOW); 
  }
  else if (param.asInt() == 1)
  {
    digitalWrite(Door,HIGH); 
  }
}

