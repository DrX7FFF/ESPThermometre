// https://randomnerdtutorials.com/esp8266-adc-reading-analog-values-with-nodemcu/
#include "DHTesp.h" // library DHT sensor library for ESPx https://github.com/beegee-tokyo/DHTesp
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>        //https://github.com/tzapu/WiFiManager
#include <ArduinoOTA.h>

#define SYSTEMNAME "TEST"
#define PORT 8888
#define DELTATEMP 0.2
#define DELTASENSOR 20
#define RESENDMSEC 300000
const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0

WiFiUDP udp;
DHTesp dht;
IPAddress broadcastIP;

void setup() {
  Serial.begin(115200);

  WiFiManager wifiManager;
//  wifiManager.resetSettings();
  wifiManager.autoConnect(SYSTEMNAME);

  ArduinoOTA.setHostname(SYSTEMNAME);
  ArduinoOTA.begin();

  dht.setup(4, DHTesp::DHT22); // Connect DHT sensor to GPIO 4

  IPAddress myIP = WiFi.localIP();
  IPAddress myMask = WiFi.subnetMask();
  Serial.println(myIP);
  Serial.println(myMask);
  for(int i = 0; i<4; i++)
    broadcastIP[i] = (myIP[i] & myMask[i]) | (~ myMask[i]);
  Serial.println(broadcastIP);
  Serial.printf("UDP port : %d/r/n",PORT);

  udp.begin(PORT);
}

void loop() {
  float lastSendTemp = 2000;
  int lastSendSensor = 0;
  unsigned long lastSendMillis;
  
  ArduinoOTA.handle();  
  delay(dht.getMinimumSamplingPeriod());
  
  float temp = dht.getTemperature();
  int sensor = analogRead(analogInPin);

  if ((fabs(lastSendTemp - temp) > DELTATEMP) || 
      (abs(lastSendSensor - sensor) > DELTASENSOR) ||
      (millis() - lastSendMillis > RESENDMSEC)) {
    lastSendTemp = temp;
    lastSendSensor = sensor;
    lastSendMillis = millis();

    float tempAna = sensor*0.1-30.4;

    udp.beginPacket(broadcastIP,PORT);
    udp.printf("{\"mes1\":%3.1f,\"mes2\":%d,\"mes3\":%3.1f}", temp, sensor, tempAna);
    udp.endPacket();
  }
}
