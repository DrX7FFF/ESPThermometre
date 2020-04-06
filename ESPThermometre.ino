// https://randomnerdtutorials.com/esp8266-adc-reading-analog-values-with-nodemcu/

//#include <ESP8266WiFi.h>
//#include <ESP8266WebServer.h>
//#include <WiFiManager.h>        //https://github.com/tzapu/WiFiManager

#define SYSTEMNAME "TEST"

const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0


void setup() {
  Serial.begin(115200);
  Serial.println("Start");

//  WiFiManager wifiManager;
//  wifiManager.autoConnect(SYSTEMNAME);

//  Serial.println("Ready");
}

ADC_MODE(ADC_VCC);

int sensorValue = 0;  // value read from the pot
int outputValue = 0;  // value to output to a PWM pin
float tempValue = 0;
void loop() {
  sensorValue = analogRead(analogInPin);
    // map it to the range of the PWM out
  outputValue = map(sensorValue, 0, 1024, 0, 255);
  tempValue = ((float)sensorValue) *11/64 - 70;
  // print the readings in the Serial Monitor
  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print("\t output = ");
  Serial.print(outputValue);
  Serial.print("\t Temp = ");
  Serial.print(tempValue);
  Serial.print("\t Alim = ");
  Serial.print(ESP.getVcc());
  Serial.println();
  delay(1000);
}
