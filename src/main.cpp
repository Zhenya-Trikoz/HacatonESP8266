#include "Arduino.h"
#define BLYNK_PRINT Serial // лог работы модуля в мониторе
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "math.h"
#define M_PI 3.141592653589793238462643
#define SEALEVELPRESSURE_HPA (1013.25)

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

Adafruit_BME280 bme;
unsigned long delayTime;

int pinData;

char auth[] = "1vf_k6tCKCDaR1eAskDCKTRfKQkB0zzy";

int ledGPS = 12;
int bazzer = 14;
int ledGren = 13;
int ledRed = 15;
int buttonTemperature = 16;
int buttonOff = 2;

boolean ind = 1;
boolean flag = false;

void setup()
{
  Serial.begin(9600);                           // See the connection status in Serial Monitor
  Blynk.begin(auth, "mikrot2", "Sparta130855"); //подключаем свою WIFI сеть
  //Blynk.begin(auth, "Darknet", "*D20021976K*"); //подключаем свою WIFI сеть
  pinMode(ledGPS, OUTPUT);
  pinMode(bazzer, OUTPUT); //объявляем пин как выход

  unsigned status = bme.begin(0x76);

  delayTime = 10;

  Serial.println();
  pinMode(ledGren, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(buttonTemperature, INPUT);
  pinMode(buttonOff, INPUT);
  digitalWrite(ledGPS, LOW);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGren, LOW);
}

BLYNK_WRITE(V1)
{
  pinData = param.asInt();
  if (pinData == 1)
  {
    flag = true;
  }
}
BLYNK_WRITE(V2)
{
  if (flag == true)
  {
    float distance;
    pinData = param.asInt();
    GpsParam gps(param);
    Serial.print("Lat: ");
    Serial.println(gps.getLat(), 8);
    Serial.print("Lon: ");
    Serial.println(gps.getLon(), 8);
    Serial.println();
    double x1 = gps.getLat();
    double y1 = gps.getLon();

    double x2 = 51.5048056;
    double y2 = 31.3343690;

    double dLat = (x2 - x1) * M_PI / 180.0;
    double dLon = (y2 - y1) * M_PI / 180.0;

    x1 = (x1)*M_PI / 180.0;
    x2 = (x2)*M_PI / 180.0;

    double a = pow(sin(dLat / 2), 2) +
               pow(sin(dLon / 2), 2) *
                   cos(x1) * cos(x2);
    double rad = 6371;
    double c = 2 * asin(sqrt(a));
    distance = rad * c;
    Serial.println(distance, 6);
    if (distance <= 0.005)
    {
      Serial.println("Досягнуто місце призначення");
      digitalWrite(ledGPS, HIGH);
      tone(bazzer, 200); //включаем на 500 Гц
      delay(400);
      tone(bazzer, 400); //включаем на 1000 Гц
      delay(500);
      tone(bazzer, 600); //включаем на 500 Гц
      delay(600);
      tone(bazzer, 1000); //включаем на 1000 Гц
      delay(500);
      //delay(10000);
    }
    else
    {
      Serial.println("Місце призначення не досягнуто");
      digitalWrite(ledGPS, LOW);
     
    }
    noTone(bazzer);
    flag = false;
  }
}

void printValues()
{
  ind = 0;
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}

void loop()
{
  Blynk.run();
  if (digitalRead(buttonOff) == LOW)
  {
    noTone(bazzer);
    digitalWrite(ledGPS, LOW);
  }
  // }
  if (digitalRead(buttonTemperature) == HIGH)
  {
    if (ind == 1)
    {
      printValues();
      if (bme.readTemperature() > 37)
      {
        digitalWrite(ledRed, HIGH);
      }
      else
      {
        digitalWrite(ledGren, HIGH);
      }
    }
  }
  else
  {
    ind = 1;
    digitalWrite(ledRed, LOW);
    digitalWrite(ledGren, LOW);
  }
  //digitalWrite(ledGPS, pinData);
  //  printValues();
}