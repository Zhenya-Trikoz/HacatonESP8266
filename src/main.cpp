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

int pinData;

char auth[] = "1vf_k6tCKCDaR1eAskDCKTRfKQkB0zzy";
// char auth[] = "JaHzq_NqiDuW05l4OdSWtJMOFL6jFrzC";
int ledGPS = 12;
int bazzer = 14;
int ledGren = 13;
int ledRed = 15;
int buttonTemperature = 16;
int buttonOff = 2; //кнопка выкл светодиода gps

boolean ind = 1;      //переменная флаг для темпер модуля
boolean flag = false; //переменная флаг для обозначения вкл кнопки с Blynk, вычисляем координаты и находиться ли об*ект в диапазоне 5 метров

void setup() //эти команды выполнятся только один раз при старте системы
{
  Serial.begin(9600);                           // See the connection status in Serial Monitor
  Blynk.begin(auth, "mikrot2", "Sparta130855"); //подключаем свою WIFI сеть
  bme.begin(0x76);                              //инициализация датчика температуры, задаем адрес
  Serial.println();
  pinMode(ledGPS, OUTPUT); //объявляем пин как выход
  pinMode(bazzer, OUTPUT);
  pinMode(ledGren, OUTPUT);
  pinMode(ledRed, OUTPUT);

  pinMode(buttonTemperature, INPUT); // задаєм значення пинов для кнопок
  pinMode(buttonOff, INPUT);

  digitalWrite(ledGPS, LOW);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGren, LOW);
}

BLYNK_WRITE(V1) //кнопка на виртуальном пине V1 с приложения Blynk
{
  pinData = param.asInt();
  if (pinData == 1)
  {
    flag = true;
  }
}

BLYNK_WRITE(V2) //gps sensor на виртуальном пине V2 принимает  широту, долготу
{
  if (flag == true) // после нажатия кнопки выполняем поиск gps данных
  {
    float distance;

    pinData = param.asInt();
    GpsParam gps(param);

    Serial.print("Lat: ");
    Serial.println(gps.getLat(), 8);
    Serial.print("Lon: ");
    Serial.println(gps.getLon(), 8);
    Serial.println();
    //подсчет по формуле Гаверсина
    //метод определения расстояния между двумя gps координатами
    double x1 = gps.getLat(); //считываем широту
    double y1 = gps.getLon(); //долготу

    double x2 = 51.5048056; // координаты центра этого помещения
    double y2 = 31.3343690;

    double dLat = (x2 - x1) * M_PI / 180.0; //перевод с градусов в радианы
    double dLon = (y2 - y1) * M_PI / 180.0;

    x1 = (x1)*M_PI / 180.0;
    x2 = (x2)*M_PI / 180.0;

    double a = pow(sin(dLat / 2), 2) +
               pow(sin(dLon / 2), 2) *
                   cos(x1) * cos(x2);

    double rad = 6371; // радиус Земли

    double c = 2 * asin(sqrt(a));

    distance = rad * c;

    Serial.println(distance, 6); //дистанція между прибором и телефоном обьекта

    if (distance <= 0.005) // если об*ект находиться на расстоянии 5 метров и ближе
    {
      Serial.println("Досягнуто місце призначення"); // он на месте
      digitalWrite(ledGPS, HIGH);
      tone(bazzer, 200); //включаем на 500 Гц
      delay(400);
      tone(bazzer, 400); //включаем на 1000 Гц
      delay(500);
      tone(bazzer, 600); //включаем на 500 Гц
      delay(600);
      tone(bazzer, 1000); //включаем на 1000 Гц
      delay(500);
    }
    else
    {
      Serial.println("Місце призначення не досягнуто");
      digitalWrite(ledGPS, LOW);
    }
    noTone(bazzer); //выключаем звук пищалки
    flag = false;   // затираем флаг
  }
}

void printValues()
{
  ind = 0;
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature()); //считываем температуру
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F); //отображаем атмосферное давление в гПа
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = "); //высота
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = "); //влажность
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}

void loop()
{
  Blynk.run(); //запуск Blynk

  if (digitalRead(buttonOff) == LOW)
  {
    noTone(bazzer);
    digitalWrite(ledGPS, LOW);
  }

  if (digitalRead(buttonTemperature) == HIGH) /*высчитываем один раз температуру воздуха если кнопка для доп модуля нажата*/
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
}