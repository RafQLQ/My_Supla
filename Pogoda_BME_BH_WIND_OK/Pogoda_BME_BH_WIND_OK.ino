
/**
 * Supla.org NodeMCU WiFi minimal example
 * Author: Programistyk - Kamil Kaminski <kamil@programistyk.pl>
 * 
 * This example shows how to configure SuplaDevice for building for NodeMCU within Arduino IDE
 */


#include <srpc.h>
#include <log.h>
#include <eh.h>
#include <proto.h>
#include <IEEE754tools.h>
// We define our own ethernet layer
#define SUPLADEVICE_CPP
#include <SuplaDevice.h>
#include <DHT.h>
#include <lck.h>

#include <BH1750FVI.h>  // I2C
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);

#include <WiFiClient.h>
#include <ESP8266WiFiType.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiServer.h>
#include <ESP8266WiFiGeneric.h>
#include <WiFiClientSecure.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiSTA.h>
#include <WiFiUdp.h>
#include <Bounce2.h>
WiFiClient client;



#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define WindImpuls 13 // GPIO 13 - nr wejścia impulsatora  anemometru
#define SEALEVELPRESSURE_HPA (1013.25) // cisnienie na poziomie morza 

Adafruit_BME280 bme; // I2C


double diameter = 2.75;
double mph; //Utworzenie zmiennej mile/godzinÄ
double kmh; //Utowrzenie zmiennej km/h
double ms; // Utworzenie zmiennej m/s 
double pomiar[3];
int licz_pomiar;

int rawVoltage ;
float voltage;

  
// Setup Supla connection
const char* ssid     = "SSID";
const char* password = "PASS";

//***************************************************************************************
double get_pressure(int channelNumber, double pressure) {
   
 pressure = bme.readPressure() / 100.0F;
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure()/100);
    Serial.println(" hPa");
   return  pressure; 

   
}
//***************************************************************************************
double  get_wind(int channelNumber, double last_val){
  
    double t = -275;  
    t=ms;
    return t;
}
//***************************************************************************************

void get_temperature_and_humidity(int channelNumber, double *temp, double *humidity) {
   *temp = bme.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");
    
    *humidity = bme.readHumidity();
    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
    if ( isnan(*temp) || isnan(*humidity) ) {
      *temp = -275;
      *humidity = -1;
    }
  
   
}
//***************************************************************************************
double get_distance(int channelNumber, double t) {

 // float lux = analogRead(LIGHTSENSORPIN);             //TEMT6000
   uint16_t lux = abs(LightSensor.GetLightIntensity());     //BH1750
    t = abs(lux);
    Serial.print("Natezenie swiatla = ");
    Serial.print(LightSensor.GetLightIntensity());
    return t;

} //void
//***************************************************************************************
//Ĺrednica anemometru

// Odczyt obrotĂłw (RPM)
int half_revolution_time = 0; //Utworzenie zmiennej przechowujÄcej
double rpm = 0; //Utworzenie zmiennej RPM (obroty)
unsigned long lastmillis = 0; //Utworzenie zmiennej long lastmilis

void rpm_fan() {
  unsigned long static last_event = 0;
  if (millis() - last_event < 5) {   //debouncing
    return;
  }
  half_revolution_time = (millis() - last_event);
  last_event = millis();
}
//***************************************************************************************


void setup() {
  Serial.begin(115200);
  LightSensor.begin();  
  delay(10);
  pinMode(WindImpuls, INPUT_PULLUP);   
  attachInterrupt(digitalPinToInterrupt(WindImpuls), rpm_fan, FALLING); 
  Wire.begin(4,5);
  //bme.begin(0x76);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  
  // ﻿Replace the falowing GUID
    char GUID[SUPLA_GUID_SIZE] = {0x4A,0x09,0x0E,0xBC,0x38,0x0D,0x93,0xE8,0xF5,0x25,0x39,0x95,0xE2,0x84,0xA7,0x11};

  // ﻿with GUID that you can retrieve from https://www.supla.org/arduino/get-guid

  // Ethernet MAC address
  uint8_t mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  /*
   * Having your device already registered at cloud.supla.org,
   * you want to change CHANNEL sequence or remove any of them,
   * then you must also remove the device itself from cloud.supla.org.
   * Otherwise you will get "Channel conflict!" error.
   */
    
  // CHANNEL0 - RELAY
   SuplaDevice.setName("R@F_Stacja_Pogodowa");


  
   SuplaDevice.addPressureSensor(); //Pressure
   SuplaDevice.addWindSensor(); // Wind m/s
   SuplaDevice.addDHT22();  // BME T i %
   SuplaDevice.addDistanceSensor(); // Natezenie swiatla
   
   SuplaDevice.setPressureCallback(&get_pressure);
   WiFi.softAPdisconnect(true);
  
  
  SuplaDevice.begin(GUID,              // Global Unique Identifier 
                    mac,               // Ethernet MAC address
                    "svrS.supla.org",  // SUPLA server address
                    777,                 // Location ID 
                    "ClouD_PASS");               // Location Password

}
//***************************************************************************************

void loop() {
 
  SuplaDevice.iterate();
  SuplaDevice.setPressureCallback(&get_pressure);
  
    if (millis() - lastmillis >= 1000) {
    //Aktualizuj co sekundÄ, bÄdzie to rĂłwnoznaczne z odczytem czÄstotliwoĹci (Hz)

    lastmillis = millis();          // Aktualizacja lastmillis
    
    noInterrupts();                   // W trakcie kalkulacji wyĹÄcz obsĹugÄ przerwaĹ
    if (half_revolution_time>0) rpm = (30000 / half_revolution_time) ;      
    interrupts() ; //PrzywrĂłÄ przerwania
    if (rpm>0) {
      mph = diameter / 5 * 3.14 * rpm * 60 / 5280;//Odczyt prÄdkoĹci wiatru w milach/godzinÄ
      mph = mph * 3.5; // Kalibracja bĹÄdu odczytu, wartoĹÄ naleĹźy dobraÄ we wĹasnym zakresie
      kmh = mph * 1.609;// Zamiana mil/godzine na km/h
      ms = kmh / 3.6;     
   Serial.print("Wind meter = ");
      Serial.print(ms);
      Serial.println(" m/s"); 
    }
       pomiar[licz_pomiar]=ms;  
      if ((pomiar[0]==pomiar[1]) and (pomiar[1]==pomiar[2]) and (pomiar[2]==pomiar[3])) {
          ms=0;      
      }
      licz_pomiar=licz_pomiar+1;    
      if (licz_pomiar==4) licz_pomiar=0;
  
  }

}
//***************************************************************************************

// Supla.org ethernet layer
    int supla_arduino_tcp_read(void *buf, int count) {
        _supla_int_t size = client.available();
       
        if ( size > 0 ) {
            if ( size > count ) size = count;
            return client.read((uint8_t *)buf, size);
        };
    
        return -1;
    };
    
    int supla_arduino_tcp_write(void *buf, int count) {
        return client.write((const uint8_t *)buf, count);
    };
    
    bool supla_arduino_svr_connect(const char *server, int port) {
          return client.connect(server, 2015);
    }
    
    bool supla_arduino_svr_connected(void) {
          return client.connected();
    }
    
    void supla_arduino_svr_disconnect(void) {
         client.stop();
    }
    
    void supla_arduino_eth_setup(uint8_t mac[6], IPAddress *ip) {

       // Serial.println("WiFi init");
        WiFi.begin(ssid, password);

        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
        //    Serial.print(".");
        }

        //Serial.print("\nlocalIP: ");
        //Serial.println(WiFi.localIP());
        //Serial.print("subnetMask: ");
        //Serial.println(WiFi.subnetMask());
        //Serial.print("gatewayIP: ");
        //Serial.println(WiFi.gatewayIP());
    }
//***************************************************************************************

SuplaDeviceCallbacks supla_arduino_get_callbacks(void) {
          SuplaDeviceCallbacks cb;
          
          cb.tcp_read = &supla_arduino_tcp_read;
          cb.tcp_write = &supla_arduino_tcp_write;
          cb.eth_setup = &supla_arduino_eth_setup;
          cb.svr_connected = &supla_arduino_svr_connected;
          cb.svr_connect = &supla_arduino_svr_connect;
          cb.svr_disconnect = &supla_arduino_svr_disconnect;        
          cb.get_temperature_and_humidity =get_temperature_and_humidity;
          cb.get_rgbw_value = NULL;
          cb.set_rgbw_value = NULL;
          cb.get_pressure = &get_pressure;
          cb.get_wind=&get_wind;
          cb.get_distance= get_distance;
          return cb;
}


