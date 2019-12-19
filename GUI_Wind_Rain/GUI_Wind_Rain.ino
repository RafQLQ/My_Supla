/*
Copyright (C) R@F QLQ
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/

#include <FS.h>       // ---- esp board manager 2.5.2 --- iwip Variant V2 higher Bandwidth
#include <SPI.h>
#include <SuplaDevice.h>
#include <io.h>
#include <Bounce2.h>
#include <WiFiManager.h>
#include <ArduinoJson.h> //--------- https://github.com/bblanchon/ArduinoJson/tree/v5.13.2 ------

#include <Wire.h>



extern "C"
{
#include "user_interface.h"
} 

//SoftwareSerial pzem;
//#define D0 16  //no internal pullup resistor
//#define D1  5
//#define D2  4
//#define D3  0  //must not be pulled low during power on/reset, toggles value during boot
//#define D4  2  //must not be pulled low during power on/reset, toggles value during boot
//#define D5 14
//#define D6 12
//#define D7 13
//#define D8 15  //must not be pulled high during power on/reset

//################## WIATR ########################################
#define WindImpuls 12 // D7 - nr wejścia impulsatora  anemometru
double diameter = 2.75;
double mph; //Utworzenie zmiennej mile/godzinÄ
double kmh; //Utowrzenie zmiennej km/h
double ms; // Utworzenie zmiennej m/s 
double pomiar[3];
int licz_pomiar;

//######################### DESZCZ #################################
const byte interruptPin = 13; // D6 - nr wejścia impulsatora  deszczomierza
const int interval = 500;
volatile unsigned long tiptime = millis();
double rainrate;
unsigned long curtime;

//#################################################################

int wificonfig_pin = 0; // D3
int C_W_state = HIGH; 
int last_C_W_state = HIGH;
unsigned long time_last_C_W_change = 0; 
long C_W_delay = 5000;               // ---------------------- config delay 5 seconds ---------------------------
char Supla_server[81];
char Email[81];
char Wys[51];
char Supla_name[51];
char Supla_status[51];
byte mac[6];
bool shouldSaveConfig = false;
bool initialConfig = false;
//int customFieldLength = 40;  // ---part of custom Field ( buttons )--
//WiFiManagerParameter custom_field;  // ---part of custom Field ( buttons )--
int npm;





WiFiManager wifiManager;
#include <supla/network/esp_wifi.h>
Supla::ESPWifi wifi("", "");  //------ Do not change----wifimanager takes care------


void saveConfigCallback () {          
  Serial.println("Should save config");
  shouldSaveConfig = true;
}
void ondemandwifiCallback () {
   
   WiFiManagerParameter custom_Supla_server("server", "supla server", Supla_server, 81,"required");
   WiFiManagerParameter custom_Email("email", "Email", Email, 81,"required");
   WiFiManagerParameter custom_Supla_name("name", "Supla Device Name", Supla_name, 51,"required");
   WiFiManagerParameter custom_Supla_status("status", "Supla Last State", Supla_status, 51,"readonly");
   
   wifiManager.setBreakAfterConfig(true);
   wifiManager.setSaveConfigCallback(saveConfigCallback);
 
  
   wifiManager.addParameter(&custom_Supla_server);
   wifiManager.addParameter(&custom_Email);
    wifiManager.addParameter(&custom_Supla_name);
    wifiManager.addParameter(&custom_Supla_status);

   wifiManager.setCustomHeadElement("<style>html{ background-color: #01DF3A;}</style><div class='s'><svg version='1.1' id='l' x='0' y='0' viewBox='0 0 200 200' xml:space='preserve'><path d='M59.3,2.5c18.1,0.6,31.8,8,40.2,23.5c3.1,5.7,4.3,11.9,4.1,18.3c-0.1,3.6-0.7,7.1-1.9,10.6c-0.2,0.7-0.1,1.1,0.6,1.5c12.8,7.7,25.5,15.4,38.3,23c2.9,1.7,5.8,3.4,8.7,5.3c1,0.6,1.6,0.6,2.5-0.1c4.5-3.6,9.8-5.3,15.7-5.4c12.5-0.1,22.9,7.9,25.2,19c1.9,9.2-2.9,19.2-11.8,23.9c-8.4,4.5-16.9,4.5-25.5,0.2c-0.7-0.3-1-0.2-1.5,0.3c-4.8,4.9-9.7,9.8-14.5,14.6c-5.3,5.3-10.6,10.7-15.9,16c-1.8,1.8-3.6,3.7-5.4,5.4c-0.7,0.6-0.6,1,0,1.6c3.6,3.4,5.8,7.5,6.2,12.2c0.7,7.7-2.2,14-8.8,18.5c-12.3,8.6-30.3,3.5-35-10.4c-2.8-8.4,0.6-17.7,8.6-22.8c0.9-0.6,1.1-1,0.8-2c-2-6.2-4.4-12.4-6.6-18.6c-6.3-17.6-12.7-35.1-19-52.7c-0.2-0.7-0.5-1-1.4-0.9c-12.5,0.7-23.6-2.6-33-10.4c-8-6.6-12.9-15-14.2-25c-1.5-11.5,1.7-21.9,9.6-30.7C32.5,8.9,42.2,4.2,53.7,2.7c0.7-0.1,1.5-0.2,2.2-0.2C57,2.4,58.2,2.5,59.3,2.5z M76.5,81c0,0.1,0.1,0.3,0.1,0.6c1.6,6.3,3.2,12.6,4.7,18.9c4.5,17.7,8.9,35.5,13.3,53.2c0.2,0.9,0.6,1.1,1.6,0.9c5.4-1.2,10.7-0.8,15.7,1.6c0.8,0.4,1.2,0.3,1.7-0.4c11.2-12.9,22.5-25.7,33.4-38.7c0.5-0.6,0.4-1,0-1.6c-5.6-7.9-6.1-16.1-1.3-24.5c0.5-0.8,0.3-1.1-0.5-1.6c-9.1-4.7-18.1-9.3-27.2-14c-6.8-3.5-13.5-7-20.3-10.5c-0.7-0.4-1.1-0.3-1.6,0.4c-1.3,1.8-2.7,3.5-4.3,5.1c-4.2,4.2-9.1,7.4-14.7,9.7C76.9,80.3,76.4,80.3,76.5,81z M89,42.6c0.1-2.5-0.4-5.4-1.5-8.1C83,23.1,74.2,16.9,61.7,15.8c-10-0.9-18.6,2.4-25.3,9.7c-8.4,9-9.3,22.4-2.2,32.4c6.8,9.6,19.1,14.2,31.4,11.9C79.2,67.1,89,55.9,89,42.6z M102.1,188.6c0.6,0.1,1.5-0.1,2.4-0.2c9.5-1.4,15.3-10.9,11.6-19.2c-2.6-5.9-9.4-9.6-16.8-8.6c-8.3,1.2-14.1,8.9-12.4,16.6C88.2,183.9,94.4,188.6,102.1,188.6z M167.7,88.5c-1,0-2.1,0.1-3.1,0.3c-9,1.7-14.2,10.6-10.8,18.6c2.9,6.8,11.4,10.3,19,7.8c7.1-2.3,11.1-9.1,9.6-15.9C180.9,93,174.8,88.5,167.7,88.5z'/></svg>");
   wifiManager.setMinimumSignalQuality(8);
   //wifiManager.setShowStaticFields(true); // force show static ip fields
   //wifiManager.setShowDnsFields(true);    // force show dns field always
   wifiManager.setConfigPortalTimeout(180);

   if (!wifiManager.startConfigPortal("Supla Wind & Rain")) { Serial.println("Not connected to WiFi but continuing anyway.");} else { Serial.println("connected...yeey :)");}                
    strcpy(Supla_server, custom_Supla_server.getValue());
    strcpy(Email, custom_Email.getValue());
    strcpy(Supla_name, custom_Supla_name.getValue());
     WiFi.softAPdisconnect(true);   //  close AP
}

void status_func(int status, const char *msg) {    //    ------------------------ Status --------------------------
  if (status != 10){
     strcpy(Supla_status, msg); 
  }              
}
// DHT22 Sensor read implementation


//***************************** WIATR *******************************************
double  get_wind(int channelNumber, double last_val){
  
    double t = -275;  
    t=ms;
    return t;
}

//*****************************OPADY_DESZCZ*******************************************

double get_rain(int channelNumber, double t) {


//double t = -275;  
    t = rainrate*100;
    return t;
    
}    
//######################################## czujniki end ###############################

//#############################################################

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




    if (millis() - lastmillis >= 1000) {
    //Aktualizuj co sekundę
    lastmillis = millis();          // Aktualizacja lastmillis
    
    noInterrupts();                   // W trakcie kalkulacji wyącz obsĹę przerwań
    if (half_revolution_time>0) rpm = (30000 / half_revolution_time) ;      
    interrupts() ; //Przywróć przerwania
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

//##################################################################

void setup() {
  wifi_set_sleep_type(NONE_SLEEP_T);
  Serial.begin(115200);
  
  
  //WIATR 
  pinMode(WindImpuls, INPUT_PULLUP);   
  attachInterrupt(digitalPinToInterrupt(WindImpuls), rpm_fan, FALLING);
 
  //DESZCZ
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, FALLING);
   

  
  if (WiFi.SSID()==""){ initialConfig = true;} 

  if (SPIFFS.begin()) {  // ------------------------- wificonfig read -----------------
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;         
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);   //print config data to serial on startup
        if (json.success()) {Serial.println("\nparsed json");         
          strcpy(Supla_server, json["Supla_server"]);
          strcpy(Email, json["Email"]);
          strcpy(Supla_name, json["Supla_name"]);         
        } else {
          Serial.println("failed to load json config");
          initialConfig = true;
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }

  SuplaDevice.setWindCallback(&get_wind);
  SuplaDevice.setRainCallback(&get_rain);
 
   uint8_t mac[WL_MAC_ADDR_LENGTH];
   WiFi.macAddress(mac);
   char GUID[SUPLA_GUID_SIZE] = {mac[WL_MAC_ADDR_LENGTH - 6], mac[WL_MAC_ADDR_LENGTH - 5], mac[WL_MAC_ADDR_LENGTH - 4], mac[WL_MAC_ADDR_LENGTH - 3],                                
                                 mac[WL_MAC_ADDR_LENGTH - 2], mac[WL_MAC_ADDR_LENGTH - 1], mac[WL_MAC_ADDR_LENGTH - 1], mac[WL_MAC_ADDR_LENGTH - 2], 
                                 mac[WL_MAC_ADDR_LENGTH - 3], mac[WL_MAC_ADDR_LENGTH - 4], mac[WL_MAC_ADDR_LENGTH - 5], mac[WL_MAC_ADDR_LENGTH - 6]};
   char AUTHKEY[SUPLA_AUTHKEY_SIZE] = {mac[WL_MAC_ADDR_LENGTH - 1], mac[WL_MAC_ADDR_LENGTH - 2],mac[WL_MAC_ADDR_LENGTH - 3], mac[WL_MAC_ADDR_LENGTH - 4],
                                       mac[WL_MAC_ADDR_LENGTH - 5], mac[WL_MAC_ADDR_LENGTH - 6],mac[WL_MAC_ADDR_LENGTH - 6], mac[WL_MAC_ADDR_LENGTH - 5],
                                       mac[WL_MAC_ADDR_LENGTH - 4], mac[WL_MAC_ADDR_LENGTH - 3],mac[WL_MAC_ADDR_LENGTH - 2], mac[WL_MAC_ADDR_LENGTH - 1]};
                             
 
    
    SuplaDevice.addRainSensor();
    SuplaDevice.addWindSensor();
    SuplaDevice.setName(Supla_name);
    SuplaDevice.setStatusFuncImpl(&status_func);
  
  SuplaDevice.begin(GUID,              // Global Unique Identifier 
                    Supla_server,  // SUPLA server address
                    Email,   // Email address used to login to Supla Cloud
                    AUTHKEY);          // Authorization key
                      
}

void loop() {
  if (initialConfig == true){ondemandwifiCallback();}

  int C_W_read = digitalRead(wificonfig_pin);{  // ---------------------let's read the status of the Gpio to start wificonfig ---------------------
   if (C_W_read != last_C_W_state) {  time_last_C_W_change = millis();}      
    if ((millis() - time_last_C_W_change) > C_W_delay) {     
     if (C_W_read != C_W_state) {     
       C_W_state = C_W_read;       
       if (C_W_state == LOW) {
        ondemandwifiCallback () ;} } }         
     last_C_W_state = C_W_read;            
   }
      
  if (shouldSaveConfig == true) { // ------------------------ wificonfig save --------------
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["Supla_server"] = Supla_server;
    json["Email"] = Email;
    json["Supla_name"] = Supla_name;
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {Serial.println("failed to open config file for writing"); }   
    json.prettyPrintTo(Serial);
    json.printTo(configFile);
    configFile.close();
    Serial.println("config saved");    
    shouldSaveConfig = false;
    initialConfig = false; 
    WiFi.mode(WIFI_STA);   
    delay(5000);
    ESP.restart(); 
  }
  
  SuplaDevice.iterate();
}

//**********************************WYLICZ OPADY****************************************
void count() {
  float de;
  // Grab the current ms count for common calculations
  unsigned long curtime = millis();
  
  // Make sure we don't record bounces
  if ((curtime - tiptime) < interval) {
    return;
  }
// How long since the last tip?
  unsigned long tipcount = curtime - tiptime;
  tiptime = curtime;
  
  // Calculate mm/hr from period between cup tips
  rainrate = 914400.0 / tipcount;
  
  Serial.print("Cup tip: ");
  Serial.print(tipcount);
  Serial.println("ms");  
  
  Serial.print("Opady deszczu: ");
  Serial.print(rainrate/10);
  Serial.println("l/m2");  
  Serial.println("mm/h");  

}
//**********************************WYLICZ OPADY END ****************************************
