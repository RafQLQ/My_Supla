                                              /**
   Supla.org NodeMCU WiFi minimal example
   Author: Programistyk - Kamil Kaminski <kamil@programistyk.pl>

   This example shows how to configure SuplaDevice for building for NodeMCU within Arduino IDE
*/
#define LIGHTSENSORPIN A0 //Ambient light sensor reading
#include <BH1750FVI.h>
#include <srpc.h>
#include <log.h>
#include <eh.h>
#include <proto.h>
#include <IEEE754tools.h>
// We define our own ethernet layer
#define SUPLADEVICE_CPP
#include <SuplaDevice.h>
#include <lck.h>

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
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define OLED_RESET 0
Adafruit_SSD1306 display(OLED_RESET); // GPIO5 - SCL , GPIO4 -SDA


#define SEALEVELPRESSURE_HPA (1023.0)

Adafruit_BME280 bme; // I2C

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);
#define LOGO32_GLCD_HEIGHT 32
#define LOGO32_GLCD_WIDTH  32
static const unsigned char PROGMEM temp_glcd_bmp[] =
{ // temp_home
  0x00, 0x03, 0xc0, 0x00, 0x00, 0x04, 0x20, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x08, 0x50, 0x00, 
  0x00, 0x08, 0x50, 0x00, 0x00, 0x08, 0x50, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x09, 0x50, 0x00, 
  0x00, 0x08, 0xd0, 0x00, 0x00, 0x0b, 0x10, 0x00, 0x00, 0x0b, 0xd0, 0x00, 0x00, 0x0b, 0xd0, 0x00, 
  0x00, 0x0b, 0xd0, 0x00, 0x00, 0x0b, 0xd0, 0x00, 0x00, 0x0b, 0xd0, 0x00, 0x00, 0x0b, 0xd0, 0x00, 
  0x00, 0x0b, 0xd0, 0x00, 0x00, 0x0b, 0xd0, 0x00, 0x00, 0x0b, 0xd0, 0x00, 0x00, 0x0b, 0xd0, 0x00, 
  0x00, 0x0b, 0xd0, 0x00, 0x00, 0x0b, 0xd0, 0x00, 0x00, 0x0b, 0xd0, 0x00, 0x00, 0x17, 0xe8, 0x00, 
  0x00, 0x2f, 0xf4, 0x00, 0x00, 0x2f, 0x94, 0x00, 0x00, 0x2f, 0xb4, 0x00, 0x00, 0x2f, 0xf4, 0x00, 
  0x00, 0x07, 0xe0, 0x00, 0x00, 0x13, 0xc8, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x03, 0xc0, 0x00
};



static const unsigned char PROGMEM pressure_glcd_bmp[] =
{
  // 'pressure'
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 
  0x01, 0xc1, 0xc1, 0xf8, 0x03, 0xc3, 0xc1, 0xf8, 0x03, 0x83, 0x81, 0xf8, 0x03, 0x03, 0x00, 0x18, 
  0x07, 0x07, 0x00, 0x18, 0x07, 0x07, 0x00, 0xf8, 0x07, 0x07, 0x00, 0xf8, 0x07, 0x07, 0x00, 0xf8, 
  0x07, 0x07, 0x00, 0x18, 0x03, 0x03, 0x00, 0x18, 0x03, 0x83, 0x80, 0x18, 0x03, 0x83, 0x81, 0xf8, 
  0x03, 0x83, 0x81, 0xf8, 0x03, 0x83, 0x80, 0x18, 0x01, 0x81, 0x80, 0x18, 0x01, 0x81, 0x80, 0x18, 
  0x03, 0x83, 0x80, 0xf8, 0x03, 0x83, 0x80, 0xf8, 0x03, 0x83, 0x80, 0xf8, 0x13, 0x93, 0x80, 0x18, 
  0x1f, 0x1f, 0x00, 0x18, 0x1f, 0x1f, 0x01, 0xf8, 0x1e, 0x1e, 0x01, 0xf8, 0x1f, 0x1f, 0x01, 0xf8, 
  0x1f, 0x9f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

static const unsigned char PROGMEM humidity_glcd_bmp[] =
{

  0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 
  0x00, 0xfc, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x01, 0xce, 0x00, 0x00, 0x03, 0xcf, 0x00, 0x00, 
  0x03, 0x87, 0x00, 0x00, 0x07, 0x87, 0x80, 0x00, 0x07, 0x03, 0x80, 0x00, 0x0e, 0x01, 0xc0, 0x00, 
  0x1e, 0x01, 0xe0, 0x00, 0x1c, 0x00, 0xe0, 0x00, 0x38, 0x00, 0x70, 0x00, 0x78, 0x00, 0x78, 0x00, 
  0x70, 0x80, 0x38, 0x00, 0x61, 0x48, 0x18, 0x00, 0xe2, 0x48, 0x1c, 0x00, 0xe3, 0x50, 0x1c, 0x00, 
  0xe1, 0xa0, 0x1c, 0x00, 0xe0, 0x2a, 0x1c, 0x00, 0xe0, 0x52, 0x1c, 0x00, 0xe0, 0x92, 0x1c, 0x00, 
  0x60, 0x8c, 0x18, 0x00, 0x70, 0x00, 0x38, 0x00, 0x38, 0x00, 0x70, 0x00, 0x3c, 0x00, 0xf0, 0x00, 
  0x1e, 0x01, 0xe0, 0x00, 0x0f, 0xff, 0xc0, 0x00, 0x07, 0xff, 0x80, 0x00, 0x00, 0xfc, 0x00, 0x00
};

static const unsigned char PROGMEM rain_glcd_bmp[] = 
{
0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00, // ················
  0x00,0xF3,0xE0,0x00, // ····██·██▌······
  0x01,0x8E,0x18,0x00, // ···▐▌·█▌·▐▌·····
  0x03,0x0C,0x08,0x00, // ···█··█···▌·····
  0x04,0x38,0x0F,0x00, // ··▐··█▌···██····
  0x08,0x28,0x11,0x80, // ··▌··▌▌··▐·▐▌···
  0x10,0x40,0x00,0xC0, // ·▐··▐·······█···
  0x10,0x80,0x00,0x40, // ·▐··▌·······▐···
  0x0F,0x00,0x00,0x40, // ··██········▐···
  0x01,0x00,0x00,0x40, // ···▐········▐···
  0x01,0x00,0x00,0x80, // ···▐········▌···
  0x00,0xFF,0xFF,0x00, // ····████████····
  0x00,0x14,0x96,0x00, // ·····▐▐·▌▐▐▌····
  0x00,0x24,0xA4,0x00, // ·····▌▐·▌▌▐·····
  0x00,0x21,0x0C,0x00, // ·····▌·▐··█·····
  0x00,0x40,0x08,0x00, // ····▐·····▌·····
  0x00,0x10,0x80,0x00, // ·····▐··▌·······
  0x00,0x10,0x80,0x00, // ·····▐··▌·······
  0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00, // ················
  0x00,0x00,0x00,0x00  // ················
};

static const unsigned char PROGMEM lux_glcd_bmp[] =
{ 
  0x00,0x01,0xC0,0x00, // ·······▐█·······
  0x00,0x01,0xC0,0x00, // ·······▐█·······
  0x00,0xC1,0xC1,0x80, // ····█··▐█··▐▌···
  0x00,0xE1,0xC3,0x80, // ····█▌·▐█··█▌···
  0x00,0x71,0xC7,0x00, // ····▐█·▐█·▐█····
  0x00,0x70,0x87,0x00, // ····▐█··▌·▐█····
  0x00,0x30,0x06,0x00, // ·····█····▐▌····
  0x18,0x07,0xF0,0x0C, // ·▐▌···▐███····█·
  0x3E,0x1F,0xF8,0x3C, // ·██▌·▐████▌··██·
  0x1F,0x3F,0x7E,0x78, // ·▐██·███▐██▌▐█▌·
  0x07,0x78,0x0E,0x70, // ··▐█▐█▌···█▌▐█··
  0x00,0x70,0x07,0x00, // ····▐█····▐█····
  0x00,0xE0,0x03,0x80, // ····█▌·····█▌···
  0x00,0xE0,0x03,0x80, // ····█▌·····█▌···
  0x7C,0xC0,0x03,0x9F, // ▐██·█······█▌▐██
  0xFE,0xC0,0x01,0xBF, // ███▌█······▐▌███
  0x7C,0xC0,0x01,0x9F, // ▐██·█······▐▌▐██
  0x00,0xE0,0x03,0x80, // ····█▌·····█▌···
  0x00,0xE0,0x03,0x80, // ····█▌·····█▌···
  0x00,0x70,0x07,0x00, // ····▐█····▐█····
  0x07,0x78,0x0F,0x70, // ··▐█▐█▌···██▐█··
  0x0F,0x3E,0x3E,0x78, // ··██·██▌·██▌▐█▌·
  0x1E,0x1F,0xFC,0x3C, // ·▐█▌·▐█████··██·
  0x18,0x07,0xF0,0x0C, // ·▐▌···▐███····█·
  0x00,0x30,0x06,0x00, // ·····█····▐▌····
  0x00,0x70,0x87,0x00, // ····▐█··▌·▐█····
  0x00,0x71,0xC7,0x00, // ····▐█·▐█·▐█····
  0x00,0xE1,0xC3,0x80, // ····█▌·▐█··█▌···
  0x00,0xC1,0xC3,0x80, // ····█··▐█··█▌···
  0x00,0x01,0xC1,0x00, // ·······▐█··▐····
  0x00,0x01,0xC0,0x00, // ·······▐█·······
  0x00,0x00,0x80,0x00  // ········▌·······
};

static const unsigned char PROGMEM logo32_glcd_bmp[] =
{
  // 'logo SUPLA'
  0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x03, 0xf8, 0x00, 0x00, 0x07, 0xfc, 0x00, 0x00,
  0x0e, 0x0e, 0x00, 0x00, 0x0c, 0x06, 0x00, 0x00, 0x1c, 0x03, 0x00, 0x00, 0x1c, 0x03, 0x00, 0x00,
  0x1c, 0x03, 0x00, 0x00, 0x0c, 0x07, 0x00, 0x00, 0x0e, 0x0f, 0x80, 0x00, 0x07, 0xfc, 0xe0, 0x00,
  0x03, 0xf8, 0x30, 0x00, 0x00, 0xf0, 0x0d, 0xe0, 0x00, 0x10, 0x07, 0x30, 0x00, 0x18, 0x02, 0x10,
  0x00, 0x18, 0x06, 0x18, 0x00, 0x08, 0x02, 0x10, 0x00, 0x08, 0x03, 0xf0, 0x00, 0x0c, 0x07, 0xc0,
  0x00, 0x04, 0x0c, 0x00, 0x00, 0x04, 0x08, 0x00, 0x00, 0x04, 0x10, 0x00, 0x00, 0x00, 0x20, 0x00,
  0x00, 0x03, 0xc0, 0x00, 0x00, 0x04, 0x40, 0x00, 0x00, 0x0c, 0x20, 0x00, 0x00, 0x0c, 0x20, 0x00,
  0x00, 0x04, 0x40, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const unsigned char PROGMEM ASL_glcd_bmp[] =
{
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 
  0x00, 0x00, 0x60, 0x00, 0x00, 0x08, 0x21, 0x00, 0x00, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x62, 0x00, 
  0x00, 0x1f, 0xf8, 0x00, 0x00, 0x3f, 0x0c, 0x00, 0x00, 0x61, 0x84, 0x00, 0x00, 0xc0, 0xc4, 0xe0, 
  0x01, 0xc0, 0xf4, 0x00, 0x03, 0x00, 0x1c, 0x00, 0x06, 0x00, 0x0c, 0x00, 0x06, 0x00, 0x0c, 0x00, 
  0x06, 0x10, 0xcd, 0x00, 0x03, 0x10, 0x99, 0x00, 0x01, 0x84, 0x30, 0x00, 0x00, 0x25, 0x00, 0x00, 
  0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 
  };

char* Message[] = {"This is string 1",
                   "Already initialized",
                   "Cb not assigned",
                   "Invalid GUID",
                   "Unknown server address",
                   "Unknow location ID",
                   "Initialized",
                   "Channel limit exceeded",
                   "Rozlaczony",
                   "Rejstracja w toku",
                   "Iterate fail",
                   "Protocol version error",
                   "Bad credentials",
                   "Temporarily unawaliable",
                   "Location conflict",
                   "Channel conflict",
                   "Polaczony i gotowy",
                   "Device is diasbled",
                   "Location is disabled",
                   "Device limit execeeded"
                  };



#if (SSD1306_LCDHEIGHT != 64)
//#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// ***** logowanie WiFi ********************************************************************************
WiFiClient client;

// Setup Supla connection
const char* ssid     = "SSID";
const char* password = "pass";

char str[10];
char StatCommStr[25];
int StatCommInt;
byte Icon;
byte FiveSek;




//// ******* BME280 Temperatura i wilgotnosc *********************************

void get_temperature_and_humidity (int channelNumber, double *temp, double *humidity){
  
    *temp = bme.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

  
    *humidity = bme.readHumidity();
    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());               // wyremowane dla BMP280   
    Serial.println(" %");
    if ( isnan(*temp) || isnan(*humidity) ) {
      *temp = -275;
      *humidity = -1;
    }

  }


//// ******* Ciśnienie *****************************************************  
double get_pressure(int channelNumber, double pressure) {
   
 pressure = bme.readPressure() / 100.0F;
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure()/100);
    Serial.println(" hPa");
   return  pressure;   
  }
////******* Natężenie zrobione obsługą czujnika odległości
double get_distance(int channelNumber, double t) {

 // float lux = analogRead(LIGHTSENSORPIN);             //TEMT6000
   double lux = LightSensor.GetLightIntensity();     //BH1750
    t = abs(lux);
    Serial.print("Natezenie swiatla = ");
    Serial.print(LightSensor.GetLightIntensity());
    delay(1250);
    return t;
  }

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void timer0_ISR (void) {
  FiveSek++;
  if (FiveSek==5) {
    FiveSek=0;
    Icon++;
    if (Icon==5) Icon=1;
  }
  timer0_write(ESP.getCycleCount() + 80000000L); // 80MHz == 1sec 
}

//// ******* Funkcja główna *********************************************************************************
void setup() {
  
    Serial.begin(9600); //BH1750
  //pinMode(LIGHTSENSORPIN, INPUT);   // dodany czujnik natężenia 
    LightSensor.begin();  
   
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(timer0_ISR);
  timer0_write(ESP.getCycleCount() + 80000000L); // 80MHz == 1sec
  interrupts();
 
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
    }

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
 
  Icon = 0;

  // Clear the buffer.
  display.clearDisplay();

  // text display tests
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setRotation(2);
  display.setCursor(30, 25);
  display.print("SUPLA");
  drawbitmap(logo32_glcd_bmp, LOGO32_GLCD_HEIGHT, LOGO32_GLCD_WIDTH);
  display.display();





  // Replace the falowing GUID
  char GUID[SUPLA_GUID_SIZE] = {0x27,0x8B,0xB5,0x9A,0x6C,0x9E,0xFE,0x5A,0x3C,0x09,0xE0,0x5D,0x01,0x04,0x53,0xEB};
 
  //  GUID that you can retrieve from https://www.supla.org/arduino/get-guid

  // Ethernet MAC address
  uint8_t mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  /*
     Having your device already registered at cloud.supla.org,
     you want to change CHANNEL sequence or remove any of them,
     then you must also remove the device itself from cloud.supla.org.
     Otherwise you will get "Channel conflict!" error.
  */
    
          // CHANNEL0 
            SuplaDevice.addDHT22(); // ten objekt jest tylko po to aby w aplikacji wyĹ›wietlic temp i wilgotnosc
	          SuplaDevice.addDistanceSensor(); // ten obiekt wyswietla natezenie niestety bez jednostek (LUX)
	          SuplaDevice.addPressureSensor();  // cisnienie
 
     

  

  SuplaDevice.setName("R@F_POG");
  SuplaDevice.setStatusFuncImpl(&status_func);

  SuplaDevice.begin(GUID,              // Global Unique Identifier
                    mac,               // Ethernet MAC address
                    "svrx.supla.org",  // SUPLA server address
                    1111,                 // Location ID
                    "xxx");               // Location Password

}


// ***********************************************************************************************************
void loop() {
  SuplaDevice.iterate();
  DisplayTemp();

}

// ****   Obsługa OLED    ****************************************************************************************************

void DisplayTemp() {

   
//double Light_Intensity = LightSensor.GetLightIntensity();

 
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setRotation(2);
  display.setCursor(45, 25);  
 
  switch (Icon) {
      case 1:
        display.setTextSize(3);
        display.setCursor(45, 25);  
        display.print(bme.readTemperature(), 0);
        display.print((char)247);
        display.println("C");
        drawbitmap(temp_glcd_bmp, LOGO32_GLCD_HEIGHT, LOGO32_GLCD_WIDTH);
        
        break;
      case 2:
       display.setTextSize(3);
        display.setCursor(45, 25);  
       display.print(bme.readHumidity(), 0);
        display.println("%");
        drawbitmap(humidity_glcd_bmp, LOGO32_GLCD_HEIGHT, LOGO32_GLCD_WIDTH);
        break;
        
      case 3:
         display.setTextSize(2);
         display.setCursor(32, 25);  
        display.print(bme.readPressure() / 100.0F, 0);
        display.println(" hPa");
        drawbitmap(pressure_glcd_bmp, LOGO32_GLCD_HEIGHT, LOGO32_GLCD_WIDTH);
        break;
        
  /*     case 4:
       display.setTextSize(2);
        display.setCursor(45, 25);  
        display.print(Light_Intensity, 0);
        display.println(" Lux");
        drawbitmap(lux_glcd_bmp, LOGO32_GLCD_HEIGHT, LOGO32_GLCD_WIDTH);           
        break;
      
    */            
      case 4:
        display.setTextSize(2);
        display.setCursor(45, 25);  
        display.print(bme.readAltitude(SEALEVELPRESSURE_HPA), 0);
        display.println(" npm");
        drawbitmap(ASL_glcd_bmp, LOGO32_GLCD_HEIGHT, LOGO32_GLCD_WIDTH);
        break;
    
 };
//---------------------        
 
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 55);
  display.print(Message[StatCommInt - 1]);
  display.display();
  display.display();

}; //DisplayTemp

void drawbitmap(const uint8_t *bitmap, uint8_t w, uint8_t h) {
  uint8_t icons[NUMFLAKES][3];

  display.drawBitmap(0, 0, bitmap, w, h, WHITE);
} //drawbitmap


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
        Serial.print(".");
  }

  Serial.print("\nlocalIP: ");
  Serial.println(WiFi.localIP());
  Serial.print("subnetMask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("gatewayIP: ");
  Serial.println(WiFi.gatewayIP());
}

SuplaDeviceCallbacks supla_arduino_get_callbacks(void) {
  SuplaDeviceCallbacks cb;

  cb.tcp_read = &supla_arduino_tcp_read;
  cb.tcp_write = &supla_arduino_tcp_write;
  cb.eth_setup = &supla_arduino_eth_setup;
  cb.svr_connected = &supla_arduino_svr_connected;
  cb.svr_connect = &supla_arduino_svr_connect;
  cb.svr_disconnect = &supla_arduino_svr_disconnect;
  cb.get_temperature_and_humidity = get_temperature_and_humidity;
  cb.get_pressure = get_pressure; 
  cb.get_temperature = NULL;
  cb.get_rgbw_value = NULL;
  cb.set_rgbw_value = NULL;
  cb.get_distance= get_distance;

  return cb;
}



void status_func(int status, const char *msg) {
  Serial.print("Status : ");
  Serial.print(status);
  StatCommInt = status;
  Serial.print(" - ");
  Serial.println(Message[StatCommInt - 1]);
  display.fillRect(0, 55, 128, 65, BLACK);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(2);
  display.setCursor(0, 55);
  display.print(Message[StatCommInt - 1]);
  display.display();

}

