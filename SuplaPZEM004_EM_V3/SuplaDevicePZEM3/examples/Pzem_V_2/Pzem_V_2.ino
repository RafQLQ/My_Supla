/*
   Copyright (C) AC SOFTWARE SP. Z O.O.

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version 2
   of the License, or (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
   */

#include <SPI.h>
#include <SuplaDevice.h>
#include <io.h>
#include <supla/sensor/PzemV2.h>

// ESP8266 based board:
#include <supla/network/esp_wifi.h>
Supla::ESPWifi wifi("your_wifi_ssid", "your_wifi_password");


void setup() {

    Serial.begin(9600);

    // Replace the falowing GUID with value that you can retrieve from https://www.supla.org/arduino/get-guid
    char GUID[SUPLA_GUID_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    // Replace the following AUTHKEY with value that you can retrieve from: https://www.supla.org/arduino/get-authkey
    char AUTHKEY[SUPLA_AUTHKEY_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    /*
     * Having your device already registered at cloud.supla.org,
     * you want to change CHANNEL sequence or remove any of them,
     * then you must also remove the device itself from cloud.supla.org.
     * Otherwise you will get "Channel conflict!" error.
     */

    new Supla::Sensor::PZEMv2(5, 4); // (RX,TX) connect to TX,RX of PZEM


    /*
     * SuplaDevice Initialization.
     * Server address, LocationID and LocationPassword are available at https://cloud.supla.org
     * If you do not have an account, you can create it at https://cloud.supla.org/account/create
     * SUPLA and SUPLA CLOUD are free of charge
     *
     */

    SuplaDevice.begin(GUID, // Global Unique Identifier
            "svr1.supla.org", // SUPLA server address
            "email@address", // Email address used to login to Supla Cloud
            AUTHKEY); // Authorization key

}

void loop() {
    SuplaDevice.iterate();
}
