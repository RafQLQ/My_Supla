/*
 Copyright (C) AC SOFTWARE SP. Z O.O.

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

#ifndef _PzenV2_h
#define _PzemV2_h

#include <Arduino.h>
#include <PZEM004T.h>
#include <SoftwareSerial.h>
#include "supla/sensor/one_phase_electricity_meter.h"



namespace Supla {
namespace Sensor {

class PZEMv2 : public OnePhaseElectricityMeter {
    public:
        PZEMv2(int8_t pinRX,int8_t pinTX) : pzem(pinRX, pinTX), ip(192, 168, 1, 1) {
        }

        void onInit() {
            // any initialization required for pzem (it is called within SuplaDevice.begin()
            // it should also do first read of data (so it can call readValuesFromDevice() at the end, but please
            // remember to keep this as last line:
            pzem.setAddress(ip);
            readValuesFromDevice();
            updateChannelValues();
        }

        virtual void readValuesFromDevice() {
            // implement reading data from device
            // use set...() methods from https://github.com/klew/arduino/blob/ma ... ty_meter.h
            // to set values for specific parameters
            // in .h file there is a comment for each set method that tells what are the units. Usually parameter is int and
            // value is like : energy in 0.00001 kWh for setFwdActEnergy(char phase, _supla_int64_t energy) method
            // i.e.:
            // voltage in 0.01 V
            setVoltage(0, pzem.voltage(ip) * 100);
            // current in 0.001 A
            setCurrent(0, pzem.current(ip) * 1000);
            // power in 0.00001 kW
            setPowerActive(0, pzem.power(ip) * 100);
            // energy in 0.00001 kWh
            setFwdActEnergy(0, pzem.energy(ip) * 100);
        }


        PZEM004T pzem;
        IPAddress ip;


};
};  // namespace Sensor
};  // namespace Suplaa


#endif
