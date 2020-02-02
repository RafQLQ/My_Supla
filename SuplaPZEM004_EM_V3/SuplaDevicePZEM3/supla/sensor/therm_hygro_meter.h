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

#ifndef _therm_hygro_meter_h
#define _therm_hygro_meter_h

#include "thermometer.h"

#define HUMIDITY_NOT_AVAILABLE -1

namespace Supla {
namespace Sensor {
class ThermHygroMeter: public Thermometer {
  public:
    ThermHygroMeter() {
      channel.setType(SUPLA_CHANNELTYPE_HUMIDITYANDTEMPSENSOR);
      channel.setDefault(SUPLA_CHANNELFNC_HUMIDITYANDTEMPERATURE);
    }

    double getTemp() {
      return TEMPERATURE_NOT_AVAILABLE;
    }

    double getHumi() {
      return HUMIDITY_NOT_AVAILABLE;
    }

    void iterateAlways() {
      if (lastReadTime + 10000 < millis()) {
        lastReadTime = millis();
        channel.setNewValue(getTemp(), getHumi());
      }
    }

    protected:
};

};  // namespace Sensor
};  // namespace Supla

#endif
