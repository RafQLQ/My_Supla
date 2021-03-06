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

#ifndef _element_h
#define _element_h

#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
#include <cstddef>
#endif
#include "channel.h"

namespace Supla {
/*
class Element;

class ElementIterator {
  public:
    ElementIterator& operator++() {
      if (currentPtr) {
        currentPtr = currentPtr->nextPtr;
      }
      return *this;
    }

    ElementIterator(Element *ptr) {
      currentPtr = ptr;
    }

  protected:
    Element *currentPtr;
}
*/
class Element {
  public:

    Element() {
      if (firstPtr == nullptr) {
        firstPtr = this;
      } else {
        last()->nextPtr = this;
      }
      nextPtr = nullptr;
    }

    static Element *begin() {
      return firstPtr;
    }

    static Element *last() {
      Element *ptr = firstPtr;
      while (ptr && ptr->nextPtr) {
        ptr = ptr->nextPtr;
      }
      return ptr;
    }

    Element *next() {
      return nextPtr;
    }

    // method called during SuplaDevice initialization. I.e. load initial state, initialize pins etc.
    virtual void onInit() {};

    // TODO:
    // method called during Config initialization (i.e. read from EEPROM, FRAM). Called only if Config module is
    // configured
    virtual void onLoadConfig() { };

    // method called on each SuplaDevice iteration (before Network layer iteration). When Device is connected,
    // both iterateAlways() and iterateConnected() are called.
    virtual void iterateAlways() {};

    // method called on each Supla::Device iteration when Device is connected and registered to Supla server
    virtual bool iterateConnected(void *srpc) {
      Channel *channel = getChannel();
      if (channel && channel->isUpdateReady() && channel->nextCommunicationTimeMs < millis()) {
        channel->nextCommunicationTimeMs = millis() + 100;
        channel->sendUpdate(srpc);
        return false;
      }
      return true;
    }

    // method called on timer interupt
    // Include all actions that have to be executed periodically regardless of other SuplaDevice activities
    virtual void onTimer() { };

    // method called on fast timer interupt
    // Include all actions that have to be executed periodically regardless of other SuplaDevice activities
    virtual void onFastTimer() { };

  protected:
    virtual Channel *getChannel() {
      return nullptr;
    }
    static Element *firstPtr;;
    Element *nextPtr;
};

};  // namespace Supla

#endif
