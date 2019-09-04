#include <PZEM004Tv30.h>        // biblioteka OZEM-004T v.3 https://github.com/mandulaj/PZEM-004T-v30.

PZEM004Tv30 pzem(4, 5);


void setup() {
  Serial.begin(115200);

  // Funkcja zerowania licznika energii. Zerowanie następuje przy każdym uruchomieniu programu. Aby wyłączyć zerowanie dodaj znacznik komentarza przed poniższymi 6 liniami.
   // Serial.println();
    //Serial.println();
    //Serial.println("ZEROWANIE LICZNIKA ENERGII");
    //Serial.println();
    //Serial.println();
 //   pzem.resetEnergy();
}

void loop() {

  // Wyświetlenie wartości mierzonych w monitorze portu szeregowego.
  
    float voltage = pzem.voltage();
    Serial.print(voltage, 1); Serial.print(" V | ");

    float frequency = pzem.frequency();
    Serial.print(frequency, 1); Serial.print(" Hz | ");

    float current = pzem.current();
    Serial.print(current, 3); Serial.print(" A | ");

    float power = pzem.power();
    Serial.print(power, 1); Serial.print(" W | ");

    float power_factor = pzem.pf();
    Serial.print(power_factor, 2); Serial.print(" cos ø | ");

    float energy = pzem.energy();
    Serial.print(energy, 3); Serial.println(" kWh");

  // Odczyt wartości mierzonych z interwałem 5 sekund.
    delay(5000);
}
