#include <QMC5883LCompass.h>
QMC5883LCompass compass;
//const float dataLength = 3;
//float datos[dataLength];

void setup() {
  Serial.begin(115200); // Changed from 9600 to 1000000!
  Wire.begin(21,22);
  compass.init();

}

void loop() {
  int16_t x, y, z;
  delay(100);
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  Serial.println(x);
  //Serial.print(",");
  Serial.println(y);
  //Serial.print(",");
  Serial.println(z);
  delay(100);
}
