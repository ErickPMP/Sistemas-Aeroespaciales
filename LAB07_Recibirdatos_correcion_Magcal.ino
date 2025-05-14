    #include <QMC5883LCompass.h>
    QMC5883LCompass compass;
    
    void setup() {
      Serial.begin(9600);
      Wire.begin();
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

//***************************************



#include <QMC5883LCompass.h>
#include <Wire.h>
// Magnetometro calibracion Matlab*****
float b[3] = {-127.912810589833,-514.211709225008,421.741715997397};
float A[3][3] = {{1.04988476567549,-0.0573519069595377,0.0281982999586134},
                 {-0.0573519069595377,0.986643078277130,0.0699773707780927},
                 {0.0281982999586134,0.0699773707780927,0.974413099953857}};

float m[3];
float magnevalues[3];
float x_minus_b[3];
float mx,my,mz;
QMC5883LCompass compass;
double anguloConvertido;
int x,y,z;
float angulo;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
}
void loop() {
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  magnevalues[0]=x;
  magnevalues[1]=y;
  magnevalues[2]=z;
    for (int i = 0; i < 3; i++) {
        x_minus_b[i] = magnevalues[i] - b[i];  // x - b
  }

   // Multiplicacin de (x - b) por A
    for (int i = 0; i < 3; i++) {
        m[i] = 0;  // Inicializa el vector resultado
        for (int j = 0; j < 3; j++) {
            m[i] += x_minus_b[j] * A[j][i];  // Multiplicacin de matrices
        }
  }

  mx=m[1];
  my=m[2];
  mz=m[3];
  //angulo = atan2(y, z);
  angulo = atan2(mx, my);
  anguloConvertido = angulo*(180/M_PI);
  //if(anguloConvertido<0) anguloConvertido=anguloConvertido+360;
  delay(250);
}
