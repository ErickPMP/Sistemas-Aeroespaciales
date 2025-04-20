//Pines MPU es derecha
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Servo.h>

MPU6050 sensor;
Servo ESC1; 
Servo ESC2;
// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

long tiempo_prev0,tiempo_prev1,tiempo_prev2;
float dt=0.001;
float ang_y;
float ang_y_prev;
float accel_ang_y;
bool Signo = 0;
float t=0.0,w=0.0,p=0.0;
float u=0.0,u_controlD=0.0,u_controlI=0.0,e=0.0,In=0.0,Out=0.0;
float Kp=1.0,Ki=0.0,Kd=0.0;
typedef union {
  float number;
  uint8_t bytes[4];
} valor;

valor e_ML, u_ML, Out_ML, In_ML, ang_y_ML;
void setup() {
  Serial.begin(9600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
  sensor.setXAccelOffset(-1359);
  sensor.setYAccelOffset(548);
  sensor.setZAccelOffset(765);
  sensor.setXGyroOffset(63);
  sensor.setYGyroOffset(47);
  sensor.setZGyroOffset(11);
  ESC1.attach(9,1000,2000);
  ESC2.attach(10,1000,2000);
  delay(1000);
}

void loop() {

  u_controlD = 90.0+u;
  u_controlI = 90.0-u;
  ESC1.write(u_controlD);
  ESC2.write(u_controlI);

  if(millis()-tiempo_prev0>1){
    tiempo_prev0=millis();
    //Calcular angulo de rotación con giroscopio y filtro complemento 
    ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
    ang_y_prev=ang_y;
    t = t + 1.0/1000.0;
    In = 0.0;
    Out = ang_y;
    e = In - Out;
    u = Kp * e;
    if(u>0){Signo = 1;}
    else {Signo = 0;}

    if(u >= 180){u = 180;}
    if(u <= -180){u = -180;}
    else{u = u;}
  }
/*
  if(millis()-tiempo_prev1>10){
    tiempo_prev1=millis();
    u_ML.number = u;
    e_ML.number = e;
    Out_ML.number = Out;
    In_ML.number = In;
    ang_y_ML.number = ang_y;
    Serial.write('V');
    for (int i = 0; i < 4; i++) Serial.write(u_ML.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(e_ML.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(Out_ML.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(In_ML.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(ang_y_ML.bytes[i]);
    Serial.write('\n');
  }
*/
  if(millis()-tiempo_prev2>100){
    tiempo_prev2=millis();
    // Leer las aceleraciones y velocidades angulares
    sensor.getAcceleration(&ax, &ay, &az);
    sensor.getRotation(&gx, &gy, &gz);
    //Calcular los ángulos con acelerometro
    accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
    
    Serial.print("e uD uI Out In ang_x:\t");
    Serial.print(e); Serial.print("\t");
    Serial.print(u_controlD); Serial.print("\t");
    Serial.print(u_controlI); Serial.print("\t");
    Serial.print(Out); Serial.print("\t");
    Serial.print(In); Serial.print("\t");
    Serial.println(ang_y);
    
  }
}
