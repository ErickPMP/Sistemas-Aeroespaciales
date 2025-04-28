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

unsigned long tiempo_prev0,tiempo_prev1,tiempo_prev2;
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

valor e_ML, u_ML, Out_ML, In_ML, ang_y_ML,u_controlD_ML,u_controlI_ML;
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
  ESC1.write(0.0);
  ESC2.write(0.0);
  delay(6000);
}

void loop() {

  if(t<10.0){In=0.0;}
  if(t>=10.0 && t<20.0){In=10.0;}
  if(t>=20.0 && t<30.0){In=0.0;}
  if(t>=30.0 && t<40.0){In=20.0;}
  if(t>=40.0 && t<50.0){In=0.0;}
  if(t>=50.0 && t<60.0){In=30.0;}
  if(t>=60.0 && t<70.0){In=0.0;}
  if(t>=70.0 && t<80.0){In=40.0;}
  if(t>=80.0 && t<90.0){In=0.0;}
  if(t>=90.0 && t<100.0){In=50.0;}
  if(t>=100.0 && t<110.0){In=0.0;}
  if(t>=110.0 && t<120.0){In=60.0;}
  if(t>=120.0 && t<130.0){In=0.0;}
  if(t>=130.0 && t<140.0){In=70.0;}
  if(t>=140.0 && t<150.0){In=0.0;}
  if(t>=150.0 && t<160.0){In=80.0;}
  if(t>=160.0 && t<170.0){In=0.0;}
  if(t>=170.0 && t<180.0){In=90.0;}
  if(t>=180.0 && t<190.0){In=0.0;}
  if(t>=190.0 && t<200.0){In=-10.0;}
  if(t>=200.0 && t<210.0){In=0.0;}
  if(t>=210.0 && t<220.0){In=-20.0;}
  if(t>=220.0 && t<230.0){In=0.0;}
  if(t>=230.0 && t<240.0){In=-30.0;}
  if(t>=240.0 && t<250.0){In=0.0;}
  if(t>=250.0 && t<260.0){In=-40.0;}
  if(t>=260.0 && t<270.0){In=0.0;}
  if(t>=270.0 && t<280.0){In=-50.0;}
  if(t>=280.0 && t<290.0){In=0.0;}
  if(t>=290.0 && t<300.0){In=-60.0;}
  if(t>=300.0 && t<310.0){In=0.0;}
  if(t>=310.0 && t<320.0){In=-70.0;}
  if(t>=320.0 && t<330.0){In=0.0;}
  if(t>=330.0 && t<340.0){In=-80.0;}
  if(t>=340.0 && t<350.0){In=0.0;}
  if(t>=350.0 && t<360.0){In=-90.0;}
  if(t>=360){t = 0.0;}

  u_controlD = 90.0+In;
  u_controlI = 90.0-In;
  ESC1.write(u_controlD);
  ESC2.write(u_controlI);

  if(micros()-tiempo_prev0>10000){
    tiempo_prev0=micros();
    //Calcular angulo de rotación con giroscopio y filtro complemento 
    //ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
    //ang_y_prev=ang_y;
    t = t + 1.0/100.0;
  }

  if(millis()-tiempo_prev1>50){
    tiempo_prev1=millis();
    u_controlD_ML.number = u_controlD;
    u_controlI_ML.number = u_controlI;
    In_ML.number = In;
    Out_ML.number = ang_y;
    Serial.write('V');
    for (int i = 0; i < 4; i++) Serial.write(u_controlD_ML.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(u_controlI_ML.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(In_ML.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(Out_ML.bytes[i]);
    Serial.write('\n');
  }

  if(millis()-tiempo_prev2>100){
    tiempo_prev2=millis();
    // Leer las aceleraciones y velocidades angulares
    //sensor.getAcceleration(&ax, &ay, &az);
    //sensor.getRotation(&gx, &gy, &gz);
    //Calcular los ángulos con acelerometro
    //accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
    /*
    Serial.print("e uD uI Out In ang_x:\t");
    Serial.print(t); Serial.print("\t");
    Serial.print(u_controlD); Serial.print("\t");
    Serial.print(u_controlI); Serial.print("\t");
    Serial.print(Out); Serial.print("\t");
    Serial.print(In); Serial.print("\t");
    Serial.println(ang_y);
    */
  }
}
