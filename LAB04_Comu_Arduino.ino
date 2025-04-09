float a0=1.0;
float a1=-1.99800199866733;
float a2=0.998001998667333;

float b0=0.0;
float b1=2.83901609810619e-05;
float b2=2.83712405151578e-05;
float In,Out,rk,rk1,rk2;
float ms,s,t,w;
float frec = 1000;
float e,u;
float Kp=1.0;

unsigned long timeold;
typedef union {
  float number;
  uint8_t bytes[4];
} valor;

valor e_ML;
valor u_ML;
valor Out_ML;
valor In_ML;

void setup() {
Serial.begin(9600);
 noInterrupts();
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;
  TCCR0A |= (1 << WGM01); // CTC mode
  TCCR0B = _BV(CS02)| _BV(CS00); // Clock/1024
  OCR0A = 16000000/1024/frec/2;
  TIMSK0 |= (1 << OCIE0A);
  interrupts();
  
  timeold = 0;
}

ISR(TIMER0_COMPA_vect){
  ms = ms+1;
  w=w+1;
  if(ms == 1000.00){
  ms = 0.0;
  s = s+1;
  }
  t = s + ms/1000.00;

  if(t<1)In=0.0;
  if(t>=1)In=1.0;

  if(w == 10){
    w = 0.0;
    u_ML.number = u;
    e_ML.number = e;
    Out_ML.number = Out;
    In_ML.number = In;
    Serial.write('V');
      for (int i = 0; i < 4; i++) { Serial.write(u_ML.bytes[i]); }
      for (int i = 0; i < 4; i++) { Serial.write(e_ML.bytes[i]); }
      for (int i = 0; i < 4; i++) { Serial.write(Out_ML.bytes[i]); }
      for (int i = 0; i < 4; i++) { Serial.write(In_ML.bytes[i]); }
    Serial.write('\n');
  }
  Out = rk*b0+rk1*b1+rk2*b2;
  rk = u-rk1*a1-rk2*a2;
  rk2 = rk1;
  rk1 = rk;

  e=In-Out;
  u=e*Kp;
}

void loop() {
  
}
