clc
clear all

R = 2;   %m Cilindro
H = 6;   %m Altura
r = 3;   %m Largo del brazo
M = 500; %Kg Masa del cilindro
m1 = 8;  %Kg Masas puntual1
m3 = 8;  %Kg
m2 = 5;  %Kg
m4 = 5;  %Kg

m = [m1 m2 m3 m4]; %m1 m2 m3 m4


Ixx = (1/4)*M*R^2 +(1/12)*M*H^2 ; %Kg*m^2
Iyy = Ixx;
Izz = 0.5*M*R^2;


Ic = [Ixx   0       0;
      0     Iyy     0;
      0     0       Izz];
  
r_M = [ 5  0  0;  %r1
        0  5  0;  %r2
       -5  0  0;  %r3
        0 -5  0]; %r4

%Matriz Antisimétrica (Skew-symmetric matrix)
for i = 1:4
    vec_r = r_M(i,:);  
    s_r = [    0     -vec_r(3)  vec_r(2);
            vec_r(3)     0     -vec_r(1);
           -vec_r(2)  vec_r(1)     0    ];
    I{i} = m(i)*s_r*s_r'; 
end

I_T = Ic + I{1} + I{2} + I{3} + I{4}
