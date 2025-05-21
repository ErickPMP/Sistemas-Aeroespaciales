clc, clear all, close all
D=1; %m
Ms=173; %kg
L=2; %m
ma1=2;% m
ma2=2;% m
ma3=2;% m
ma4=2;% m

ma = [ma1 ma2 ma3 ma4];
Ixx=2/5*Ms*(D/2)^2;
Iyy=2/5*Ms*(D/2)^2;
Izz=2/5*Ms*(D/2)^2;

Ic=[Ixx 0   0;
    0  Iyy  0;
    0   0  Izz];

r_M = [ 0.5  0  0;  %r1
        0  0.5  0;  %r2
       -0.5  0  0;  %r3
        0 -0.5  0]; %r4

Ia=[0 0 0;
    0 1/3*ma1*L^2 0;
    0 0 1/3*ma1*L^2];

Ib=[1/3*ma1*L^2 0 0;
    0 0 0;
    0 0 1/3*ma1*L^2];


%Matriz Antisimétrica (Skew-symmetric matrix)
for i = 1:4
    vec_r = r_M(i,:);  
    s_r = [    0     -vec_r(3)  vec_r(2);
            vec_r(3)     0     -vec_r(1);
           -vec_r(2)  vec_r(1)     0    ];
    if i==1 || i ==3
        I{i} = Ia+ma(i)*s_r*s_r'; 
    elseif i== 2 || i==4
        I{i} = Ib+ma(i)*s_r*s_r';
    end
end

I_T = Ic + I{1} + I{2} + I{3} + I{4}
