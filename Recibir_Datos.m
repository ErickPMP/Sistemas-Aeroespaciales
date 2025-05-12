tf=40;
ts=0.1;
t=0:ts:tf;

N=length(t);

COM='COM3';
delete(instrfind({'Port'},{COM}));
PS=serial(COM,"BaudRate",115200);

X=zeros(1,N);
Y=zeros(1,N);
Z=zeros(1,N);

tiempo=zeros(1,N);

fopen(PS);

pause(3);

disp('Recibiendo Datos...');

for k=1:N
    tic

    X(k)=fscanf(PS,'%f\n');
    Y(k)=fscanf(PS,'%f\n');
    Z(k)=fscanf(PS,'%f\n');

    tiempo(k)=toc;
    while toc<ts
    end
end
disp('Datos Recolectados...');

fclose(PS);
delete(PS);

figure
scatter3(X(1,:),Y(1,:),Z(1,:));
Matrix_magne = [X' Y' Z'];

[A,b,expMFS]  = magcal(Matrix_magne);
xCorrected = (Matrix_magne-b)*A;

de = HelperDrawEllipsoid;
de.plotCalibrated(A,b,expMFS,Matrix_magne,xCorrected,'Auto');


r = sum(xCorrected.^2,2) - expMFS.^2;
E = sqrt(r.'*r./N)./(2*expMFS.^2);
fprintf('Residual error in corrected data : %.2f\n\n',E);
