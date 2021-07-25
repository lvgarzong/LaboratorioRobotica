
function [y dt]=FTrayectoria()

ancho = 1200/1000*0.4;
d_ang = pi/8;


radio = 0.05*0.38/(d_ang);
%radio = 0.048/(d_ang);
% Tramo 1
pini1 = radio;
pf1 = (ancho - 2*radio) + radio;
% pini1 = Longitud/12;
% pf1 = Longitud-(Longitud/12);
t0 = 0;

a0 = pini1;
a1 = 500/1000;

tf1 = (pf1-pini1)/a1;
dt = tf1/20;
t1 = (0:dt:tf1);

x1 = a0 + a1*(t1-t0);
y1 = zeros(1,length(x1));

vel1 = diff(x1)/(dt);

% plot(x1, y1)
% figure
% plot(t1(1:length(t1)-1), vel1)

% Curva 1 Tramo 2

ang = (-pi/2+d_ang: d_ang: 0);
x2 = radio.*cos(ang) + x1(length(x1));
y2 = radio.*sin(ang) + radio;

% Tramo 3

pini3 = y2(length(y2));
pf3 = (ancho - 2*radio) + radio;
t0 = 0;

a0 = pini3;
a1 = 500/1000;

tf3 = (pf3-pini3)/a1;
t3 = (0:dt:tf3);

y3 = a0 + a1*(t3-t0);
x3 = ones(1,length(y3))*x2(length(x2));

vel3 = diff(y3)/(dt);

% plot(x3, y3)
% figure
% plot(t3(1:length(t3)-1), vel3)

% Curva 2 Tramo 4

ang = (0+d_ang: d_ang: pi/2);
x4 = radio.*cos(ang) + x3(length(x3))- radio;
y4 = radio.*sin(ang) + y3(length(y3));

% Tramo 5

pini5 = x4(length(x4));
pf5 = radio;
t0 = 0;

a0 = pini5;
a1 = 500/1000;

tf5 = (pf5-pini5)/a1;
t5 = (0:dt:abs(tf5));

x5 = a0 - a1*(t5-t0);
y5 = ones(1,length(x5))*y4(length(y4));

vel5 = diff(x5)/(dt);

% plot(x5, y5)
% figure
% plot(t5(1:length(t5)-1), vel5)

% Curva 3 Tramo 6

ang = (pi/2+d_ang: d_ang: pi);
x6 = radio.*cos(ang) + radio;
y6 = radio.*sin(ang) + y5(length(y5)) - radio;

% Tramo 7

pini7 = y6(length(y6));
pf7 = radio;
t0 = 0;

a0 = pini7;
a1 = 500/1000;

tf7 = (pf7-pini7)/a1;
t7 = (0:dt:abs(tf7));

y7 = a0 - a1*(t7-t0);
x7 = zeros(1,length(y7));

vel7 = diff(x7)/(dt);

% plot(x7, y7)
% figure
% plot(t7(1:length(t7)-1), vel7)

% Curva 4 Tramo 8

ang = (pi+d_ang: d_ang: 3*pi/2);
x8 = radio.*cos(ang) + radio;
y8 = radio.*sin(ang) + radio;

% Completo
x_tr = [x1 x2 x3(2:length(x3)) x4 x5(2:length(x5)) x6 x7(2:length(x7)) x8];
y_tr = [y1 y2 y3(2:length(y3)) y4 y5(2:length(y5)) y6 y7(2:length(y7)) y8];
% plot(x_tr, y_tr)
y=[x_tr' y_tr'];

% velx = diff(x_tr)/(dt);
% vely = diff(y_tr)/(dt);
% vtan = sqrt(velx.^2 + vely.^2)
end