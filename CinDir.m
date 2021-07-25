function [MTH] = CinDir(i)
%Esta función permite determinar la matriz de transformación homogenea de
%los eslabones a partir de los parametros DHmod.
%Los parametros de este robot ya se encuentran cargados por lo que solo es
%necesario indicar el numero de la i para crear la MTH de i respecto a i-1.

    syms q1 q2 q3 q4 q5 q6;
    L1 = 486.5/100; L2 = 150/100; L3 = 700/100; L4=600/100; L5 = 65/100;     % Dimensiones
    Alpha_i1 = [0 pi/2 0 pi/2 -pi/2 pi/2];
    A_i1 = [0 L2 L3 0 0 0];
    D_i = [L1 0 0 L4 0 0];
    Theta_i = [q1 q2+pi/2 q3 q4 q5 q6];
    
    MTH(1,1) = cos(Theta_i(i));
    MTH(1,2) = -sin(Theta_i(i));
    MTH(1,3) = 0;
    MTH(1,4) = A_i1(i);
    MTH(2,1) = sin(Theta_i(i))*cos(Alpha_i1(i));
    MTH(2,2) = cos(Theta_i(i))*cos(Alpha_i1(i));
    MTH(2,3) = -sin(Alpha_i1(i));
    MTH(2,4) = -sin(Alpha_i1(i))*D_i(i);
    MTH(3,1) = sin(Theta_i(i))*sin(Alpha_i1(i));
    MTH(3,2) = cos(Theta_i(i))*sin(Alpha_i1(i));
    MTH(3,3) = cos(Alpha_i1(i));
    MTH(3,4) = cos(Alpha_i1(i))*D_i(i);
    MTH(4,1) = 0;
    MTH(4,2) = 0;
    MTH(4,3) = 0;
    MTH(4,4) = 1;
end

