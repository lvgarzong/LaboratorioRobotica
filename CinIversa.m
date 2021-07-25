function [q1, q2, q3, q4, q5, q6] = CinIversa(xe, ye, ze, roll, pitch, yaw, L1, L2, L3, L4, L5, L, Robot, config)
    
    R0T = rpy2r(roll, pitch, yaw, 'deg');   % Matriz rotación efector final

    pe = [xe; ye; ze];      % Coordenadas efector final
    pw = pe - L5*R0T(:,3);  % Coordenadas muñeca
    xw = pw(1);             % Coordenada x muñeca
    yw = pw(2);             % Coordenada y muñeca
    zw = pw(3);             % Coordenada z muñeca
    
    r = sqrt(xw^2 + yw^2);
    Lz = zw - L1;
    
    if config == "codoArriba"        
        Lxy = r - L2;        
        Lp = sqrt(Lxy^2 + Lz^2);

        q1 = atan2(yw,xw);       % Valor q1

        alpha = atan2(Lz, Lxy);
        cosB = (L3^2 + Lp^2 - L4^2)/(2*L3*Lp);
        senB = sqrt(1 - cosB^2);
        beta = atan2(senB, cosB);
        q2 = -((pi/2) - (alpha + beta)); % Valor q2 

        cosY = (L3^2 + L4^2 - Lp^2)/(2*L3*L4);
        senY = sqrt(1 - cosY^2);
        gama = atan2(senY, cosY);
        phi = pi - gama;
        q3 = (pi/2) - phi;       % Valor q3 

        MTH03 = L(1).A(q1)*L(2).A(q2)*L(3).A(q3); % MTH de 3 con respecto a la base
        [R03, P03] = tr2rt(MTH03);              % Separar rotación y translación
        [RT6, PT6] = tr2rt(Robot.tool^(-1));    % Matriz de rotación de 6 con respecto al TCP

        R06 = R0T*RT6;                          % Matriz de rotación de 6 con respecto a la base
        R36 = (R03^(-1))*R06;                   % Matriz de rotación de 6 respecto a 3
        
        % Valor q4 
        q4 = atan2(R36(3,3),R36(1,3));
        % Valor q5 
        c5 = -R36(2,3);
        s5 = sqrt(1 - c5^2);
        q5 = atan2(s5,c5);
        % Valor q6 
        q6 = atan2(-R36(2,2),R36(2,1));
        % Vector de posiciones artiulares en grados
        [q1 q2 q3 q4 q5 q6]
        [rad2deg(q1) rad2deg(q2) rad2deg(q3) rad2deg(q4) rad2deg(q5) rad2deg(q6)]
    end
end