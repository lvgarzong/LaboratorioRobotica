function y = jacobiano(L, Robot)
syms q1 q2 q3 q4 q5 q6     % Configuraciones del robot simbólicas
threshold = 1e-10;      % Umbral

q = [q1 q2 q3 q4 q5 q6];   % Vector de configuraciones
Rot_total = [];         % Vector para almacenar todas las rotaciones
Trasl_total = [];       % Vector para almacenar todas las posiciones
MTH_aux = 1;            % Valor inicial variable auxiliar MTH
for i = 1:7
    if i<7
        % Calculo de la matriz T_i_i+1
        MTH = mapSymType(L(i).A(q(i)),'rational', @(x) piecewise(abs(x)<=threshold, 0, x));
        MTH_aux = MTH_aux*MTH;          % Multiplicación de MTH (0Tn)     
    elseif i==7
        MTH_aux = MTH_aux*Robot.tool;   % Multiplicación con la MTH del efector final
    end
    [Rot Trasl] = tr2rt(MTH_aux);       % Extracción de rotaciones y traslaciones
    Rot_total = [Rot_total Rot];         % Almacenamiento de las rotaciones
    Trasl_total = [Trasl_total Trasl];   % Almacenamiento de las traslaciones
end

% art = ['R' 'R' 'R' 'R' 'R' 'R'];    % Tipo de articulación
J0 = [];                        % Matriz jacobiana
vel = [];                       % Matriz velocidad

for n = 1:6
    j = (n-1)*3+1;              % Variable para avanzar de 3 en 3      
    % R0i*[0 0 1]'
    vel_ang = Rot_total(:, j:j+2)*[0; 0; 1];
    % R0i*[0 0 1]'x(P0n - P0i)
    vel_lin = cross(vel_ang, Trasl_total(:,7)-Trasl_total(:,n));       

    vel = [vel_lin; vel_ang];   % Almacenamiento de velocidades
    J0 = [J0 vel];              % Jacobiano
end
y=J0;
end