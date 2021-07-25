%% CINEMATICA DIRECTA (Metodo manual)
clf
clear
% Calculo de las MTH de cada eslabon
L5 = 65/100;
MTH_01 = CinDir(1);
MTH_12 = CinDir(2);
MTH_23 = CinDir(3);
MTH_34 = CinDir(4);
MTH_45 = CinDir(5);
MTH_56 = CinDir(6);
tcp = [  -1  0   0  0;...   
         0  -1   0  0 ;...
         0  0   1  L5 ;...
         0  0   0  1  ];
     
threshold = 1e-10;

MTH_0t =  vpa(mapSymType(MTH_01*MTH_12*MTH_23*MTH_34*MTH_45*MTH_56*tcp ,'rational', @(x) piecewise(abs(x)<=threshold, 0, x)));

q1=0.5; q2=0.2; q3=0.4; q4=0.5; q5=0; q6=1.5;
% q1=-pi/2; q2=0.3; q3=0; q4=pi/2; q5=0.4; q6=1.2;
% q1=0; q2=1; q3=-0.5; q4=2; q5=1; q6=0.5;
% q1=-1; q2=-0.3; q3=-pi/5; q4=0.4; q5=0.2; q6=1;

R_0t = eval(MTH_0t(1:3,1:3)); 

p = atan2(-(R_0t(3,1)),sqrt((R_0t(1,1))^2+(R_0t(2,1))^2))
r = atan2(R_0t(2,1)/cos(p),R_0t(1,1)/cos(p))
y = atan2(R_0t(3,2)/cos(p),R_0t(3,3)/cos(p))
pos =  eval(MTH_0t(1:3,4)')


tr2rpy(eval(MTH_0t))

%% CINEMATICA DIRECTA (Peter Corke RVC)
% Robot 6R
L1 = 486.5/100; L2 = 150/100; L3 = 700/100; L4=600/100; L5 = 65/100;     % Dimensiones
ws =  1.5* [-10 10 -10 10 -10 10];      % Espacio de trabajo
plot_options = {'workspace',ws,'scale',.5,'view',[125 25], 'tilesize',2,  'ortho',...
                'lightpos',[2 2 10] };

% Eslabones y articulaciones
%            Theta  d   a   alpha  type mdh offset  qlim
L(1) = Link('revolute', 'alpha', 0, 'a', 0, 'd', L1, 'offset', 0, 'modified');
L(2) = Link('revolute', 'alpha', pi/2, 'a', L2, 'd', 0, 'offset', pi/2, 'modified');
L(3) = Link('revolute', 'alpha', 0, 'a', L3, 'd', 0, 'offset', 0, 'modified');
L(4) = Link('revolute', 'alpha', pi/2, 'a', 0, 'd', L4, 'offset', 0, 'modified');
L(5) = Link('revolute', 'alpha', -pi/2, 'a', 0, 'd', 0, 'offset', 0, 'modified');
L(6) = Link('revolute', 'alpha', pi/2, 'a', 0, 'd', 0, 'offset', 0, 'modified');

% Representación del robot     
Robot = SerialLink(L,'name', 'Robot_{6R}', 'plotopt', plot_options);
Robot.tool = [  -1  0   0  0;...     % Nuevo valor TCP
                 0  -1   0  0 ;...
                 0  0   1  L5 ;...
                 0  0   0  1  ];

% Representación del robot
Robot           % Mostrar parametros del robot
%q = [0.5 0.2 0.4 0.5 0 1.5];    % Valores iniciales de q - movimiento de las articulaciones
q = [-1.7660 0.662 -0.1043 -2.3387 1.4182 -2.3503];
%q = [-pi/2 0.3 0 pi/2 0.4 1.2];
%q = [0 1 -0.5 2 1 0.5];
%q = [-1 -0.3 -pi/5 0.4 0.2 1];
Robot.teach(q)  % Dibujar e interactuar con el robot

TCP = Robot.fkine(q);
tr2rpy(TCP,'zyx')
transl(TCP)

%% CINEMATICA DIRECTA (RST)
% Importar robot
%robotRBT = loadrobot("abbIrb1600",'DataFormat','row','Gravity',[0 0 -9.81]);
%show(robotRBT);
%transform = getTransform(robotRBT,randomConfiguration(robotRBT),'L2','L6')
L1 = 486.5/100; L2 = 150/100; L3 = 700/100; L4=600/100; L5 = 65/100;     % Dimensiones

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');  
tform = trvec2tform([0, 0, L1]); % User defined 
setFixedTransform(jnt1,tform); 
body1.Joint = jnt1; 
robot = rigidBodyTree;
addBody(robot,body1,'base')
body2 = rigidBody('body2'); 
jnt2 = rigidBodyJoint('jnt2','revolute'); 
tform2 = trvec2tform([L2, 0, 0])*eul2tform([pi/2, -pi/2, 0]); % User defined 
setFixedTransform(jnt2,tform2); 
body2.Joint = jnt2; 
addBody(robot,body2,'body1'); % Add body2 to body1
body3 = rigidBody('body3'); 
body4 = rigidBody('body4'); 
jnt3 = rigidBodyJoint('jnt3','revolute'); 
jnt4 = rigidBodyJoint('jnt4','revolute'); 
tform3 = trvec2tform([L3, 0, 0]); % User defined 
tform4 = trvec2tform([0, -L4, 0])*eul2tform([0, 0, pi/2]); % User defined 
setFixedTransform(jnt3,tform3); 
setFixedTransform(jnt4,tform4); 
body3.Joint = jnt3;
body4.Joint = jnt4;
addBody(robot,body3,'body2'); % Add body3 to body2 
addBody(robot,body4,'body3'); % Add body4 to body3
body5 = rigidBody('body5'); 
body6 = rigidBody('body6'); 
jnt5 = rigidBodyJoint('jnt5','revolute'); 
jnt6 = rigidBodyJoint('jnt6','revolute'); 
tform5 = eul2tform([0, 0, -pi/2]); % User defined 
tform6 = eul2tform([0, 0, pi/2]); % User defined 
setFixedTransform(jnt5,tform5); 
setFixedTransform(jnt6,tform6);  
body5.Joint = jnt5;
body6.Joint = jnt6;
addBody(robot,body5,'body4'); % Add body5 to body4 
addBody(robot,body6,'body5'); % Add body6 to body5

bodyEndEffector = rigidBody('endeffector'); 
tform7 = trvec2tform([0, 0, L5])*eul2tform([pi, 0, 0]); % User defined 
setFixedTransform(bodyEndEffector.Joint,tform7); 
addBody(robot,bodyEndEffector,'body6');

JointName = {'jnt1' 'jnt2' 'jnt3' 'jnt4' 'jnt5' 'jnt6'}; 
JointPosition = {0 0 0 0 0 0};
%JointPosition = {0.5 0.2 0.4 0.5 0 1.5};
%JointPosition = {-pi/2 0.3 0 pi/2 0.4 1.2};
%JointPosition = {0 1 -0.5 2 1 0.5};
%JointPosition = {-1 -0.3 -pi/5 0.4 0.2 1};
config = struct('JointName',JointName,'JointPosition',JointPosition);
 
tform = getTransform(robot,config,'endeffector','base'); %MTH
showdetails(robot);
show(robot)
tform2trvec(tform) %Vector de traslacion 
tform2eul(tform) %Euler ZYX = Angulos fijos 

