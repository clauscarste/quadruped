%this fct is used to calculate the matix to add roll_pich_yaw controll and ajust for the diferent orientation of the leg cordinate frames

syms yaw pich roll xm ym zm body_parametersL body_parametersW

rotx = [1 0 0 0; 0 cos(roll) -sin(roll) 0 ; 0 sin(roll) cos(roll) 0; 0 0 0 1];
roty = [cos(yaw) 0 sin(yaw) 0; 0 1 0 0; -sin(yaw) 0  cos(yaw) 0;0 0 0 1];
rotz = [cos(pich) -sin(pich) 0 0; sin(pich) cos(pich) 0 0; 0 0 1 0; 0 0 0 1];

T = (rotx*roty*rotz)*[1 0 0 xm; 0 1 0 ym; 0 0 1 zm; 0 0 0 1];

T0 = T*[0 0 sin(pi/2) -body_parametersL/2; 0 1 0 0; -sin(pi/2) 0 0 body_parametersW/2; 0 0 0 1]
T1 = T*[0 0 sin(pi/2) body_parametersL/2; 0 1 0 0; -sin(pi/2) 0 0 body_parametersW/2; 0 0 0 1]
T2 = T*[0 0 sin(-pi/2) body_parametersL/2; 0 1 0 0; -sin(-pi/2) 0 0 -body_parametersW/2; 0 0 0 1]
T3 = T*[0 0 sin(-pi/2) -body_parametersL/2; 0 1 0 0; -sin(-pi/2) 0 0 -body_parametersW/2; 0 0 0 1]

