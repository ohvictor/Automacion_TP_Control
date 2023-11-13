%% Constants
clear variables;
close all;
clc

l = 1;  %m
B = 1;  %N/(m/s)
m = 1;  %kg

fd = 10; %N
Kenv = 1000*1e3; %N/m

mask = [1 1 0 0 0 0];
%% Wall
vertices = [
    2 0 -1;
    0 2 -1;
    0 2 1;
    2 0 1   ];
%% Links, Tool y Robot
L(1) = Revolute('a',1, 'B',B, 'm',m, 'r',[1 0 0]);
L(2) = Revolute('a',1, 'B',B, 'm',m, 'r',[1 0 0]);
Tool = transl([0 0 0]);

robot = SerialLink(L, 'tool', Tool);
robot.name = "RR";

%% Posición inicial y final
q0 = [pi/2,-pi/2];

Ti_ = transl(1,1,0);
Tf_ = transl(1-sqrt(1/2),1+sqrt(1/2),0);
qi = robot.ikine(Ti_,'mask',mask,'q0',q0);
qf = robot.ikine(Tf_,'mask',mask,'q0',qi);
Ti = robot.fkine(qi);
Tf = robot.fkine(qf);

%robot.teach(qi)

%% Trayectoria cartesiana
step = 1e-2;
time = (0:step:2)';
Tcart = ctraj(Ti,Tf,length(time));
q = robot.ikine(Tcart,'q0',q0,'mask',mask);
qd = zeros(length(time),2);
qdd = zeros(length(time),2);
qd(2:end,:)= diff(q)/step;
qdd(2:end,:)= diff(qd)/step;

%robot.plot(q)
%% Transformación a ejes ortogonales a la pared


%%
sim_q.time = time;
sim_qd.time = time;
sim_qdd.time = time;
sim_q.signals.values = q;
sim_qd.signals.values = qd;
sim_qdd.signals.values = qdd;

%%
OWT = transl(1,1,0)*trotz(pi/4);
WOT = trotz(-pi/4)*transl(-1,-1,0);

W_Tcart = transl(SE3(WOT.*(double(Tcart))));




