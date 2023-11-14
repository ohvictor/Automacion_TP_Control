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
L(1) = Revolute('a',1, 'B',B, 'G', 1, 'm',m, 'r',[1 0 0]);
L(2) = Revolute('a',1, 'B',B, 'G', 1, 'm',m, 'r',[1 0 0]);
Tool = transl([0 0 0]);

robot = SerialLink(L, 'tool', Tool);
robot.name = "RR";

%% Posición inicial y final
q0 = [pi/2,-pi/2];

%Ti_ = transl(1,1,0);
%Tf_ = transl(1-sqrt(1/2),1+sqrt(1/2),0);
%qi = robot.ikine(Ti_,'mask',mask,'q0',q0);
%qf = robot.ikine(Tf_,'mask',mask,'q0',qi);
%Ti = robot.fkine(qi);
%Tf = robot.fkine(qf);

%robot.teach(qi)

%% Trayectoria cartesiana en ejes ortogonales a la pared
OW_T = transl(1,1,0)*trotz(pi/4);
WO_T = trotz(-pi/4)*transl(-1,-1,0);
OW_R2x2 = OW_T(1:2,1:2);
WO_R2x2 = WO_T(1:2,1:2);

step = 1e-2;
time = (0:step:2)';


W_Ti = transl(0,0,0);
W_Tf = transl(0,1,0);
W_Tcart = ctraj(W_Ti,W_Tf,length(time));

%% Transformación a ejes ortogonales a la pared
%O_Tcart = SE3(OW_T.*double(W_Tcart));
for i = 1:length(time)
    O_Tcart(:,:,i) = OW_T*W_Tcart(:,:,i);
end

q = robot.ikine(O_Tcart,'q0',q0,'mask',mask);
qd = zeros(length(time),2);
qdd = zeros(length(time),2);
qd(2:end,:)= diff(q)/step;
qdd(2:end,:)= diff(qd)/step;

%robot.plot(q)



%%
sim_q.time = time;
sim_qd.time = time;
sim_qdd.time = time;
sim_q.signals.values = q;
sim_qd.signals.values = qd;
sim_qdd.signals.values = qdd;

%%
sim_x.time = time;
sim_xd.time = time;
sim_xdd.time = time;


sim_x_SE3 = transl(SE3(W_Tcart));
sim_x.signals.values = sim_x_SE3(:,1:2);
sim_xd.signals.values = zeros(length(time),2);
sim_xd.signals.values(2:end,:) = diff(sim_x.signals.values)/step;
sim_xdd.signals.values = zeros(length(time),2);
sim_xdd.signals.values(2:end,:) = diff(sim_xd.signals.values)/step;

%%
figure(1)
plot(sim_x.time, sim_x.signals.values)
legend('W_x','W_y');
figure(2)
plot(sim_xd.time, sim_xd.signals.values)
legend('W_xd','W_yd');
figure(3)
plot(sim_xdd.time, sim_xdd.signals.values)
legend('W_xdd','W_ydd');

%%

% TAUG = R.gravload(Q)
% I = R.inertia(Q)
% C = R.coriolis(Q,QD)
% TAU = R.friction(QD)

%%
Kv = 2000;
Kp = 100;

Kvf = 1000;
Kpf = 200000;