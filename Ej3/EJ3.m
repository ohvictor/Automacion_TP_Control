%% Constants
clear variables;
close all;
clc

l = 1;  %m
B = 1;  %N/(m/s)
m = 1;  %kg

fd = 10; %N
Kenv = 1000000;%000; %N/mm

mask = [1 1 0 0 0 0];
%% Wall
vertices = [
    2 0 -1;
    0 2 -1;
    0 2 1;
    2 0 1   ];
%% Links, Tool y Robot
L(1) = Revolute('a',0, 'B',B, 'm',m, 'r',[1 0 0],'modified');
L(2) = Revolute('a',1, 'B',B, 'm',m, 'r',[1 0 0],'modified');
Tool = transl([1 0 0]);

robot = SerialLink(L, 'tool', Tool);
robot.name = "RR";

robot_dis = robot.perturb(0.8);
robot_dis.name = "RR_{dis}";
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
%{
% Sistema Críticamente amortiguado
eta = 1;
setting_time = step;
wn = 4/(setting_time*eta);

% Ecuación Característica s^2 + 2·e·wn + wn^2
Kp = (wn^2);
%Kv = 10 * eye(2);
Kv = 2*sqrt(Kp);
%}

%%

%       m xdd + b xd + k x = 0
%       wn = sqrt(k/m)
%       eta = b / (2 sqrt(k m))
%       para eta = 1 -> wn = 5.8335... / Ts

%       edd + kv ed + kp e = 0
%       wn = sqrt(kp)
%       eta = kv / (2 sqrt(kp))

eta = 1;
Ts = 4*step;
wn = 5.8335 / Ts;
Kp = wn^2;
Kv = eta*(2*sqrt(Kp));

%%
Kvf = 800*sqrt(1000);
Kpf = 160000*1000;

%Kvf = Kv;
%Kpf = Kp;
%% 

ej3_offset = 1e-5; % 1e-6

%%

q_out_norm = out.q_out.Data;
t = out.q_out.Time;

hold on;
anim = Animate('movie.mp4');
for i=1:length(t)
    robot.plot(q_out_norm(i,:));
    anim.add();
end

for i=length(t):-1:1
    robot.plot(q_out_norm(i,:));
    anim.add();
end
anim.close();

hold off

%%

for i = 1:length(out.q.Time)
    %robot.inertia(out.q.Data(i,:)) - out.M.Data(:,:,i)
    %inv(out.J.Data(:,:,i))'*out.M.Data(:,:,i)*inv(out.J.Data(:,:,i)) - out.MX.Data(:,:,i)
    %out.alpha_f.Data(i,:)' - out.MX.Data(:,:,i)*out.f.Data(i,:)'
end


