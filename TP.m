%% Constants
clear variables;
close all;
clc

l = 1;  %m
B = 1;  %N/(m/s)
m = 1;  %kg

mask = [1 1 0 0 0 0];
%% Wall
vertices = [
    2 0 -1;
    0 2 -1;
    0 2 1;
    2 0 1   ];
%% Links y Tool
L(1) = Revolute('a',1, 'B',B, 'm',m, 'r',[1 0 0]);
L(2) = Revolute('a',1, 'B',B, 'm',m, 'r',[1 0 0]);
Tool = transl([0 0 0]);

%% Robot
robot = SerialLink(L, 'tool', Tool);
robot.name = "RR";
% KR.teach();
xaxis(-2,2);
yaxis(-2,2);

%% Posiciones Inicial
Ti = transl(1, -1, 0)*trotz(0);
Tf = transl(1, 1, 0)*trotz(pi/2);

%% Simulación
T = 2;
step = T/1e2;
t0 = 0;

t = (t0:step:T)';
%% Trayectoria Cartesiana
Tcart = ctraj(Ti, Tf, length(t));
q0 = [0 -pi/2];

qc = robot.ikine(Tcart,q0,'mask',mask);
qcd = zeros(length(t),2);
qcdd = zeros(length(t),2);

qcd(2:end,:)= diff(qc)/step;
qcdd(2:end,:)= diff(qcd)/step;

Treal = robot.fkine(qc).T;

%% Preparo la trayectoria para Simulink
sim_q = work_prep(t,qc);
sim_qd = work_prep(t,qcd);
sim_qdd = work_prep(t,qcdd);
%% Preparación del ambiente 3D
Pcart = squeeze(Tcart(1:3,4,:));
Preal = squeeze(Treal(1:3,4,:));
hold on
plot3(Pcart(1,:),Pcart(2,:),Pcart(3,:),'LineWidth',2);
plot3(Preal(1,:),Preal(2,:),Preal(3,:),'LineWidth',1,'Color', 'g');
plot_poly(vertices','animate', 'fillcolor','white','edgecolor','red');

%% Animación del Robot final
anim = Animate('movie.mp4');
for i=1:length(t)
    robot.plot(qc(i,:));
    anim.add();
end

for i=length(t):-1:1
    robot.plot(qc(i,:));
    anim.add();
end
anim.close();

hold off

%% Gráficos Joints
figure(2);
% Q1 vs t
subplot(2,1,1);
plot(t,qc(:,1)/pi*180);
ylim([-15,105]);
yticks([0 30 60 90]);
ylabel('Ángluo (º)');
title('Joint 1');
grid on;

% Q2 vs t
subplot(2,1,2);
plot(t,qc(:,2)/pi*180);
ylim([-130,-80]);
yticks([-120 -105 -90]);
ylabel('Ángluo (º)');

xlabel('Tiempo (s)');
title('Joint 2');
grid on;

%% Gráficos Cartesianos
% X,Xreal vs t
figure(3);
subplot(2,1,1);
x_t = Tcart(1,4,:);
x_t = squeeze(x_t);

x_r = Treal(1,4,:);
x_r = squeeze(x_r);

plot(t,[x_t x_r]);
title('Posición Cartesiana X');
ylabel('Posición X (m)');
ylim([0 2]);
grid on;
legend('Teórico','Real');
% Y,Yreal vs t
subplot(2,1,2);
y_t = Tcart(2,4,:);
y_t = squeeze(y_t);

y_r = Treal(2,4,:);
y_r = squeeze(y_r);

plot(t,[y_t, y_r]);
title('Posición Cartesiana Y');
ylabel('Posición Y (m)');
xlabel('Tiempo (s)');
grid on;
legend('Teórico','Real');

%% Gráficos errores Cartesianos
% X,Xreal vs t
figure(4);
subplot(2,1,1);

plot(t,x_r-x_t);
title('Error Posición Cartesiana X');
ylabel('Error Posición X (m)');
grid on;

% Y,Yreal vs t
subplot(2,1,2);
y_t = Tcart(2,4,:);
y_t = squeeze(y_t);

y_r = Treal(2,4,:);
y_r = squeeze(y_r);

plot(t,y_r-y_t);
title('Error Posición Cartesiana Y');
ylabel('Error Posición Y (m)');
xlabel('Tiempo (s)');
grid on;

%% Gráficos XY
figure(5);
plot(x_t,y_t, x_r,y_r);
xlim([-2 2])
ylim([-2 2]);
grid on;
xlabel('X (m)');
ylabel('Y (m)');
legend('Teórico','Real');
title('Posición del End Effector');