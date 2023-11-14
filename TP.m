clear variables;
close all;
clc

%% Generación de robots
% Devuelve robot y robot_dis (80%)
gen_robot

%% Parámetros de simulación
T = 2;
step = T/1e2;
t0 = 0;

t = (t0:step:T)';

%% Generación de Trayectoria
% Puntos inicial y final
Ti = transl(1, -1, 0)*trotz(0);
Tf = transl(1, 1, 0)*trotz(pi/2);

% Trayectoria cartesiana
Tcart = ctraj(Ti, Tf, length(t));
q0 = [0 -pi/2];

% Trayectoria en espacio de joints
% Posiciones angulares de Joint
qc = robot.ikine(Tcart,q0,'mask',mask);

% Velocidades angulares de Joint
qcd = zeros(length(t),2);
qcd(2:end,:)= diff(qc)/step;

% Aceleraciones angulares de Joint
qcdd = zeros(length(t),2);
qcdd(2:end,:)= diff(qcd)/step;

%% Simulación

% Actualización de trayectoria en Simulink
sim_q = work_prep(t,qc);
sim_qd = work_prep(t,qcd);
sim_qdd = work_prep(t,qcdd);

% Sistema Críticamente amortiguado
eta = 1;
setting_time = step;
wn = setting_time/eta;

% Ecuación Característica s^2 + 2·e·wn + wn^2
Kp = (wn^2) * eye(2);
%Kv = 10 * eye(2);
Kv = 2*sqrt(Kp);

%% Ejecución
simout = sim('model_norm');
q_out_norm = simout.q_out.Data;

simout = sim('model_pert');
q_out_pert = simout.q_out.Data;

%% Preparación del ambiente 3D
plot_poly(vertices','animate', 'fillcolor','white','edgecolor','red');

plot3(Pcart(1,:),Pcart(2,:),Pcart(3,:),'LineWidth',2);
plot3(Preal(1,:),Preal(2,:),Preal(3,:),'LineWidth',1,'Color', 'g');
%% Animación del Robot final
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

%% Trayectoria generada por error numérico de joints
Treal = robot.fkine(qc).T;

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