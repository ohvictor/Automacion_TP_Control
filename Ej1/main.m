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
simout_norm = sim('model_norm', 'FixedStep',num2str(step));
q_out_norm = simout_norm.q_out.Data;
T_in_norm = simout_norm.T_in.Data;
T_out_norm = simout_norm.T_out.Data;

simout_pert = sim('model_pert');
q_out_pert = simout_pert.q_out.Data;
T_in_pert = simout_pert.T_in.Data;
T_out_pert = simout_pert.T_out.Data;

%% Script para armar videos de los robots. 
%animar_robots

%% Trayectoria generada por error numérico de joints
Treal = robot.fkine(qc).T;

%% Graficar el robot normal
T_in = T_in_norm; %#ok<NASGU>
T_out = T_out_norm; %#ok<NASGU>
q_in = qc; %#ok<NASGU>
q_out = q_out_norm; %#ok<NASGU>

graficar_robots

%% Graficar el robot perturbado
T_in = T_in_pert;
T_out = T_out_pert;
q_in = qc;
q_out = q_out_pert;

graficar_robots