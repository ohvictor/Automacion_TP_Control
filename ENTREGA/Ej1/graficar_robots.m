%% Angulos de Joints
figure(2);
% Qin,Qout 1 vs t
subplot(2,1,1);
plot(t,q_in(:,1)/pi*180,t,q_out(:,1)/pi*180);
ylim([-15,105]);
yticks([0 30 60 90]);
ylabel('Ángluo (º)');
title('Joint 1');
legend('Deseado','Obtenido','Location','southeast');
grid on;

% Qin,Qout 2 vs t
subplot(2,1,2);
plot(t,q_in(:,2)/pi*180,t,q_out(:,2)/pi*180);
ylim([-130,-80]);
yticks([-120 -105 -90]);
ylabel('Ángluo (º)');

xlabel('Tiempo (s)');
title('Joint 2');
legend('Deseado','Obtenido','Location','southeast');
grid on;

%% Gráficos Cartesianos
% X,Xreal vs t
figure(3);
subplot(2,1,1);
x_t = T_in(1,4,:);
x_t = squeeze(x_t);

x_r = T_out(1,4,:);
x_r = squeeze(x_r);

plot(t,[x_t x_r]);
title('Posición Cartesiana X');
ylabel('Posición X (m)');
ylim([0 2]);
grid on;
legend('Deseada','Obtenida','Location','southeast');

% Y,Yreal vs t
subplot(2,1,2);
y_t = T_in(2,4,:);
y_t = squeeze(y_t);

y_r = T_out(2,4,:);
y_r = squeeze(y_r);

plot(t,[y_t, y_r]);
title('Posición Cartesiana Y');
ylabel('Posición Y (m)');
xlabel('Tiempo (s)');
grid on;
legend('Deseada','Obtenida','Location','southeast');

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

plot(t,y_r-y_t);
title('Error Posición Cartesiana Y');
ylabel('Error Posición Y (m)');
xlabel('Tiempo (s)');
grid on;

%% Gráficos XY
figure(5);
plot(x_t,y_t, x_r,y_r);
xlim([-2 2]);
ylim([-2 2]);
grid on;
xlabel('X (m)');
ylabel('Y (m)');
legend('Teórico','Real');
title('Posición del End Effector');
