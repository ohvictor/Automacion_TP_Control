%Completo - Normal%

clear all

%% Un gráfico que muestre los ángulos de "joints" en función del tiempo.

x = out.q.Time;
y = out.q.Data*57.2958; % To Radians

plot(x,y)

xlabel('Tiempo (s)');
ylabel('Ángulo (º)');
legend('Joint 1', 'Joint 2');
grid on;
grid minor;


%% Un gráfico que muestre la posición del efector final en espacio cartesiano en función del tiempo.

x = out.q.Time;
EE = double(EndEffector);
EEx_temp = EE(1,4,:);
EEy_temp = EE(2,4,:);

EEx = EEx_temp(:);
EEy = EEy_temp(:);

% Creating subplots
subplot(2, 1, 1); % 2 rows, 1 columns, first subplot
plot(x, EEx ,time, x_O_Tcart);
title('End Effector - Posicion Cartesiana X');
legend('Real', 'Teorico');
xlabel('Tiempo (s)');
ylabel('Posicion (m)');
grid on;
grid minor;

subplot(2, 1, 2); % 2 rows, 2 columns, second subplot
plot(x, EEy, time, y_O_Tcart);
title('End Effector - Posicion Cartesiana Y');
legend('Real', 'Teorico')
xlabel('Tiempo (s)');
ylabel('Posicion (m)');;
grid on;
grid minor;


%% Un gráfico que muestre la posición del efector final en espacio cartesiano en el plano XY (overhead view).

EE = double(EndEffector);
EEx_temp = EE(1,4,:);
EEy_temp = EE(2,4,:);

EEx = EEx_temp(:);
EEy = EEy_temp(:);

plot(EEx,EEy,x_O_Tcart, y_O_Tcart);
legend('Real', 'Teorico')
xlabel('X (m)');
ylabel('Y (m)');
grid on;
grid minor;

%% Un gráfico que muestre el movimiento del manipulador.


%% Un gráfico que muestre la fuerza deseada y la ejercida por el manipulador en función del tiempo, dada una pose inicial del robot que requiera un transitorio.

x = out.q.Time;
y = out.Fe.Data; 

P1=[0 10];
P2=[2 10];

plot(x,y(:,1),[P1(1) P2(1)],[P1(2) P2(2)]);

xlabel('Tiempo (s)');
ylabel('Fuerza(N)');
legend('Real', 'Teorico');
grid on;
grid minor;



%% Volver a generar los resultados de los puntos 3a, 3b, 3c, 3d y 3e cuando se perturba el
%modelo del robot un 80 %.