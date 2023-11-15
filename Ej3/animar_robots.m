%% Preparación del ambiente 3D robot normal
% Dibujar pared
plot_poly(vertices','animate', 'fillcolor','white','edgecolor','red');
hold on
title('Robot normal')
% Obtener puntos en 3D de la trayectoria del EE
[~,t_in_n] = tr2rt(T_in_norm);
[~,t_out_n] = tr2rt(T_out_norm);

plot3(t_in_n(:,1),t_in_n(:,2),t_in_n(:,3),'LineWidth',4,'Color','r');
plot3(t_out_n(:,1),t_out_n(:,2),t_out_n(:,3),'LineWidth',2,'Color','w');
%% Animación del Robot normal
hold on;
anim = Animate('cart_normal.mp4');
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

%{
%% Preparación del ambiente 3D robot perturbado
% Dibujar pared
plot_poly(vertices','animate', 'fillcolor','white','edgecolor','red');
title('Robot perturbado');
hold on
% Obtener puntos en 3D de la trayectoria del EE
[~,t_in_p] = tr2rt(T_in_pert);
[~,t_out_p] = tr2rt(T_out_pert);

plot3(t_in_p(:,1),t_in_p(:,2),t_in_p(:,3),'LineWidth',4,'Color','r');
plot3(t_out_p(:,1),t_out_p(:,2),t_out_p(:,3),'LineWidth',2,'Color','w');

%% Animación del Robot perturbado
hold on;
anim = Animate('cart_perturbado.mp4');
for i=1:length(t)
    robot_dis.plot(q_out_pert(i,:));
    anim.add();
end

for i=length(t):-1:1
    robot_dis.plot(q_out_pert(i,:));
    anim.add();
end
anim.close();

hold off
%}