%% Constants
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
L(1) = Revolute('a',0, 'B',B, 'm',m, 'r',[1 0 0],'modified');
L(2) = Revolute('a',1, 'B',B, 'm',m, 'r',[1 0 0],'modified');
Tool = transl([0 0 0]);

%% Robot
robot = SerialLink(L, 'tool', Tool);
robot.name = "RR";

%% Robot perturbado
robot_dis = robot.perturb(0.8);
robot_dis.name = "RR_{dis}";