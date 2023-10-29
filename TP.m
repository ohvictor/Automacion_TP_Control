%% Constants
clear all;
clc

l = 1;  %m
B = 1;  %N/(m/s)
m = 1;  %kg

%% Wall
vertices = [
    2 0 -1;
    0 2 -1;
    0 2 1;
    2 0 1
    ];
plot_poly(vertices', 'fillcolor','bo','edgecolor','red');

%% Links
L(1) = Revolute('a',1, 'B',1, 'm',1, 'r',[1 0 0]);
L(2) = Revolute('a',1, 'B',1, 'm',1, 'r',[2 0 0]);

%% Robot
KR = SerialLink(L);
KR.name = "robot";
KR.teach();
xaxis(-2,2);
yaxis(-2,2);

