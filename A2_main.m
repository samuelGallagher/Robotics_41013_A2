%Assignment 2 Main

%Student Names: Stephanie Baxter, Samuel Gallagher


% High Priorities needed:
% GUI Implementation





%% Reset Workspace
clear all
close all
clc
%% Definitions

%Don't keep this
tableZ = 0.930; %Standard Table Height

%% Insert Elements
%Insert UR5 robot arm

%ur7_ = GP7(false);
%ur7_.model.base = transl(-0.3,0,0.93)* trotx(pi/2) * troty(pi/2) * trotz(0);
hold on
%Plot as per UR5 function
%GP7_.PlotAndColourRobot();

%% Insert Simulated Environment
% *****Replace this section with a class function, with choice of simple or
% complex environment*************

%Insert simulation environment 'labview.ply'
[f,v,data] = plyread('labview.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
hold on;
% plot the trisurf (offset variables not used)
trisurf(f,v(:,1),v(:,2),+v(:,3),'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat', 'linestyle','none');

axis equal

%Workspace dimensions

workspace = [-3 3 -2 2 0 2] %Fit around the space of the lab set
%GP7_.model.plotopt = { 'noname', 'noshadow', 'nowrist','workspace',workspace,'scale',0.2};
%GP7_.model.plot([0 0 0 0 0 0 0]);
drawnow;