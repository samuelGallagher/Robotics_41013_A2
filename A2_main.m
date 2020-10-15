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
tableZ = 0.84; %Standard Table Height

%% Insert Elements


%Change this to UR5 for testing purposes
gp7_ = GP7;
gp7_.model.base = transl(-0.25,0.15,0.84);
hold on
%Plot as per UR5 function
gp7_.PlotAndColourRobot();
%gp7_.model.base.getpos()

workspace = [-3 3 -2 2 0 2] %Fit around the space of the lab set

q_current = gp7_.returnRobotJoints()
%% Cake Locations

workspace = [-1 1 -1 1 0 1]; %Fit around the space of the lab set

%Tray location, decided previously
tray_x = -0.8;
tray_y = 0.14;
tray_z = 0.89;
tray_location = transl(tray_x, tray_y, tray_z);

%The purpose of this section is to select the cupcake locations from a set
%of 12 options.
%Cake Location Array
Cake_Locations= [tray_x,     tray_y+0.05,    tray_z...
                 ;tray_x,     tray_y+0.1,    tray_z...
                 ;tray_x,     tray_y-0.05,    tray_z...
                 ;tray_x,     tray_y-0.1,    tray_z...
                 %Left Row
                 ;tray_x-0.09,     tray_y+0.05,    tray_z...
                 ;tray_x-0.09,     tray_y+0.1,    tray_z...
                 ;tray_x-0.09,     tray_y-0.05,    tray_z...
                 ;tray_x-0.09,     tray_y-0.1,    tray_z...
                 %Right Row
                 ;tray_x+0.09,     tray_y+0.05,    tray_z...
                 ;tray_x+0.09,     tray_y+0.1,    tray_z...
                 ;tray_x+0.09,     tray_y-0.05,    tray_z...
                 ;tray_x+0.09,     tray_y-0.1,    tray_z];

%Replace this hard coded decision with random elements via the gui. 
%An array of cake placemenets is made using the 12 options from
%Cake_Locations.
    
rand1=1;
rand2=4;
rand3=11;

cakePlace(:,:,1) = transl(Cake_Locations(rand1,1),Cake_Locations(rand1,2),Cake_Locations(rand1,3));
cakePlace(:,:,2) = transl(Cake_Locations(rand2,1),Cake_Locations(rand2,2),Cake_Locations(rand2,3));
cakePlace(:,:,3) = transl(Cake_Locations(rand3,1),Cake_Locations(rand3,2),Cake_Locations(rand3,3));

%Example of X in 1st cakePlace
%cakePlace(1,4,i);

cakePlace(:,:,:);


Tray(num2str(1),tray_location,workspace);
for j = 1:3
cake_placement = transl(cakePlace(1,4,j),cakePlace(2,4,j),cakePlace(3,4,j))

    
num2str(j)
cake_placement
workspace
Cake(num2str(j),cake_placement,workspace);
        
        
end
%% Insert Simulated Environment
% *****Replace this section with a class function, with choice of simple or
% complex environment*************

%Insert simulation environment 'labview.ply'
[f,v,data] = plyread('lab_set_low.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
hold on;
% plot the trisurf (offset variables not used)
trisurf(f,v(:,1),v(:,2),+v(:,3),'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat', 'linestyle','none');

axis equal

%Insert simulated oven with same method
[f,v,data] = plyread('oven_low.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on
[f,v,data] = plyread('oven_door_low.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on
%Workspace dimensions

%GP7_.model.plotopt = { 'noname', 'noshadow', 'nowrist','workspace',workspace,'scale',0.2};
%GP7_.model.plot([0 0 0 0 0 0 0]);
drawnow;

%% Minor movement 
%Currently works with UR5 and not GP7 class
% Moves from origin to aim1 to aim2 

text_message = ("This is text for placement")

%Translation for aim1 and aim2
aim1 = transl(-0.8, 0.15, 0.85);
aim2 = transl(0.5, 0.15,0.85);
steps = 50;

%Set first row in plan to origin aka joints at 0
gp7_q_plan = zeros(1,6);
%ikcon calculates q plan required for first position
gp7_ikcon = gp7_.model.ikcon(aim1);
%Adds calculations to plan
gp7_q_plan = [gp7_q_plan;gp7_ikcon];

%Ditto for aim2
gp7_ikcon = gp7_.model.ikcon(aim2);
gp7_q_plan = [gp7_q_plan;gp7_ikcon];



%Trajectory between first and second row in qp7_q_plan
gp7qmatrix = jtraj(gp7_q_plan(1,:),gp7_q_plan(2,:),steps);

for i=1:steps
    animate(gp7_.model, gp7qmatrix(i,:));
    drawnow();

end
pause(0.5);
%Trajectory between second and third row in qp7_q_plan

gp7qmatrix = jtraj(gp7_q_plan(2,:),gp7_q_plan(3,:),steps);

for i=1:steps
    animate(gp7_.model, gp7qmatrix(i,:));
    drawnow();

end

q_current = gp7_.returnRobotJoints();

%gp7_.RMRC(0.3, 0.8, 0.85);



            %CakeSlotChoice(5);
             %           CakeSlotChoice(9);
            %CakeSlotChoice(7);
            %pause(3);
            %CakeLocationSpawn()