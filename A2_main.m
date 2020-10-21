
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
gp7_.PlotAndColourRobot();
workspace = [-3 3 -2 2 0 2] %Fit around the space of the lab set
q_current = gp7_.returnRobotJoints()

%% Light Curtain
planeXntersect1 = 1.3;
planeBounds = [planeXntersect1-eps,planeXntersect1+eps,0,1.4,0,1.5]; 
[Y,Z] = meshgrid(planeBounds(3):0.1:planeBounds(4),planeBounds(5):0.1:planeBounds(6));
X = repmat(planeXntersect1,size(Y,1),size(Y,2));
s = meshz(X,Y,Z,'FaceAlpha',0.3)

planeXntersect2 = -2.6;
planeBounds = [planeXntersect2-eps,planeXntersect2+eps,0,1.4,0,1.5]; 
[Y,Z] = meshgrid(planeBounds(3):0.1:planeBounds(4),planeBounds(5):0.1:planeBounds(6));
X = repmat(planeXntersect2,size(Y,1),size(Y,2));
s = meshz(X,Y,Z,'FaceAlpha',0.3)

planeXntersect = 1.4;
planeBounds = [planeXntersect-eps,planeXntersect+eps,-2.6,1.3,0,1.5]; 
[X,Z] = meshgrid(planeBounds(3):0.1:planeBounds(4),planeBounds(5):0.1:planeBounds(6));
Y = repmat(planeXntersect,size(X,1),size(X,2));
s = meshz(X,Y,Z,'FaceAlpha',0.3)


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



%% %% 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Scotty
[f,v,data] = plyread('Scotty.ply','tri');
ScottyVertexCount = size(v,1);

% Move center point to origin
midPoint = sum(v)/ScottyVertexCount;
ScottyVerts = v - repmat(midPoint,ScottyVertexCount,1);

% Create a transform to describe the location (at the origin, since it's centered
ScottyPose = transl(0,2,0.5);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
ScottyMesh_h = trisurf(f,ScottyVerts(:,1),ScottyVerts(:,2), ScottyVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

% Move forwards (facing in -y direction)
forwardTR = makehgtform('translate',[0,-0.01,0]);
ScottyPose = ScottyPose * forwardTR 
updatedPoints = [ScottyPose * [ScottyVerts,ones(ScottyVertexCount,1)]']';  

% Now update the Vertices
ScottyMesh_h.Vertices = updatedPoints(:,1:3)

% Now move it many times
while 1
pose1 = transl(0, 1.5,0.85);
steps = 75;
gripperInitial = zeros(1,6);
gp7_ikcon = gp7_.model.ikcon(pose1);
gripperInitial = [gripperInitial;gp7_ikcon];
gp7qmatrix = jtraj(gripperInitial(1,:),gripperInitial(2,:),steps);

for i=1:steps
    animate(gp7_.model, gp7qmatrix(i,:));
    drawnow();
    ScottyPose = ScottyPose * forwardTR 
    updatedPoints = [ScottyPose * [ScottyVerts,ones(ScottyVertexCount,1)]']';
    % Update the mesh vertices in the patch handle
    ScottyMesh_h.Vertices = updatedPoints(:,1:3);
   
    if ScottyPose(2,4) <= 1.6;
        disp('WatchOut!')
       return
    end

   
    drawnow();   
end
drawnow;
end


