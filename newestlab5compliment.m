

set(0,'DefaultFigureWindowStyle','docked')
gp7_ = GP7;
gp7_.model.base = transl(0.3,0.5,0.5);
hold on
gp7_.PlotAndColourRobot();
q_current = gp7_.returnRobotJoints();
bot = gp7_.model;


%% Create sphere
sphereCenter = [1,1,0.5];
radius = 0.2;
[X,Y,Z] = sphere(20);
X = X * radius + sphereCenter(1);
Y = Y * radius + sphereCenter(2);
Z = Z * radius + sphereCenter(3);

%% Plot it
% Plot point cloud
points = [X(:),Y(:),Z(:)];
spherePc_h = plot3(points(:,1),points(:,2),points(:,3),'r.'); pause
delete (spherePc_h)

% % Or a triangle mesh
tri = delaunay(X,Y,Z);
sphereTri_h = trimesh(tri,X,Y,Z);
drawnow();
view(3)
axis equal

%% Move Robot

pose1 = transl(1.3, 1,0);
steps = 50;
gripperInitial = zeros(1,6);
gp7_ikcon = gp7_.model.ikcon(pose1);
gripperInitial = [gripperInitial;gp7_ikcon];
gp7qmatrix = jtraj(gripperInitial(1,:),gripperInitial(2,:),steps);

for i=1:steps
    animate(gp7_.model, gp7qmatrix(i,:));
      if CheckCollision(bot,sphereCenter,radius) == 1
     break
    end
    drawnow();
end
 q_current = gp7_.returnRobotJoints();
 jointangles = q_current*180/pi;
   
  
