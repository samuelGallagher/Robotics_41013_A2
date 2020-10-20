classdef Spawn < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1;
        robot;
        tableZ;
        cakeArray=zeros(1,12);
        CurrentQValues;
        stop_status=0;
        tray;
        door;
        bottle;
        tray_x;
        tray_y;
        tray_z;
        bottle_x;
        bottle_y;
        bottle_z;
        Cake_Locations;
        bottle_location;
        workspace;
        cake_cache;
        bottle_cache;
        previous_RPY;
    end
    
    methods
        function self = Spawn()
            self.tableZ=0.84;
            cakeArray=zeros(1,12);
            stop_status=0;
            self.previous_RPY = zeros(1,3);
            self.robot = GP7(false);
            self.robot.model.base = transl(-0.25, 0.45, self.tableZ-0.2);
            self.CurrentQValues = zeros(1,6);
            self.workspace = [-1 1 -1 1 -0.3 1];
            %Tray location, decided previously
            self.tray_x = -0.62;
            self.tray_y = 0.45;
            self.tray_z = self.tableZ+0.04;
            tray_location = transl(self.tray_x, self.tray_y, self.tray_z);
            door_location = transl(0.41,0.64,0.17);
            self.bottle_x = -0.62;
            self.bottle_y = 0.1;
            self.bottle_z = self.tableZ+0.29;
            self.bottle_location = transl(self.bottle_x,self.bottle_y,self.bottle_z);
            self.robot.PlotAndColourRobot();%robot,workspace);
            
            self.tray = Tray(num2str(13), tray_location, self.workspace);
            self.door = Door(num2str(14),door_location,self.workspace);
            %self.bottle = Bottle(num2str(15),transl(0,0,0),self.workspace,'yellow');
            
            %origin_spawn(i) = Cake(num2str(i), transl(0,0,0), self.workspace, colour);
            
            
            %self.bottle = Bottle(num2str(15),bottle_location,self.workspace);
            %self.tray.tray.base = transl(0-1,1,0.5);
            %animate(self.tray.tray,0);
            %plot3d(self.tray.tray,0,'workspace',self.workspace,'view',[-30,30],'delay',0);
            
            % Insert Simulated Environment
            % *****Replace this section with a class function, with choice of simple or
            % complex environment*************
            
            %Insert simulation environment 'labview.ply'
            [f,v,data] = plyread('Bench.ply','tri');
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
            
            self.cake_cache = self.cakeDrop('yellow');
            self.bottle_cache = self.bottleDrop('yellow');
            self.bottle_cache(1).bottle.base = self.bottle_location;
            animate(self.bottle_cache(1).bottle,0);
            
            set(gca, 'CameraPosition', [-700 3000 1000]);
            
        end
        
        %Place cake as 1 in position that is given as positive
        %When called in GUI, this should result in a set of 0's and 1's
        %determining which cupcakes are used. example of first three is
        %[1 1 1 0 0 0 0 0 0 0 0 0]
        function CakeSlotChoice(self, id)
            for i = 1:12
                if i == id
                    self.cakeArray(1,id) = 1;
                end
                
            end
            
        end
        
        function origin_spawn = cakeDrop(self, colour)
            for i = 1:12
                origin_spawn(i) = Cake(num2str(i), transl(0,0,0), self.workspace, colour);
            end
        end
        function bottle_spawn = bottleDrop(self, colour)
            for i = 1:1
                bottle_spawn(i) = Bottle(num2str(15), transl(0,0,0), self.workspace, colour);
            end
        end
        function CakeLocationSpawn(self)
            
            %Defining the location for cupcakes
            self.Cake_Locations = zeros(12,3);
            %self.Cake_Locations(1,:) = self.tray_x, self.tray_y, self.tray_z
            
            self.Cake_Locations= [
                %Middle Row
                ;self.tray_x,     self.tray_y+0.04,    self.tray_z-0.04...            %11   Really 1
                ;self.tray_x,     self.tray_y+0.13,    self.tray_z-0.04...       %8    Really 2
                ;self.tray_x,     self.tray_y-0.04,    self.tray_z-0.04...       %5   Really 3
                ;self.tray_x,     self.tray_y-0.13,    self.tray_z-0.04...       %2   Really 4
                %Left Row
                ;self.tray_x-0.085,     self.tray_y+0.04,    self.tray_z-0.04...  %12   Really5
                ;self.tray_x-0.085,     self.tray_y+0.13,    self.tray_z-0.04...  %9   Really6
                ;self.tray_x-0.085,     self.tray_y-0.04,    self.tray_z-0.04...  %6   Really7
                ;self.tray_x-0.085,     self.tray_y-0.13,    self.tray_z-0.04...  %3   Really8
                %Right Row
                ;self.tray_x+0.085,     self.tray_y+0.04,    self.tray_z-0.04...  %7   Really9
                ;self.tray_x+0.085,     self.tray_y+0.13,    self.tray_z-0.04...  %10   Really10
                ;self.tray_x+0.085,     self.tray_y-0.04,    self.tray_z-0.04...  %4   Really11
                ;self.tray_x+0.085,     self.tray_y-0.13,    self.tray_z-0.04];   %1   Really12
            
            for i = 1:12
                if 1==self.cakeArray(1,i)
                    cakePlace(:,:,i) = transl(self.Cake_Locations(i,1),self.Cake_Locations(i,2),self.Cake_Locations(i,3));
                    cake_placement = transl(cakePlace(1,4,i),cakePlace(2,4,i),cakePlace(3,4,i));
                    
                    self.cake_cache(i).cake.base = transl(self.Cake_Locations(i,1),self.Cake_Locations(i,2),self.Cake_Locations(i,3));
                    %self.cake_cache.i.cake.base = transl(self.Cake_Locations(i,1),self.Cake_Locations(i,2),self.Cake_Locations(i,3));
                    %self.cake_cache(i) = Cake(num2str(i),cake_placement,self.workspace);
                    
                    %RMRC for location
                    if self.robot.mode == 3
                        self.SpawnRMRCPoint(cakePlace(1,4,i),cakePlace(2,4,i)+0.2,cakePlace(3,4,i)+0.2, -pi/2, 0, pi/2, 1.5);
                        pause(1);
                    end
                    
                    animate(self.cake_cache(i).cake,0);
                    %Cake(num2str(i),cake_placement,self.workspace);
                    
                end
                %set(gca, 'CameraPosition', [-700 3000 1000]);
                
            end
            
            
            
        end
        %Nathan Help
        function RobotPoseAnimate(self,q)
            self.CurrentQValues = q;
            self.robot.model.animate(q);
            drawnow();
        end
        function stopChange(self, state)
            if state == 1
                self.stop_status = 1;
            else
                self.stop_status = 0;
            end
        end
        
        
        
        function ColourChoice(self, colour);
            if contains('yellow', colour)
                self.bottle_cache = self.bottleDrop('yellow');
                self.cake_cache = self.cakeDrop('yellow');
            end
            if contains('red', colour)
                self.bottle_cache = self.bottleDrop('yellow');
                self.cake_cache = self.cakeDrop('red');
            fprintf('Colour Choice Red Specifically');
            end
            if contains('blue', colour)
                self.bottle_cache = self.bottleDrop('yellow');
                self.cake_cache = self.cakeDrop('blue');
            end
            self.bottle_cache(1).bottle.base = self.bottle_location;
            animate(self.bottle_cache(1).bottle,0);
            fprintf('Colour Choice End');
        end
        
        
        
        function SpawnRMRCPoint(self, xend, yend, zend, end_roll, end_pitch, end_yaw, t)
            %function Lab9Solution_Question1(self, xstart, ystart, zstart, xend, yend, zend)
            %q = self.model.getpos();
            q_start = self.robot.model.getpos()
            Transform = self.robot.model.fkine(q_start);
            xstart = Transform(1,4);
            ystart = Transform(2,4);
            zstart = Transform(3,4);
            % http://planning.cs.uiuc.edu/node103.html
            %rotm = tform2rotm(T);
            %eulZYX = rotm2eul(rotm);
            %start_roll = eulZYX(1,1)
            %start_pitch = eulZYX(1,2)
            %start_yaw = eulZYX(1,3)
            start_roll = self.previous_RPY(1,1);
            start_pitch = self.previous_RPY(1,2);
            start_yaw = self.previous_RPY(1,3);
            %T_goal = self.model.fkine(q_goal);
            %xend = T_goal(1,4);
            %yend = T_goal(2,4);
            %zend = T_goal(3,4);
            % 1.1) Set parameters for the simulation
            t = 5;             % Total time (s)
            %t;
            deltaT = 0.05;      % Control frequency
            steps = t/deltaT;   % No. of steps for simulation
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.05;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
            
            % 1.2) Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,6);       % Array for joint anglesR
            qdot = zeros(steps,6);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory
            
            % 1.3) Set up trajectory, initial pose
            %Possible have a case system for types of trajectory (stay
            %parallel to ground, etc)
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i=1:steps
                %Do if approach = straight, x curve etc.
                x(1,i) = (1-s(i))*xstart + s(i)*xend; % Points in x
                if self.robot.mode ==4
                    x(2,i) = ystart - 0.3*cos(i*delta);
                else
                    x(2,i) = (1-s(i))*ystart + s(i)*yend; % Points in y
                end
                x(3,i) = (1-s(i))*zstart + s(i)*zend; % Points in z
                theta(1,i) = (1-s(i))*(-start_roll)+s(i)*(end_roll);                 % Roll angle
                theta(2,i) = (1-s(i))*(-start_pitch)+s(i)*(end_pitch);            % Pitch angle
                theta(3,i) = (1-s(i))*(-start_yaw)+s(i)*(end_yaw);                 % Yaw angle
                if start_roll == end_roll
                    theta(1,i) = end_roll;
                end
                if start_pitch == end_pitch
                    theta(2,i) = end_pitch;
                end
                if start_yaw == end_yaw
                    theta(3,i) = end_yaw;
                end
                if self.robot.mode == 2
                    theta(1,i) = end_roll;
                    theta(2,i) = end_pitch;
                    theta(3,i) = end_yaw;
                end
                
                
                
                %Use theta(1,i)=(1-s(i)*roll_start+s(i)*roll_end for gradual movements;
                
            end
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            %q0 = zeros(1,7);                                                            % Initial guess for joint angles
            q0 = q_start;
            %qMatrix(1,:) = self.robot.model.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint
            qMatrix(1,:) = q_start;
            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                T = self.robot.model.fkine(qMatrix(i,:));                               % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                %deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = self.robot.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state as respect to the base
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
            end
            qMatrix(:,:)
            for i=1:steps
                while self.robot.stop_status ==1
                    %Do nothing
                    %Problem where it cannot break from loop properly.
                    %Investigate further
                    self.robot.stop_status;
                    pause(0.5);
                end
                
                
                
                %fprintf('Animate %d Number \n,i');
                animate(self.robot.model, qMatrix(i,:));
                %Change Tray Position
                %Condition, is tray being moved? Use a case basis possibly?
                if self.robot.mode ==2
                    %Affect the translation of the door
                    self.door.door.base = transl(0.41,0.64,0.17)*trotx((-pi/4)*(i/steps));
                    animate(self.door.door,0);
                end
                if self.robot.mode ==7
                    self.door.door.base = transl(0.41,0.64,0.17)*trotx((-pi/4)-(-pi/4)*(i/steps));
                    animate(self.door.door,0);
                end
                if self.robot.mode==4
                    A=self.robot.model.fkine(qMatrix(i,:));
                    self.tray_x = A(1,4);
                    self.tray_y = A(2,4);
                    self.tray_z = A(3,4);
                    tray_location = transl(self.tray_x, self.tray_y, self.tray_z);
                    self.tray.tray.base = transl(self.tray_x, (self.tray_y-0.2), self.tray_z);
                    animate(self.tray.tray,0);
                    %Insert tray movement. Might need to be implemented in
                    %Spawn instead
                    %plot3d(self.tray.tray,0,'workspace',self.workspace,'view',[-30,30],'delay',0);
                    %Tray(num2str(0),tray_location, self.workspace);
                    self.CakeLocationSpawn();
                    %set(gca, 'CameraPosition', [-700 3000 1000]);
                end
                if  self.robot.mode ==3
                    A=self.robot.model.fkine(qMatrix(i,:));
                    %Implement Rotation Element
                    self.bottle_location = transl(A(1,4),A(2,4)-0.2,A(3,4));
                    self.bottle_cache(1).bottle.base = self.bottle_location;
                    animate(self.bottle_cache(1).bottle,0);
                    
                end
                drawnow();
            end
            self.previous_RPY = [end_roll, end_pitch, end_yaw];
        end
    end
end