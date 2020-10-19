classdef Spawn < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1;
        robot;
        tableZ;
        %Big problems with cake Array. Not sure how to address yet
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
        workspace;
        cake_cache;
    end
    
    methods
        function self = Spawn()
            self.tableZ=0.84;
            cakeArray=zeros(1,12);
            stop_status=0;
            
            self.robot = GP7(false);
            self.robot.model.base = transl(-0.25, 0.45, self.tableZ-0.2);
            self.CurrentQValues = zeros(1,7);
            self.workspace = [-1 1 -1 1 -0.3 1];
            %Tray location, decided previously
            self.tray_x = -0.7;
            self.tray_y = 0.35;
            self.tray_z = self.tableZ+0.04;
            tray_location = transl(self.tray_x, self.tray_y, self.tray_z);
            door_location = transl(0.41,0.635,0.17)* trotx(-pi/4);
            self.bottle_x = -0.7;
            self.bottle_y = 0;
            self.bottle_z = self.tableZ+0.14;
            bottle_location = transl(self.bottle_x,self.bottle_y,self.bottle_z);
            self.robot.PlotAndColourRobot();%robot,workspace);
            
            self.tray = Tray(num2str(13), tray_location, self.workspace);
            self.door = Door(num2str(14),door_location,self.workspace);
            self.bottle = Bottle(num2str(15),bottle_location,self.workspace);
            %self.tray.tray.base = transl(0-1,1,0.5);
            %animate(self.tray.tray,0);
            %plot3d(self.tray.tray,0,'workspace',self.workspace,'view',[-30,30],'delay',0);
            
            % Insert Simulated Environment
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
            
            self.cake_cache = self.cakeDrop();
            
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
        
        function origin_spawn = cakeDrop(self)
            for i = 1:12
                origin_spawn(i) = Cake(num2str(i), transl(0,0,0), self.workspace);
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
                        %self.SpawnRMRCPoint(cakePlace(1,4,i),cakePlace(2,4,i),cakePlace(3,4,i)+0.2);
                        %pause(2);
                    end
                    
                    animate(self.cake_cache(i).cake,0);
                    %Cake(num2str(i),cake_placement,self.workspace);
                    
                end
                set(gca, 'CameraPosition', [-700 3000 1000]);
                
            end
            
            
            
        end
        function BatterPour(self)
            %RMRC move to cake locations. Cycle through each cake. Moving
            %the bottle is assumed. No rotation, only translation.
            
            
            
            
        end
        
        
        %Nathan Help
        function RobotPoseAnimate(self,q)
            self.CurrentQValues = q;
            self.robot.model.animate(q);
            drawnow();
            while self.stop_status ==1
                %while loop
                fprintf('Currently Stopped');
                
            end
        end
        function stopChange(self, state)
            if state == 1
                self.stop_status = 1;
            else
                self.stop_status = 0;
            end
        end
        
        
        
        function PaperColourChoice(colour);
            if colour == 1
                %thing
                
            elseif colour == 2
                %thing
                
            elseif colour == 3
                %thing
            end
            
            
        end
        
        
        
        function SpawnRMRCPoint(self, xend, yend, zend)
            %function Lab9Solution_Question1(self, xstart, ystart, zstart, xend, yend, zend)
            q_current = self.robot.returnRobotJoints()
            T = self.robot.model.fkine(q_current)
            xstart = T(1,4)
            ystart = T(2,4)
            zstart = T(3,4);
            % http://planning.cs.uiuc.edu/node103.html

            %T_goal = self.model.fkine(q_goal);
            %xend = T_goal(1,4);
            %yend = T_goal(2,4);
            %zend = T_goal(3,4);
            % 1.1) Set parameters for the simulation
            t = 5;             % Total time (s)
            deltaT = 0.025;      % Control frequency
            steps = t/deltaT;   % No. of steps for simulation
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
            
            % 1.2) Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,7);       % Array for joint anglesR
            qdot = zeros(steps,7);          % Array for joint velocities
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
                %Use theta(1,i)=(1-s(i)*roll_start+s(i)*roll_end for gradual movements;
                if self.robot.approach_mode == 0 %General Movement
                    theta(1,i) = 0;                 % Roll angle
                    theta(2,i) = 0;                     % Pitch angle
                    theta(3,i) = 0;                  % Yaw angle
                elseif self.robot.approach_mode ==1 %Approach Oven Door
                    theta(1,i) = 0;                 % Roll angle
                    theta(2,i) = -pi/2;                     % Pitch angle
                    theta(3,i) = pi/2;                  % Yaw angle
                elseif self.robot.approach_mode == 2 % Approach Bottle & Turn Back Bottle
                    theta(1,i) = -pi/2;                 % Roll angle
                    theta(2,i) = 0;                     % Pitch angle
                    theta(3,i) = pi/2;                  % Yaw angle
                elseif self.robot.approach_mode == 3 % Turn Bottle & Squeegies
                    theta(1,i) = -pi/2;                 % Roll angle
                    theta(2,i) = 0;                     % Pitch angle
                    theta(3,i) = 0;                  % Yaw angle
                elseif self.robot.approach_mode == 4 % Approach Tray & Place Tray
                    theta(1,i) = -pi/2;                 % Roll angle
                    theta(2,i) = 0;                     % Pitch angle
                    theta(3,i) = 0;                  % Yaw angle
                    
                end
            end
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            %q0 = zeros(1,7);                                                            % Initial guess for joint angles
            q0 = q_current;
            qMatrix(1,:) = self.robot.model.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint
            
            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                T = self.robot.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
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
                invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:7                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
            end
            
            
            for i=1:steps
                while self.robot.stop_status ==1
                    %Do nothing
                    %Problem where it cannot break from loop properly.
                    %Investigate further
                    self.robot.stop_status
                end
                %fprintf('Animate %d Number \n,i');
                animate(self.robot.model, qMatrix(i,:));
                %Change Tray Position
                %Condition, is tray being moved? Use a case basis possibly?
                if self.robot.mode==4
                    A=self.robot.model.fkine(qMatrix(i,:));
                    self.tray_x = A(1,4);
                    self.tray_y = A(2,4);
                    self.tray_z = A(3,4);
                    tray_location = transl(self.tray_x, self.tray_y, self.tray_z);
                    self.tray.tray.base = transl(self.tray_x, self.tray_y, self.tray_z);
                    animate(self.tray.tray,0);
                    %Insert tray movement. Might need to be implemented in
                    %Spawn instead
                    %plot3d(self.tray.tray,0,'workspace',self.workspace,'view',[-30,30],'delay',0);
                    %Tray(num2str(0),tray_location, self.workspace);
                    self.CakeLocationSpawn();
                    set(gca, 'CameraPosition', [-700 3000 1000]);
                end
                if  self.robot.mode ==3
                    A=self.robot.model.fkine(qMatrix(i,:));
                    %Implement Rotation Element
                    bottle_location = transl(A(1,4),A(2,4),A(3,4));
                    self.bottle.bottle.base = bottle_location;
                    animate(self.bottle.bottle,0);
                    
                end
                drawnow()
            end
        end
    end
end


