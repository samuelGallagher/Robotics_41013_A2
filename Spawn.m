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
        tray_x;
        tray_y;
        tray_z;
        Cake_Locations;
        workspace;
        
    end
    
    methods
        function self = Spawn()
            self.tableZ=0.84;
            cakeArray=zeros(1,12);
            stop_status=0;
            
            self.robot = GP7(false);
            self.robot.model.base = transl(-0.25, 0.15, self.tableZ);
            self.CurrentQValues = zeros(1,6);
            self.workspace = [-1 1 -1 1 -0.3 1];
            %Tray location, decided previously
            self.tray_x = -0.8;
            self.tray_y = 0.14;
            self.tray_z = self.tableZ+0.04;
            tray_location = transl(self.tray_x, self.tray_y, self.tray_z);
            
            self.robot.PlotAndColourRobot();%robot,workspace);
            
            Tray(num2str(0), tray_location, self.workspace);
            
            
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
            
            % This is not correct. Need to spawn differently due to
            % movement
            [f,v,data] = plyread('oven_door_low.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            
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
        function CakeLocationSpawn(self)

            %Defining the location for cupcakes
            self.Cake_Locations = zeros(12,3);
            %self.Cake_Locations(1,:) = self.tray_x, self.tray_y, self.tray_z

            self.Cake_Locations= [
                %Middle Row 
                ;self.tray_x,     self.tray_y,    self.tray_z-0.04...            %11   Really 1
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
                    
                    Cake(num2str(i),cake_placement,self.workspace);
                end
                set(gca, 'CameraPosition', [-700 3000 1000]);

            end
            
            
            
        end
        function CakeUpdate(self)
            %Take info updates and place all cake paper in required
            %locations. Use to update position/translation
            
            
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
            q_current = self.robot.returnRobotJoints();
            T = self.robot.model.fkine(q_current);
            xstart = T(1,4);
            ystart = T(2,4);
            zstart = T(3,4);
            
            %xstart = -0.8;
            %ystart = 0.3;
            %zstart = 0.85
            %xend = 0.3;
            %yend = 0.8;
            %zend = 0.85;
            %T_goal = self.model.fkine(q_goal);
            %xend = T_goal(1,4);
            %yend = T_goal(2,4);
            %zend = T_goal(3,4);
            % 1.1) Set parameters for the simulation
            %mdl_puma560;        % Load robot model
            %robotman = GP7;
            t = 5;             % Total time (s)
            deltaT = 0.05;      % Control frequency
            steps = t/deltaT;   % No. of steps for simulation
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
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
                x(1,i) = (1-s(i))*xstart + s(i)*xend; % Points in x
                x(2,i) = (1-s(i))*ystart + s(i)*yend; % Points in y
                x(3,i) = zend; % Points in z
                theta(1,i) = 0;                 % Roll angle
                theta(2,i) = 0;            % Pitch angle
                theta(3,i) = 0;                 % Yaw angle
            end
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            q0 = zeros(1,6);                                                            % Initial guess for joint angles
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
            
            
            for i=1:steps
                while self.robot.stop_status ==1
                    %Do nothing
                    %Problem where it cannot break from loop properly.
                    %Investigate further
                    self.robot.stop_status
                end
                animate(self.robot.model, qMatrix(i,:));
                %Change Tray Position
                %Condition, is tray being moved? Use a case basis possibly?
                if self.robot.mode==2
                    A=self.robot.model.fkine(qMatrix(i,:));
                    tray_x = A(1,4);
                    tray_y = A(2,4);
                    tray_z = A(3,4);
                    tray_location = transl(tray_x, tray_y, tray_z);
                    %Insert tray movement. Might need to be implemented in
                    %Spawn instead
                    Tray(num2str(0),tray_location, self.workspace);                set(gca, 'CameraPosition', [-700 3000 1000]);
                    set(gca, 'CameraPosition', [-700 3000 1000]);
                    self.CakeLocationSpawn();


                end
                drawnow()
            end
        end
    end
end

