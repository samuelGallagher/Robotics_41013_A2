classdef Spawn < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %Container for GP7 class object
        robot;
        %Height set to base starting position for several objects
        tableZ;
        %Empty array used to hold data on Cake Class objects
        cakeArray=zeros(1,12);
        %Container for current Q joint values
        CurrentQValues;
        %Variable linked to E-Stop, start at unused
        stop_status=0;
        %Container for Tray class object
        tray;
        %Container for Door class object
        door;
        %Container for Bottle Class object
        bottle;
        %XYZ variables used for transformation of Tray Object
        tray_x;
        tray_y;
        tray_z;
        %XYZ variables used for transformation of Bottle Object
        bottle_x;
        bottle_y;
        bottle_z;
        %Matrix of locations for Cake Object Translation
        Cake_Locations;
        %Container for bottle location
        bottle_location;
        %Workspace used in all spawning of objects
        workspace;
        %Container for Cake Objects
        cake_cache;
        %Container for Bottle Object
        bottle_cache;
        %Previous Roll, Pitch, Yaw used to guide SpawnRMRCPoint function
        %with relative accuracy
        previous_RPY;
    end
    
    methods
        function self = Spawn()
            %Spawn class initialised. Called by App, build environment and
            %spawn relevant class objects within their starting positions
            
            %Starting Information
            self.tableZ=0.84;
            cakeArray=zeros(1,12);
            stop_status=0;
            self.previous_RPY = zeros(1,3);
            self.robot = GP7(false);
            self.robot.model.base = transl(-0.25, 0.45, 0.58);
            self.CurrentQValues = zeros(1,6);
            self.workspace = [-1 1 -1 1 -0.3 1];
            %Tray Information
            self.tray_x = -0.62;
            self.tray_y = 0.45;
            self.tray_z = self.tableZ+0.04;
            tray_location = transl(self.tray_x, self.tray_y, self.tray_z);
            %Door Information
            door_location = transl(0.41,0.64,0.17);
            %Bottle Information
            self.bottle_x = -0.62;
            self.bottle_y = 0.1;
            self.bottle_z = self.tableZ+0.29;
            self.bottle_location = transl(self.bottle_x,self.bottle_y,self.bottle_z);
            self.robot.PlotAndColourRobot();%robot,workspace);
            
            %Call Object Classes
            self.tray = Tray(num2str(13), tray_location, self.workspace);
            self.door = Door(num2str(14),door_location,self.workspace);
            
            
            %% Simulated Environment
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
            
            
            %Set Camera Position isometric
            set(gca, 'CameraPosition', [-700 3000 1000]);
            
        end
        
        %Place cake as 1 in position that is given as positive
        %When called in GUI, this should result in a set of 0's and 1's
        %determining which cupcakes are used. example of first three is
        %[1 1 1 0 0 0 0 0 0 0 0 0] Values update as per App input
        function CakeSlotChoice(self, id, value)
            for i = 1:12
                if i == id
                    self.cakeArray(1,id) = 1;
                    if value == 0
                        self.cakeArray(1,id) = 0;
                    end
                end
            end
            
        end
        %Create 12 cake objects at origin of specified colour
        function origin_spawn = cakeDrop(self, colour)
            for i = 1:12
                origin_spawn(i) = Cake(num2str(i), transl(0,0,0), self.workspace, colour);
            end
        end
        %Create bottle object of specified colour at origin
        function bottle_spawn = bottleDrop(self, colour)
            for i = 1:1
                bottle_spawn(i) = Bottle(num2str(15), transl(0,0,0), self.workspace, colour);
            end
        end
        %Update location of cakes based on location of Tray Object
        function CakeLocationSpawn(self)
            
            %Defining the location for cupcakes
            self.Cake_Locations = zeros(12,3);
            
            %Locations of cakes align to App control input
            self.Cake_Locations= [
                %Top of Tray
                ;self.tray_x+0.085,     self.tray_y-0.13,    self.tray_z-0.04...   %1
                ;self.tray_x,           self.tray_y-0.13,    self.tray_z-0.04...  %2
                ;self.tray_x-0.085,     self.tray_y-0.13,    self.tray_z-0.04...  %3
                %Middle Upper of Tray
                ;self.tray_x+0.085,     self.tray_y-0.04,    self.tray_z-0.04...  %4
                ;self.tray_x,           self.tray_y-0.04,    self.tray_z-0.04...  %5
                ;self.tray_x-0.085,     self.tray_y-0.04,    self.tray_z-0.04...  %6
                %Middle Down of Tray
                ;self.tray_x+0.085,     self.tray_y+0.04,    self.tray_z-0.04...  %7
                ;self.tray_x,           self.tray_y+0.04,    self.tray_z-0.04...  %8
                ;self.tray_x-0.085,     self.tray_y+0.04,    self.tray_z-0.04...  %9
                %Bottom of Tray
                ;self.tray_x+0.085,     self.tray_y+0.13,    self.tray_z-0.04...  %10
                ;self.tray_x,           self.tray_y+0.13,    self.tray_z-0.04...  %11
                ;self.tray_x-0.085,     self.tray_y+0.13,    self.tray_z-0.04]  %12
            
            %Using only cakes which have been determined for use using
            %cakeArray, cycle through all possible locations, and place the
            %specific cake in specific location
            for i = 1:12
                if 1==self.cakeArray(1,i)
                    cakePlace(:,:,i) = transl(self.Cake_Locations(i,1),self.Cake_Locations(i,2),self.Cake_Locations(i,3));
                    cake_placement = transl(cakePlace(1,4,i),cakePlace(2,4,i),cakePlace(3,4,i));
                    
                    self.cake_cache(i).cake.base = transl(self.Cake_Locations(i,1),self.Cake_Locations(i,2),self.Cake_Locations(i,3));
                    
                    %GP7 makes RMRC to location, simulating population of cake
                    %objects individually
                    if self.robot.mode == 3
                        self.SpawnRMRCPoint(cakePlace(1,4,i),cakePlace(2,4,i)+0.2,cakePlace(3,4,i)+0.3, -pi/2, 0, pi/2, 1.5);
                        pause(1);
                    end
                    %Update positional data through animate
                    animate(self.cake_cache(i).cake,0);
                end
            end
            
            
            
        end
        %Instantaneous application of q values, used in joint manipulation
        %via app
        function RobotPoseAnimate(self,q)
            self.CurrentQValues = q;
            self.robot.model.animate(q);
            drawnow();
        end
        %Stop determined by emergency stop via app.
        function eStop(self)
            counter = 0;
            while self.stop_status == 1
                fprintf('Stopped for Count: %d \n', counter);
                counter = counter+1;
                pause(0.1);
            end
        end
        %Resume determined by emergency stop via app.
        function eStopResume(self)
            self.stop_status = 0;
        end
        
        
        %Spawn cakes and bottle using input via app to determine colour
        function ColourChoice(self, colour);
            if contains('yellow', colour)
                self.bottle_cache = self.bottleDrop('yellow');
                self.cake_cache = self.cakeDrop('yellow');
            end
            if contains('red', colour)
                self.bottle_cache = self.bottleDrop('red');
                self.cake_cache = self.cakeDrop('red');
                fprintf('Colour Choice Red Specifically');
            end
            if contains('blue', colour)
                self.bottle_cache = self.bottleDrop('blue');
                self.cake_cache = self.cakeDrop('blue');
            end
            self.bottle_cache(1).bottle.base = self.bottle_location;
            animate(self.bottle_cache(1).bottle,0);
            fprintf('Colour Choice End');
        end
        
        
        %RMRC calculations for movement between current joint positions and
        %joint positions that when applied result in the input XYZ, roll,
        %pitch, yaw input.
        %Time input is also considered
        function SpawnRMRCPoint(self, xend, yend, zend, end_roll, end_pitch, end_yaw, time)
            %RMRC function based on Lab 9 Question 1 solutions for Robotics
            
            
            %Populate starting positional data
            q_start = self.robot.model.getpos();
            Transform = self.robot.model.fkine(q_start);
            xstart = Transform(1,4);
            ystart = Transform(2,4);
            zstart = Transform(3,4);
            start_roll = self.previous_RPY(1,1);
            start_pitch = self.previous_RPY(1,2);
            start_yaw = self.previous_RPY(1,3);
            
            %Step manipulation information
            time;             % Total time (s)
            deltaT = 0.05;      % Control frequency
            steps = time/deltaT;   % No. of steps for simulation
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
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar, from 0 to 1
            for i=1:steps
                %Trajectory is determined by starting and ending position
                %within XYZ and RPY.
                %robot mode is used to determine if trajectory should be
                %a direct or curved approach, and which direction to curve
                %in
                
                x(1,i) = (1-s(i))*xstart + s(i)*xend; % Points in x
                if self.robot.mode ==4
                    %x(2,i) = ystart - 0.3*cos(i*delta);
                    x(2,i) = (1-s(i))*ystart + s(i)*yend+0.4*sin(pi*s(i)); % Points in y
                else
                    x(2,i) = (1-s(i))*ystart + s(i)*yend; % Points in y
                end
                if self.robot.mode ==1
                    x(3,i) = (1-s(i))*zstart + s(i)*zend+0.2*sin(pi*s(i)); % Points in z
                elseif self.robot.mode ==2
                    x(3,i) = (1-s(i))*zstart + s(i)*zend+0.02*sin(pi*s(i)); % Points in z
                elseif self.robot.mode ==4
                    x(3,i) = (1-s(i))*zstart + s(i)*zend+0.2*sin(pi*s(i)); % Points in z
                else
                    x(3,i) = (1-s(i))*zstart + s(i)*zend; % Points in z
                end
                %Angles are assumed to have direct movement between start
                %and end unless otherwise specified
                theta(1,i) = (1-s(i))*(-start_roll)+s(i)*(end_roll);                 % Roll angle
                theta(2,i) = (1-s(i))*(-start_pitch)+s(i)*(end_pitch);            % Pitch angle
                theta(3,i) = (1-s(i))*(-start_yaw)+s(i)*(end_yaw);                 % Yaw angle
                %If starting and ending rotation are equal, no
                %transformation is applied in order to simplify movement
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
                
                
            end
            %Calculations using above points, determining Q values that
            %result in the exact position and rotation at each instance
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            %q0 = zeros(1,7);                                                            % Initial guess for joint angles
            q0 = q_start;
            %qMatrix(1,:) = self.robot.model.ikcon(T,q0);                                % Solve joint angles to achieve first waypoint
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
                qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation
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
                %If Emergency E-Stop is activated within app, this function
                %stalls within for loop until release. Upon release will
                %continue with current aim trajectory
                self.eStop();
                %step animation of GP7
                animate(self.robot.model, qMatrix(i,:));
                
                
                if self.robot.mode ==2 % Moving Door Open Mode
                    %Affect the translation of the door
                    self.door.door.base = transl(0.41,0.64,0.17)*trotx((-pi/4)*(i/steps));
                    animate(self.door.door,0);
                end
                if self.robot.mode ==7% Moving Door Closed Mode
                    %Affect the translation of the door
                    self.door.door.base = transl(0.41,0.64,0.17)*trotx((-pi/4)-(-pi/4)*(i/steps));
                    animate(self.door.door,0);
                end
                if self.robot.mode==4 % Tray Movement Mode, includes movement of Cakes
                    A=self.robot.model.fkine(qMatrix(i,:));
                    self.tray_x = A(1,4);
                    self.tray_y = A(2,4);
                    modified_y = (self.tray_y-0.3); % Slight change in location to accomodate for gripper approach
                    self.tray_z = A(3,4);
                    tray_location = transl(self.tray_x, modified_y, self.tray_z);
                    self.tray.tray.base = transl(self.tray_x, modified_y, self.tray_z);
                    animate(self.tray.tray,0);
                    self.tray_y = modified_y;
                    
                    %Cake Movement Linked, activates at same time as tray
                    %thus updating to tray positional data
                    self.CakeLocationSpawn();
                end
                if  self.robot.mode ==3 % Moving Bottle Mode
                    A=self.robot.model.fkine(qMatrix(i,:));
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