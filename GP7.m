
classdef GP7 < handle
    % GP7 object class. MOTOMAN GP7 robot simulated using data from
    % manufacturers. 
    % Class enables functionality to animate GP7 between joint matrix's and
    % end effector locations
    % RMRC used for gradual and realistic movement using XYZ or q input. 
    %Free movement enabled using a simulated joystick.
    %VirtualTeachPendant.mlapp used
    properties
        %> Robot model
        model;
        %Status used within E-Stop
        stop_status;
        
        %Workspace
        workspace = [-3 2 -2 2 -0.3 2];
        
        %> Flag to indicate if gripper is used 
        %[Not utilised within this assignment]
        useGripper = false;
        %Robot mode, variable used to cycle between possible movement
        %requirements
        mode;
        %variable used to cycle between several set end effector rotations
        approach_mode;
    end
    
    methods%% Class for UR5 robot simulation
        function self = GP7(useGripper)
            if nargin < 1
                useGripper = false;
            end
            self.useGripper = useGripper;
            
            %> Define the boundaries of the workspace
            
            
            % robot =
            self.GetGP7Robot();
            stop_status = 0;
            % robot =
            self.PlotAndColourRobot();%robot,workspace);
            self.mode=0;
            
        end
        
        %% GetGP7Robot
        % Based on LinearUR5() object class by Peter Corke
        function GetGP7Robot(self)
            pause(0.001);
            name = ['GP_7_',datestr(now,'yyyymmddTHHMMSSFFF')];
            %     end
            %Populate with GP7 links and DH parameters
            %Joint limits from motion range using:
            % https://www.motoman.com/en-us/products/robots/industrial/assembly-handling/gp-series/gp7
            toggle = 0;
            %Joint information and DH parameters
            if toggle == 0
           L1 = Link('d',0.33,'a',0.04,'alpha',pi/2,'qlim',[deg2rad(-170),deg2rad(170)], 'offset',0); 
           L2 = Link('d',0,'a',0.45,'alpha',0,'qlim', [deg2rad(-65),deg2rad(145)], 'offset',pi/2);
           L3 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[deg2rad(-70),deg2rad(190)],'offset',0);
           L4 = Link('d',-0.4,'a',0,'alpha',pi/2,'qlim',[deg2rad(-190),deg2rad(190)], 'offset',0);
           L5 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[deg2rad(-135),deg2rad(135)], 'offset',pi/2);
           L6 = Link('d',-0.09,'a',-0.013,'alpha',0,'qlim',[deg2rad(-360),deg2rad(360)], 'offset', 0);
           %L7 = Link('d',0,'a',0,'alpha',0,'qlim',[deg2rad(-360),deg2rad(360)], 'offset', 0);
           %Testing DH parameters of UR5 used in comparative stages
            else
                L1 = Link('d',0.0892,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
                L2 = Link('d',0.1357,'a',0.425,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
                L3 = Link('d',0.1197,'a',0.39243,'alpha',pi,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
                L4 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
                L5 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
                L6 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
                %L7 = Link('d',0,'a',0,'alpha',0,'qlim',[deg2rad(-360),deg2rad(360)], 'offset', 0);
                
            end
            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6 ],'name',name);
        end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        %Taken from UR5 Function by Peter Corke
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                if self.useGripper && linkIndex == self.model.n
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['GP7Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
                else % Should use motop instead
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['GP7Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                end
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;
            
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        %Unused function
        function SubsequentLocation(self, q)
            T = self.model.fkine(q);
            
            
        end
        %RMRC using q_goal
        %Not utilised within demo
        function RMRC(self, q_goal)
            %function taken from Lab 9 Question 1 solutions for Robotics
            q_current = self.returnRobotJoints();
            T = self.model.fkine(q_current)
            xstart = T(1,4);
            ystart = T(2,4);
            zstart = T(3,4);
            rotm = tform2rotm(T);
            eulZYX = rotm2eul(rotm);
            start_roll = eulZYX(1,1);
            start_pitch = eulZYX(1,2);
            start_yaw = eulZYX(1,3);

            T_goal = self.model.fkine(q_goal);
            xend = T_goal(1,4);
            yend = T_goal(2,4);
            zend = T_goal(3,4);
            rotm_goal = tform2rotm(T_goal);
            eulZYX_goal = rotm2eul(rotm);
            end_roll = eulZYX_goal(1,1);
            end_pitch = eulZYX_goal(1,2);
            end_yaw = eulZYX_goal(1,3);
            
            % 1.1) Set parameters for the simulation
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
                x(3,i) = zstart; % Points in z
                theta(1,i) = (1-s(i))*(-start_roll)+s(i)*(end_roll);                 % Roll angle
                theta(2,i) = (1-s(i))*(-start_pitch)+s(i)*(end_pitch);            % Pitch angle
                theta(3,i) = (1-s(i))*(-start_yaw)+s(i)*(end_yaw);                 % Yaw angle
            end
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            %q0 = zeros(1,7);                                                            % Initial guess for joint angles
            q0=q_current;
            qMatrix(1,:) = self.model.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint
            
            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                T = self.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                %deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = self.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state as respect to the base
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
            end
            
            
            for i=1:steps
                while self.stop_status ==1
                    %Do nothing
                    %Problem where it cannot break from loop properly.
                    %Investigate further
                    self.stop_status = 0;
                    pause(0.1);
                end
                animate(self.model, qMatrix(i,:));
                drawnow()
            end
        end
        
        
        
        
        
        %RMRC using XYZ end locations for end effector
        %Not utilised within demo        
        function RMRCPoint(self, xend, yend, zend)
            %function taken from Lab 9 Question 1 solutions for Robotics
            q_current = self.returnRobotJoints();
            T = self.model.fkine(q_current);
            xstart = T(1,4);
            ystart = T(2,4);
            zstart = T(3,4);

            % 1.1) Set parameters for the simulation
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
            qMatrix(1,:) = self.model.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint
            
            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                T = self.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                %deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = self.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state as respect to the base
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
            end
            
            
            for i=1:steps
                while self.stop_status ==1
                    %Do nothing
                    %Problem where it cannot break from loop properly.
                    %Investigate further
                    self.stop_status
                end
                animate(self.model, qMatrix(i,:));
                %Change Tray Position
                %Condition, is tray being moved? Use a case basis possibly?
                if self.mode==2
                    A=self.model.fkine(qMatrix(i,:));
                    tray_x = A(1,4);
                    tray_y = A(2,4);
                    tray_z = A(3,4);
                    tray_location = transl(tray_x, tray_y, tray_z);
                    %Insert tray movement. Might need to be implemented in
                    %Spawn instead
                    %Tray(num2Str(1),tray_location, workspace);
                end
                drawnow()
            end
        end
        
        
        
        %Used within APP to return current joint positions of GP7
        function q = returnRobotJoints(self)
            q = self.model.getpos();
            disp(q);
        end

        %Free Movement Function for virtual Joystick
        function FreeMovement(self)
            %Slightly adapted from Lab 11 - Question 2 skeleton code
            
            id = 1; % Note: may need to be changed if multiple joysticks present
            pendant = VirtualTeachPendant;
            joy = vrjoystick(id);
            axes = pendant.read;
            %joy = vrjoystick(id);
            caps(joy) % display joystick information
            
            % Start "real-time" simulation
            q = self.returnRobotJoints();% Set initial robot configuration 'q'

            
            %HF = figure(1);         % Initialise figure to display robot
            %robot.PlotAndColourRobot();
            %robot.plot(q);          % Plot robot in initial configuration
            %robot.delay = 0.001;    % Set smaller delay when animating
            %set(HF,'Position',[0.1 0.1 0.8 0.8]);
            
            duration = 45;  % Set duration of the simulation (seconds)
            dt = 0.15;      % Set time step for simulation (seconds)
            
            n = 0;  % Initialise step count to zero
            tic;    % recording simulation start time
            while( toc < duration)
                
                n=n+1; % increment step count
                
                % read joystick
                %[axes, buttons, povs] = read(joy);
                axes = pendant.read;
                
                % -------------------------------------------------------------
                % YOUR CODE GOES HERE
                % 1 - turn joystick input into an end-effector velocity command
                % 2 - use J inverse to calculate joint velocity
                % 3 - apply joint velocity to step robot joint angles
                % -------------------------------------------------------------
                Kv = 0.2; %linear velocity gfain
                Kw = 1.0; %angular velocity gain
                
                %Can plot to any direction/element so is fine
                vx = Kv*axes(1);
                vy = Kv*axes(2);
                vz = Kv*axes(3);
                % could use vz = buttons(5)-buttons(7) to use two buttons for forward
                % and backward
                wx = Kw*axes(4);
                wy = Kw*axes(5);
                wz = Kw*axes(6);
                
                
                dx = [vx;vy;vz;wx;wy;wz];
                
                
                
                %can clamp and change them to act a certain way. for examle
                %This forces it to be above this threshold. making it not move when
                %not much pressure is applied
                dx((dx.^2)<0.01)=0;
                
                
                %2 - Use J inverse to calculate joint velocity
                
                J = self.model.jacob0(q);%q is our 6x6 matrix for jacobian.
                
                lambda = 0.1;
                Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
                
                %dx = J*dq We want opposite of this
                %dq = inv(J)*dx
                dq = Jinv_dls*dx;
                
                
                
                
                %3 - Apply joint velocity to step robot joint angles
                
                %This makes a step in the direction of the desired velocity
                q = q+(dq*dt)';
                
                %Damped least square sacrifices accuracy to damp it (adding error) in
                %order to not have large velocity vectors at joint limits.
                
                
                % Update plot
                %robot.animate(q);
                animate(self.model,q);
                T = self.model.fkine(q);
                rotm = tform2rotm(T);
                eulZYX = rotm2eul(rotm)
                drawnow()
                
                % wait until loop time elapsed
                if (toc > dt*n)
                    warning('Loop %i took too much time - consider increating dt',n);
                end
                while (toc < dt*n); % wait until loop time (dt) has elapsed
                end
            end
            
            
            
            
        end
        
    end
end