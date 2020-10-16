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
            
            Tray(num2str(1), tray_location, self.workspace);
            
            
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
    end
end

