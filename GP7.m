%Current Functionality:
%No Functionality, based on UR5 robot arm without parameters, links, or ply


classdef GP7 < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-2 2 -2 2 -0.3 2];   
        
        %> Flag to indicate if gripper is used
        useGripper = false;
        
        %Position of food given by GUI
        foodPos;
        
        %Number of food pieces placing onto tray
        foodOccurance;
        
        %Bool for door instance. true if currently closed, false if
        %currently open
        doorState;
        
        %Grip State second flag bool
        gripState;
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
% robot = 
self.PlotAndColourRobot();%robot,workspace);
end

%% GetUR5Robot
% Given a name (optional), create and return a UR5 robot model
function GetGP7Robot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['GP_7_',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end
    %Populate with GP7 links and DH parameters
    %Joint limits from motion range using:
    % https://www.motoman.com/en-us/products/robots/industrial/assembly-handling/gp-series/gp7
    
    L1 = Link('d',0.2,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
    L2 = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-65),deg2rad(145)]);
    L3 = Link('d',0,'a',0.475,'alpha',0,'offset',0,'qlim',[deg2rad(-70),deg2rad(190)]);
    L4 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-190),deg2rad(190)]);
    L5 = Link('d',0.35,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-135),deg2rad(135)]);
    L6 = Link('d',0,'a',0.75,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

    self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['motop',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['motop',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
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
    end
end




function PickUpFood(self, foodPosition)
self.foodPos = foodPosition;
%Movement calculations to location given. Need to make foodPos the correct
%data type, 
end


function GripEffector(self, gripState)
%Gripper as is may only be a part of plotting function and is unlikely to
%act within gazebo.
end

function PlaceOnTray(self, foodOccurance)
%Place on tray, have several locations depending on which instance of food
%it is, first, second, third etc
end


function OvenDoorInteract(self, doorState)
%Move from current location to oven door and change state. True if currently closed, false
%if currently open

end

function MoveTray(self)
%Move from current location to tray. Grip tray, move from tray location to
%oven (assuming open), ungrip tray. Use grip function from previous..
end


%function FreeMovement