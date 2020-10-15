classdef URCRAZY < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-2 2 -2 2 0 2];   
        
        %> Flag to indicate if gripper is used
        useGripper = false;        
    end
    
    methods%% Class for UR5 robot simulation
function self = URCRAZY(useGripper)
    if nargin < 1
        useGripper = false;
    end
    self.useGripper = useGripper;
    
%> Define the boundaries of the workspace

        
% robot = 
self.GetURCRAZYRobot();
% robot = 
self.PlotAndColourRobot();%robot,workspace);
end

%% GetUR5Robot
% Given a name (optional), create and return a UR5 robot model
function GetURCRAZYRobot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['GP_7_',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end
      

% 
           L1 = Link('d',0.33,'a',0.04,'alpha',pi/2,'qlim',[deg2rad(-170),deg2rad(170)], 'offset',0); 
           L2 = Link('d',0,'a',0.45,'alpha',0,'qlim', [deg2rad(-65),deg2rad(145)], 'offset',pi/2);
           L3 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[deg2rad(-70),deg2rad(190)],'offset',0);
           L4 = Link('d',-0.4,'a',0,'alpha',pi/2,'qlim',[deg2rad(-190),deg2rad(190)], 'offset',0);
           L5 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[deg2rad(-135),deg2rad(135)], 'offset',pi/2);
           L6 = Link('d',-0.09,'a',-0.013,'alpha',0,'qlim',[deg2rad(-360),deg2rad(360)], 'offset', 0);

    self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR5Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR5Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
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