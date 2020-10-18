%Missing
%May need to include a rotation aspect (although the plan is not to rotate
%the tray)

classdef Door < handle
    %BRICK Summary of this class goes here
    %
    properties (Constant)
        
    end
    properties
        id
        door
        workspaceDimensions
        
    end
    
    methods(Static)
        function model = GetDoorModel(name)
            [faceData,vertexData] = plyread('oven_door_low.ply','tri');
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,2) = vertexData(:,2);
            model.points = {vertexData,[]};
        end
        
    end
    methods
        function self = Door(id, location, workspaceDimensions)
            %Create Object
            self.id = id;
            self.door = self.GetDoorModel(id)
            self.door.base = location;
            plot3d(self.door,0,'workspace',workspaceDimensions,'view',[-30,30],'delay',0);
        end
    end
end