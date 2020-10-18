%Missing
%May need to include a rotation aspect (although the plan is not to rotate
%the tray)

classdef Bottle < handle
    %BRICK Summary of this class goes here
    %
    properties (Constant)
        
    end
    properties
        id
        bottle
        workspaceDimensions
        
    end
    
    methods(Static)
        function model = GetBottleModel(name)
            [faceData,vertexData] = plyread('Squeegie.ply','tri');
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,2) = vertexData(:,2);
            model.points = {vertexData,[]};
        end
        
    end
    methods
        function self = Bottle(id, location, workspaceDimensions)
            %Create Object
            self.id = id;
            self.bottle = self.GetBottleModel(id)
            self.bottle.base = location;
            plot3d(self.bottle,0,'workspace',workspaceDimensions,'view',[-30,30],'delay',0);
        end
    end
end