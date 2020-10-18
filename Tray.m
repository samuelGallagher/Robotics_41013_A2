%Missing
%May need to include a rotation aspect (although the plan is not to rotate
%the tray)

classdef Tray < handle
    %BRICK Summary of this class goes here
    %
    properties (Constant)
        
    end
    properties
        id
        tray
        workspaceDimensions
     
    end
    
    methods(Static)
        %% GetTrayModel
        function model = GetTrayModel(name)
            [faceData,vertexData] = plyread('tray.ply','tri');
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,2) = vertexData(:,2);
            model.points = {vertexData,[]};
        end
        
    end
    methods
        function self = Tray(id, location, workspaceDimensions)
            %Create Object
            self.id = id;
            self.tray = self.GetTrayModel(id)
            self.tray.base = location;
            plot3d(self.tray,0,'workspace',workspaceDimensions,'view',[-30,30],'delay',0);
        end
    end
end
