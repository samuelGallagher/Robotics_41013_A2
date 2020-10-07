classdef Cake < handle
    %CAKE Summary of this class goes here
    %
    properties (Constant)
        
    end
    properties
        id
        cake
        workspaceDimensions
    end
    
    methods(Static)
        %% GetCakeModel
        function model = GetCakeModel(name)
            if nargin < 1
                name = 'Cake';
            end
            
            %if name == 1
                [faceData,vertexData] = plyread('Box_1.ply','tri');
            %end
            if name == 2
                [faceData,vertexData] = plyread('Box_2.ply','tri');
            end
            if name == 3
                [faceData,vertexData] = plyread('Box_3.ply','tri');
            end
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,2) = vertexData(:,2);
            model.points = {vertexData,[]};
        end
    end
    methods
        function self = Cake(id, location, workspaceDimensions)
            %Create Object
            self.id = id;
            self.cake = self.GetCakeModel(id)
            self.cake.base = location;
            plot3d(self.cake,0,'workspace',workspaceDimensions,'view',[-30,30],'delay',0);
        end
    end
end