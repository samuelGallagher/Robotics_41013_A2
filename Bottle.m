classdef Bottle < handle
    %BOTTLE Object Class: Can choose between 3 colour models provided,
    %requires input of colour string along with dimension of initial 
    %
    properties (Constant)
        
    end
    properties
        id
        bottle
        workspaceDimensions
        colour
    end
    methods
        function self = Bottle(id, location, workspaceDimensions, colour)
            %Create Object and spawn at provided location
            self.id = id;
            self.bottle.workspaceDimensions = workspaceDimensions;
            self.bottle = self.GetBottleModel(id, colour)
            self.bottle.base = location;
            self.bottle.animate(0);
        end
        function model = GetBottleModel(self, name, colour)
            if nargin < 1
                name = 'Bottle';
            end
            %Cycle between possible colours
            if contains('red', colour)
                [faceData,vertexData, plyData] = plyread('Squeegie_Red.ply','tri');
            elseif contains('yellow', colour)
                [faceData,vertexData, plyData] = plyread('Squeegie_Yellow.ply','tri');
            elseif contains('blue', colour)
                [faceData,vertexData, plyData] = plyread('Squeegie_Blue.ply','tri');
            end
            
            %Link, model, vertex, face, ply data all required for colour
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,2) = vertexData(:,2);
            model.points = {vertexData,[]};
            %Assistance
            plot3d(model,0,'workspace',self.workspaceDimensions,'delay',0);
            handles = findobj('Tag', model.name);
            h = get(handles,'UserData');
            h.link(1).Children.FaceVertexCData = [plyData.vertex.red ...
                                                 ,plyData.vertex.green ...
                                                 ,plyData.vertex.blue]/255;
            h.link(1).Children.FaceColor = 'interp';
            
        end
    end
end