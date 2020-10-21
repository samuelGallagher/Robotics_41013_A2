classdef Cake < handle
    %CAKE Object Class: Can choose between 3 colour models provided,
    %requires input of colour string along with dimension of initial 
    %
    properties (Constant)
        
    end
    properties
        id
        cake
        workspaceDimensions
        colour
    end
    methods
        function self = Cake(id, location, workspaceDimensions, colour)
            %Create Object and spawn at provided location
            self.id = id;
            self.cake.workspaceDimensions = workspaceDimensions;
            self.cake = self.GetCakeModel(id, colour)
            self.cake.base = location;
            self.cake.animate(0);
        end
        function model = GetCakeModel(self, name, colour)
            if nargin < 1
                name = 'Cake';
            end
            colour
            %Cycle between possible colours
            if contains('red', colour)
                [faceData,vertexData, plyData] = plyread('Cake_Red.ply','tri');
            elseif contains('yellow', colour)
                [faceData,vertexData, plyData] = plyread('Cake_Yellow.ply','tri');
            elseif contains('blue', colour)
                [faceData,vertexData, plyData] = plyread('Cake_Blue.ply','tri');
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