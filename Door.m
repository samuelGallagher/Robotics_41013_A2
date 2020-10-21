
classdef Door < handle
    % Door object class. Spawn single model at desired location
    properties (Constant)
        
    end
    properties
        id
        door
        workspaceDimensions
     
    end
    
    methods
        %% GetDoorModel
        function model = GetDoorModel(self, name)
            [faceData,vertexData, plyData] = plyread('oven_door_low.ply','tri');
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,2) = vertexData(:,2);
            model.points = {vertexData,[]};
            %Enable true colour to be used
            plot3d(model,0,'workspace',self.workspaceDimensions,'delay',0);
            handles = findobj('Tag', model.name);
            h = get(handles,'UserData');
            h.link(1).Children.FaceVertexCData = [plyData.vertex.red ...
                                                 ,plyData.vertex.green ...
                                                 ,plyData.vertex.blue]/255;
            h.link(1).Children.FaceColor = 'interp';    
        end
        
        function self = Door(id, location, workspaceDimensions)
            %Create Object and spawn at provided location
            self.id = id;
            self.workspaceDimensions = workspaceDimensions;
            self.door = self.GetDoorModel(id)
            self.door.base = location;
            self.door.animate(0);
        end
    end
end
