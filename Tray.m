classdef Tray < handle
    % TRAY object class. Spawn single model at desired location
    %NOTE: Tray model is spawned at centre of tray instead of sides
    properties (Constant)
        
    end
    properties
        id
        tray
        workspaceDimensions
     
    end
    
    methods
        %% GetTrayModel
        function model = GetTrayModel(self, name)
            [faceData,vertexData, plyData] = plyread('tray.ply','tri');
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
        
        function self = Tray(id, location, workspaceDimensions)
            %Create Object and spawn at provided location
            self.id = id;
            self.workspaceDimensions = workspaceDimensions;
            self.tray = self.GetTrayModel(id)
            self.tray.base = location;
            self.tray.animate(0);
        end
    end
end
