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
    
    methods
        %% GetTrayModel
        function model = GetTrayModel(self, name)
            [faceData,vertexData, plyData] = plyread('tray.ply','tri');
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
        
        function self = Tray(id, location, workspaceDimensions)
            %Create Object
            self.id = id;
            self.workspaceDimensions = workspaceDimensions;
            self.tray = self.GetTrayModel(id)
            self.tray.base = location;
            self.tray.animate(0);
        end
    end
end
