classdef FruitObject < handle
    %FruitOBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        fruit
        type
        workspace
        faceData = [];
        vertexData = [];
        plyData = [];
    end
    
    methods
        function self = FruitObject(id, type, origin, workspace)
            %FruitOBJECT Construct an instance of this class
            %   Detailed explanation goes here
            self.workspace = workspace;
            self.id = id;
            self.fruit = self.GetFruitModel(id, type);
            self.fruit.base = origin;
            self.fruit.animate(0);
           

        end

        function model = GetFruitModel(self, name, type)
            if nargin < 1
                name = 'Fruit';
            end
            
            % Using brick as placeholder model. Issues with apple model.
            if type == 1
                %load('red_apple.mat');
                [self.faceData,self.vertexData,self.plyData] = plyread('Cake_Full.ply','tri');
            else
                [self.faceData,self.vertexData,self.plyData] = plyread('Cake_Full.ply','tri')
            end 
         
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {self.faceData,[]};
            model.points = {self.vertexData,[]};
            
            plot3d(model,0,'tile1color', [26/255, 102/255, 46/255],'tile2color', [26/255, 102/255, 46/255],'view',[-30,30],'delay',0);
            
            handles = findobj('Tag', model.name);
            h = get(handles,'UserData');
            h.link(1).Children.FaceVertexCData = [self.plyData.vertex.red ...
                                                 ,self.plyData.vertex.green ...
                                                 ,self.plyData.vertex.blue]/255;
            h.link(1).Children.FaceColor = 'interp';
            
       
        end
    end

end

% clf
% [faceData,vertexData] = plyread('fruit.ply','tri');
% L1 = Link('alpha',-pi/2,'a',0,'d',0.3,'offset',0);
% model = SerialLink(L1,'name','test');
% model.faces = {faceData,[]};
% vertexData(:,2) = vertexData(:,2) + 0.4;
% model.points = {vertexData * rotx(-pi/2),[]};
%             
% plot3d(model,0,'workspace',[-2, 2, -2, 2, 0, 2],'view',[-30,30],'delay',0);
% for i = 1:100
% model.base(1:3,1:3) = model.base(1:3,1:3) *  rotz((rand-0.5) * 30 * pi/180);
% animate(model,0); 
% pause(0.2)
% end
            