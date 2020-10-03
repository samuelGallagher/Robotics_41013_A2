

%% Load the table downloaded from http://tf3dm.com/3d-model/wooden-table-49763.html vertex colours added with Blender


[f,v,data] = plyread('motop1.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on
[f,v,data] = plyread('motop2.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

hold on
[f,v,data] = plyread('motop4.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

hold on
[f,v,data] = plyread('motop5.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
axis([-2,2,-2,2,-2,2]);
hold on
[f,v,data] = plyread('motop6.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


hold on
[f,v,data] = plyread('motop7.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');



hold on
[f,v,data] = plyread('motop8.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


hold on
[f,v,data] = plyread('motop3.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

hold on
[f,v,data] = plyread('tray.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');



% 
% hold on
% [f,v,data] = plyread('motop3.ply','tri');
% 
% % Get vertex count
% monkeyVertexCount = size(v,1);
% 
% % Move center point to origin
% midPoint = sum(v)/monkeyVertexCount;
% monkeyVerts = v - repmat(midPoint,monkeyVertexCount,1);
% 
% % Create a transform to describe the location (at the origin, since it's centered
% monkeyPose = eye(4);
% 
% % Scale the colours to be 0-to-1 (they are originally 0-to-255
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% 
% % Then plot the trisurf
% monkeyMesh_h = trisurf(f,monkeyVerts(:,1),monkeyVerts(:,2), monkeyVerts(:,3) ...
%     ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
% 
% if doCameraSpin
%     for ax = -40:5:40; %#ok<UNRCH>
%         for by = [[30:-3:0],[0:3:30]];
%             view(ax,by);
%             drawnow();
%             pause(0.01);
%         end;
%     end
% end
% 
% 
% 
% % Move forwards (facing in -y direction)
% forwardTR = makehgtform('translate',[0,-0.01,0]);
% 
% % Random rotate about Z
% randRotateTR = makehgtform('zrotate',(rand-0.5)* 10 * pi/180);
% 
% % Move the pose forward and a slight and random rotation
% monkeyPose = monkeyPose * forwardTR * randRotateTR;
% updatedPoints = [monkeyPose * [monkeyVerts,ones(monkeyVertexCount,1)]']';  
% 
% % Now update the Vertices
% monkeyMesh_h.Vertices = updatedPoints(:,1:3);
% 
% for i = 1:1000
%     % Random rotate about Z
%     randRotateTR = makehgtform('zrotate',(rand-0.5)* 10 * pi/180);
% 
%     % Move forward then random rotation
%     monkeyPose = monkeyPose * forwardTR * randRotateTR;
% 
%     % Transform the vertices
%     updatedPoints = [monkeyPose * [monkeyVerts,ones(monkeyVertexCount,1)]']';
%     
%     % Update the mesh vertices in the patch handle
%     monkeyMesh_h.Vertices = updatedPoints(:,1:3);
%     drawnow();   
% end







keyboard
% clf

% function displayInfo(a)
% 
% [f,v,data] = plyread('a','tri');
% % Scale the colours to be 0-to-1 (they are originally 0-to-255
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% 
% % Then plot the trisurf
% tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
%     ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
% end