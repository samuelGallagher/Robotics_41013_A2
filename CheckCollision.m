function[bool] = CheckCollision(bot,sphereCenter,radius)
pause(0.1)
tr = bot.fkine(bot.getpos);
endEffector = sqrt(sum((sphereCenter-tr(1:3,4)').^2));
if endEffector <= radius
    disp('collision')
    bool = 1
    
else
    disp('safe')
    bool = 0
    
end
end