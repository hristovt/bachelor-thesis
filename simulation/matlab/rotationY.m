%Rotation around Y axis; cosd angle in degrees
function [R] = rotationY(angle)

R=[cosd(angle), 0,-sind(angle) ;
       0      , 1,     0       ;
   sind(angle), 0, cosd(angle)];
   
end
