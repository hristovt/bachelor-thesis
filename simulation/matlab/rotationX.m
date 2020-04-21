%Rotation around X axis; cosd angle in degrees
function [R] = rotationX(angle)

R=[1,     0      ,     0       ;
   0, cosd(angle), sind(angle) ;
   0,-sind(angle), cosd(angle)];
   
end

