%Rotation around Z axis; cosd angle in degrees
function [R] = rotationZ(angle)

R=[cosd(angle), sind(angle), 0 ;
  -sind(angle), cosd(angle), 0 ;
        0     ,     0      , 1];
        
end
