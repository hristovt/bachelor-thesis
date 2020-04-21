function [R] = rotationZ(angle)
%Rotation around Z axis; cosd angle in degrees

R=[cosd(angle), sind(angle), 0;
  -sind(angle), cosd(angle), 0;
        0     ,     0      , 1];
end

