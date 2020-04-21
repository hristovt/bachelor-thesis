%% Transformation matrix function ECEF to EG

function T  = ECEF_to_EG(phi_geocentric,lambda_geocentric)

    T=rotationY(-phi_geocentric)*rotationZ(lambda_geocentric);
    
end