%Angular moment of interita 
function x = omega_J(p, q, r)

    J = evalin('base','J');
    x = [0; 0; 0];
    
    x(1) = (J(3) - J(2))*q*r;
    x(2) = (J(1) - J(3))*r*p;
    x(3) = (J(2) - J(1))*p*q;
    
end
