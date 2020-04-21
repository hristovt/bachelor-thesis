%Angular velocity with quaternions
function X = omega_quaternions(p,q,r)

    X=[0 -p -q -r ;
       p  0  r -q ;
       q -r  0  p ;
       r  q -p  0];

end
