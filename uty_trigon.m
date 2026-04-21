function psi = uty_trigon(a,b,c) % solve a*cos(psi) + b*sin(psi) = c
    if b<0
        psi = asin(-c/sqrt(a^2+b^2))- atan2(-a,-b);
    else
        psi = asin( c/sqrt(a^2+b^2))- atan2( a, b);
    end
end