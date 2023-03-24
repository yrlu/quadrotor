function val = poly_val(poly,t,r)
    val = 0;
    n = length(poly)-1;
    if r<=0
        for i=0:n
            val = val+poly(i+1)*t^i;
        end
    else
        for i=r:n
            a = poly(i+1)*prod(i-r+1:i)*t^(i-r);
            val = val + a;
        end
    end
end