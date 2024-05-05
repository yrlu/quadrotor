function c = transRot(A,b)
b_in = [b;1];
c_in = A*b_in;
c = c_in(1:3);
end

