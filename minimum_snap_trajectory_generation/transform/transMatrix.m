function A = transMatrix(v)
A = eye(4);
A(:,4) = [v;1];
end