%% Function Computes Symbolic Principle Minors

function principalMinors = computePrincipalMinors(matrix)
n = size(matrix, 1);

% Initialize Container for Principal Minors
principalMinors = sym('principalMinors', [1, n]);

for i = 1:n
    submatrix = matrix(1:i, 1:i);
    principalMinors(i) = det(submatrix);
end
end