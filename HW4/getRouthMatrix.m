%% Function Generates Symbolic Routh-Hurwitz Matrix

function routh_matrix = getRouthMatrix(char_eq, var)
% Get coefficients in descending order
coeff = coeffs(char_eq, var);
n = length(coeff) - 1;

% Initialize Routh array
routh_matrix = sym(zeros(n, n));

% Isolate Even-Indexed and Odd-Indexed Coefficients
even_coeffs = coeff(1:2:end);
odd_coeffs = coeff(2:2:end);

for i = 1:n
    
    if mod(i, 2) == 1
        routh_matrix(i, fix(i/2) + 1: fix(i/2) + 1 + length(odd_coeffs) - 1) = odd_coeffs(end:-1:1);
    else
        routh_matrix(i, fix(i/2): fix(i/2) + length(even_coeffs) - 1) = even_coeffs(end:-1:1);
    end
    
end

end