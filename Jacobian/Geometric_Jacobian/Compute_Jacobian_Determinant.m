% NameFile: Compute_geometric_jacobian_Determinant
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - geometric_jacobian: The Geometric Jacobian.
% 
%   Find: 
%   - Its Determinant taking into account the size of J and handling all
%   the cases.

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%

function determinant = Compute_Jacobian_Determinant(geometric_jacobian)

    % Check the size of the geometric_jacobian matrix
    [m, n] = size(geometric_jacobian);
    
    % If the matrix is square
    if m == n
        disp('The matrix is square. Computing the determinant using det(J).');
        % Compute the determinant using det(J)
        determinant = det(geometric_jacobian);
    % If the matrix has more rows than columns
    elseif m > n
        disp('The matrix has more rows than columns. Computing the determinant using det(J^T * J).');
        % Compute the determinant using det(J^T * J)
        determinant = det(geometric_jacobian' * geometric_jacobian);
    % If the matrix has more columns than rows
    else
        disp('The matrix has more columns than rows. Computing the determinant using det(J * J^T).');
        % Compute the determinant using det(J * J^T)
        determinant = det(geometric_jacobian * geometric_jacobian');
    end
end