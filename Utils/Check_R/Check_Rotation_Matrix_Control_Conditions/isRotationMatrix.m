% NameFile: isRotationMatrix
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 22-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given a Matrix A, check whether it is a Rotation Matrix:
%   - Square Matrix Check.
%   - Orthogonality Check.
%   - Norm = 1 Check.
%   - Determinant = 1 Check.

%%%%%% END TASK %%%%%%


function isRotMatrix = isRotationMatrix(A)
    % SQUARE MATRIX CHECK: Check if A is square (same number of rows and columns).
    [rows, cols] = size(A);
    is_squared = isequaln(rows, cols);

    % ORTHOGONALITY CHECK: Check if the columns (or rows) are orthogonal to each other.
    A_transpose_A = int64(A' * A);
    is_orthogonal = isequaln(A_transpose_A, int64(eye(rows)));

    % NORM = 1 CHECK: Check if the columns (or rows) are normalized (have a magnitude of 1).
    is_normal = all(int64(vecnorm(A)) == 1);

    % DETERMINANT = +1 CHECK: Check whether the Determinant of A is equal to +1.
    det_A = int64(det(A));
    is_det_plus_one = isequaln(det_A, 1);

    % ROTATION MATRIX OVERALL CHECK:
    is_rotation_matrix = is_squared && is_normal && is_orthogonal && is_det_plus_one;

    if is_rotation_matrix
        disp(" ");
        disp('Square Matrix Check: SATISFIED');
        disp('Normality Check: SATISFIED');
        disp('Orthogonality Check: SATISFIED');
        disp('Determinant=+1 Check: SATISFIED');
        disp(" ");
        disp('The matrix is a Rotation Matrix.');
        isRotMatrix = true;
    else
        disp('The matrix is not a Rotation Matrix.');
        isRotMatrix = false;
        % Identify which conditions are not satisfied
        if ~is_normal
            disp('The matrix is NOT Normal. Indeed: ');
            disp(vecnorm(A))
        end

        if ~is_orthogonal
            disp('The matrix is NOT Orthogonal. Indeed: ');
            disp(int64(A' * A))
        end

        if ~is_det_plus_one
            disp('The Determinant is NOT Equal to +1. Indeed: ');
            disp(det_A)
        end
    end
end
