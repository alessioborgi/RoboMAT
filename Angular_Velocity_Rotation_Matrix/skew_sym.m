% NameFile: skew_sym
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 25-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% FUNCTION %%%%%%
%   Given:
% - v: A 3-element vector.
%   Find:
%   - The corresponding 3x3 skew-symmetric matrix.
%%%%%% END FUNCTION %%%%%%

function S = skew_sym(v)
    % Ensure the input vector is a column vector
    v = v(:); % Convert v to a column vector

    % Create the 3x3 skew-symmetric matrix
    S = [ 0,    -v(3),  v(2);  % First row: [0, -v3, v2]
          v(3),  0,    -v(1);  % Second row: [v3, 0, -v1]
         -v(2),  v(1),  0 ];   % Third row: [-v2, v1, 0]
end
