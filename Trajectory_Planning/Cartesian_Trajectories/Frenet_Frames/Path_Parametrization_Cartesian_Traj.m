% NameFile: Path_Parametrization
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%
%   Given:
%    - path_parametrization_type: The typology of path.
%    - Additional Parameters depending on the Path Typology.
%   Find: 
%   - p, p_prime, p_second, p_prime_norm, p_second_norm. 
%%%%%% END TASK %%%%%%

function [p, p_prime, p_second, p_prime_norm, p_second_norm] = Path_Parametrization_Cartesian_Traj(path_parametrization_type, varargin)
    % Function to compute path parametrization and its derivatives
    
    disp(['Path parametrization type: ', path_parametrization_type]);
    
    % Define the path and its derivatives based on the type
    switch path_parametrization_type
        case 'linear'
            p_i = varargin{1};
            p_f = varargin{2};
            p = @(s) p_i + (p_f - p_i) * (s / norm(p_f - p_i));
            p_prime = @(s) (p_f - p_i) / norm(p_f - p_i);
            p_second = @(s) [0; 0; 0];
        case 'circle'
            R = varargin{1};
            p = @(s) [R * cos(s); R * sin(s); 0];
            p_prime = @(s) [-R * sin(s); R * cos(s); 0];
            p_second = @(s) [-R * cos(s); -R * sin(s); 0];
        case 'ellipse'
            a = varargin{1};
            b = varargin{2};
            phi = varargin{3};
            p = @(s) [a * cos(s + phi); b * sin(s + phi); 0];
            p_prime = @(s) [-a * sin(s + phi); b * cos(s + phi); 0];
            p_second = @(s) [-a * cos(s + phi); -b * sin(s + phi); 0];
        case 'helix_x'
            C = varargin{1};
            r = varargin{2};
            axis = varargin{3};
            h_s = varargin{4};
            p = @(s) C + [h_s * s; r * cos(s); r * sin(s)];
            p_prime = @(s) [h_s; -r * sin(s); r * cos(s)];
            p_second = @(s) [0; -r * cos(s); -r * sin(s)];
        case 'helix_y'
            C = varargin{1};
            r = varargin{2};
            axis = varargin{3};
            h_s = varargin{4};
            p = @(s) C + [r * cos(s); h_s * s; r * sin(s)];
            p_prime = @(s) [-r * sin(s); h_s; r * cos(s)];
            p_second = @(s) [-r * cos(s); 0; -r * sin(s)];
        case 'helix_z'
            C = varargin{1};
            r = varargin{2};
            axis = varargin{3};
            h_s = varargin{4};
            p = @(s) C + [r * cos(s); r * sin(s); h_s * s];
            p_prime = @(s) [-r * sin(s); r * cos(s); h_s];
            p_second = @(s) [-r * cos(s); -r * sin(s); 0];
        otherwise
            error('Unknown path parametrization type');
    end

    disp('Path and derivatives defined.');

    % Define norms of the derivatives
    p_prime_norm = @(s) sqrt(sum(p_prime(s).^2));
    p_second_norm = @(s) sqrt(sum(p_second(s).^2));
    
    disp('Norms of the derivatives calculated.');
end