% NameFile: Path_Parametrization_Line
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%    - p_i: Initial position.
%    - p_f: Final_Position.

%   Find: 
%   - p, p_prime, p_second, p_prime_norm, p_second_norm.

%%%%%% END TASK %%%%%%


function [p, p_prime, p_second, p_prime_norm, p_second_norm] = Path_Parametrization_Line(p_i, p_f)

    syms s real
    
    p = p_i+(p_f-p_i)*s;
    
    % Display p(s).
    disp("p(s) is: ");
    disp(p);

    % Calculate the first derivative of p(s).
    p_prime = diff(p);
    disp("p'(s) is: ");
    disp(simplify(p_prime));

    % Calculate the second derivative of p(s).
    p_second = diff(p_prime);
    disp("p''(s) is: ");
    disp(simplify(p_second));

    % Calculate the norm of the first derivative ||p'(s)||.
    p_prime_norm = norm(p_prime);
    disp("||p'(s)|| is: ");
    disp(simplify(p_prime_norm));

    % Calculate the norm of the second derivative ||p''(s)||.
    p_second_norm = norm(p_second);
    disp("||p''(s)|| is: ");
    disp(simplify(p_second_norm));
    
    % Display ṗ(s) = p'(s) * ṡ.
    disp(" ṗ(s)= p'(s)* ṡ");
    % Convert symbolic vector to an array of strings
    p_prime_str = strings(size(p_prime));
    for i = 1:numel(p_prime)
        p_prime_str(i) = char(p_prime(i));
    end

    % Create a column vector of "ṡ" string
    s_str = "ṡ";

    % Display the concatenated text and symbolic vector
    a = strcat(strcat ('[', strjoin(p_prime_str, '; '))    , ']^⊤  *  ', s_str);
    disp(a);
    
    % Display p̈(s) = p'(s) * s̈ + p''(s) * ṡ².
    disp(" ");
    disp(" p̈(s)= p'(s)* s̈ + p''(s)*ṡ² ");
    % Convert symbolic vector to an array of strings
    p_second_str = strings(size(p_second));
    for i = 1:numel(p_second)
        p_second_str(i) = char(p_second(i));
    end

    % Display the concatenated text and symbolic vector
    a = strcat ('[', strjoin(p_prime_str, '; '), ']^⊤  * s̈', '  +  [', strjoin(p_second_str, '; '), ']^⊤  *  ṡ²');
    disp(a);

end
