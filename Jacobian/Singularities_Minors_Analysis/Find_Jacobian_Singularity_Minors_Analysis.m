% NameFile: Find_Jacobian_Singularity
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 27-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - J: The Jacobian Matrix.
%   - variables: A Vector of variables of the jacobian (e.g., [q1, q2, q3,
%   q4, q5]).

%   Find: 
%   - All the Singularities for the Jacobian Matrix using the Minors Analaysis.

%%%%%% END TASK %%%%%%

function [solutions] = Find_Jacobian_Singularity_Minors_Analysis(J, variables)
        
    solutions = [];  % Initialize the solutions variable
    [m, n] = size(J);
    if m ~= n
        if m < n 
            for i = 1:n
                % Create a submatrix of the original non-square matrix by
                % removing one column at a time
                Jcopy=J;
                Jcopy(:,i)=[];
    
                % Determinant of the i-th submatrix
                fprintf('Determinant of the %s -th submatrix: \n', mat2str(i));
                disp(simplify(det(Jcopy)));
    
                % Compute the singularity (equal the determinant to zero)
                fprintf('singularity condition of the %s -th submatrix: \n', mat2str(i));
                singularity_condition = simplify(det(Jcopy)==0);
                disp(singularity_condition);
                solutions = [solutions; singularity_condition];  % Append the condition to solutions
            end
        else 
            %create all submatrix generated from removing any m-n rows

            %case of Jacobian 6*4
            if m==6 & n==4
                %create the sumatrices by creating all the 4by4
                %submatrices
                k=0;
                for i = 1:m
                    % Create a submatrix of the original non-square matrix by
                    % removing one column at a time
                    Jcopy=J;
                    Jcopy(i,:)=[];
                    for j=1:m-1
                        k=k+1;
                        Jcopy2=Jcopy;
                        Jcopy2(j,:)=[];
                        % Determinant of the i-th submatrix
                        fprintf('Determinant of the %s -th submatrix: \n', mat2str(k));
                        disp(simplify(det(Jcopy2)));
            
                        
                    end
                end
            else
                disp('The jabian you have parsed has more rows than columns and its not a geometric jacobian')
            end
        end
    else
        % Compute the determinant
        determinant_Jacobian = simplify(det(J));

        disp('Determinant of the Jacobian:');
        disp(simplify(determinant_Jacobian));

        
        n_var=length(variables);
        % Solve for all possible values of the symbols that make the determinant zero
        if n_var==2
            [q1_sol,q2_sol,params, cond] = solve(determinant_Jacobian == 0, variables, 'ReturnConditions', true);
            solutions=[q1_sol,q2_sol];
        elseif  n_var==3
            [q1_sol,q2_sol,q3_sol,params, cond] = solve(determinant_Jacobian == 0, variables, 'ReturnConditions', true);
            solutions=[q1_sol,q2_sol,q3_sol];
        elseif  n_var==4
            [q1_sol,q2_sol,q3_sol,q4_sol,params, cond] = solve(determinant_Jacobian == 0, variables, 'ReturnConditions', true);
            solutions=[q1_sol,q2_sol,q3_sol,q4_sol];
        end
        disp('All possible solutions for the symbols that make the determinant zero:');
        display(solutions);
        
    end
end