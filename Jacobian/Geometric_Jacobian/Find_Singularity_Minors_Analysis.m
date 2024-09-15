% NameFile: Find_Singularity_Minors_Analysis
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - J: The (Geometric) Jacobian.
%   - variables: A Vector of variables of the jacobians.
% 
%   Find: 
%   - The solutions for the Minors of it, helping in individuating the 
%     Singularities.

%%%%%% END TASK %%%%%%

function [solutions] = Find_Singularity_Minors_Analysis(J, variables)
    %Function that computes the singularity of a jacobian matrix square or
    %not

    % Initialize solutions as an empty array
    solutions = [];

    % Compute the Jacobian Size.
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
                    new_condition = simplify(det(Jcopy));
                    disp(new_condition);
                    if new_condition ~= 0
                        solutions = [solutions, new_condition==0];
                    end
        
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
                            new_condition = simplify(det(Jcopy2));
                            disp(new_condition);
                            if new_condition ~= 0
                                solutions = [solutions, new_condition==0];
                            end
               
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
    
    
            
            n_var=length(variables)
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