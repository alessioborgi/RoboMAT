% NameFile: calculateThetaAndR
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 22-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given the Rotation Matrix R, find:
%   - theta: An angle.
%   - r: A unit vector in 3 dimensions ([rx, ry, rz]).
%   Such that: 
%   𝑅 = 𝒓𝒓𝑇 + 𝐼 − 𝒓𝒓𝑇 cos 𝜃 + 𝑆(𝒓) sin 𝜃 = 𝑅(𝜃, 𝒓)

%%%%%% END TASK %%%%%%


function [theta_value_1, r1, theta_value_2, r2] = calculateThetaAndR(R)

    % Calculate the Trace of the Matrix.
    trace_R = trace(R);
    
    % Calculate the angle θ using the Resolution of ATAN2 Function.
    y = sqrt( (R(1,2)-R(2,1))^2  +  (R(1,3)-R(3,1))^2  +  (R(2,3)-R(3,2))^2);
    x = (trace_R - 1);
    disp("The value for sin(θ) is: ");
    disp(y);
    disp("The value for cos(θ) is: ");
    disp(x);

    % Compute the first solution for θ.
    theta_value_1 = atan2d(y, x);
    disp("The value for θ is: ");
    disp(theta_value_1);
    

    % First check whether the theta_value_1 = 0.
    if theta_value_1 == 0.0

        % NO SOLUTION CASE: θ = 0
        disp("SINGULAR CASE: θ = 0. NO SOLUTION");
        disp( " ");

        theta_value_2 = 0;
        r1 = 0;
        r2 = 0;
    
    elseif theta_value_1 == 180 || theta_value_1 == -180
        
        % SINGULARITY CASE: θ = +- pi
        disp("SINGULAR CASE: θ = +- pi");
        disp( " ");

        % Compute also the second Theta value.
        theta_value_2 = atan2d(-y, x);

        % Compute the value for rx,ry nd rz.
        rx = sqrt((R(1,1)+1)/2);
        ry = sqrt((R(2,2)+1)/2);
        rz = sqrt((R(3,3)+1)/2);
        disp("The value for rx is: ");
        disp(rx);
        disp("The value for ry is: ");
        disp(ry);
        disp("The value for rz is: ");
        disp(rz);

        
        % Compute r1 and r2.
        r1 = [rx; ry; rz];
        r2 = -r1;
        disp( "The two approximated signs are: ")       

        %%% SIGN AMBIGUITIES CHECK.
        disp("Remember to check for Sign Ambiguities.")

        %% rxry.
        R12 = R(1,2)/2;
        rxry = rx*ry;
        disp("R12 is: ");
        disp(R12);
        disp("rxry is: ");
        disp(rxry);

        if rxry == R12 && R12 > 0
            disp("x and y must have the Same Positive Sign")
        elseif rxry == R12 && R12 < 0
            disp("x and y must have the Same Negative Sign")
        end
        
        %% rxrz.
        R13 = R(1,3)/2;
        rxrz = rx*rz;
        disp("R13 is: ");
        disp(R13);
        disp("rxrz is: ");
        disp(rxrz);

        if rxrz == R13 && R13 > 0
            disp("x and z must have the Same Positive Sign")
        elseif rxrz == R13 && R13 < 0
            disp("x and z must have the Same Negative Sign")
        end
        
        %% rxrz.
        R23 = R(2,3)/2;    
        ryrz = ry*rz;
        disp("R23 is: ");
        disp(R23);
        disp("ryrz is: ");
        disp(ryrz);

        if ryrz == R23 && R23 > 0
            disp("y and z must have the Same Positive Sign")
        elseif ryrz == R23 && R23 < 0
            disp("y and z must have the Same Negative Sign")
        end
  
    else
        
        % We are in our STANDARD CASE

        % Compute also the second Theta value.
        theta_value_2 = atan2d(-y, x);
    
        disp('STANDARD CASE: The two solutions r1, r2 are: ');
        disp( " ");
    
        r = [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
        denominator_1 = 1/(2*sind(theta_value_1));
        r1 = r*denominator_1;
        
        denominator_2 = 1/(2*sind(theta_value_2));
        r2 = r*denominator_2;


    end

end





