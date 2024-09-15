function Rx = Rx_symbolic(theta)
    % Rx_symbolic: Generates the symbolic representation of the rotation matrix around the x-axis
    theta = deg2rad(theta);
    Rx = [1, 0, 0; 
          0, cos(theta), -sin(theta); 
          0, sin(theta), cos(theta)];
end


