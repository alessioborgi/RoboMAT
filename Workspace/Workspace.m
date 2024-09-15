% NameFile: Workspace
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 18-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - DH: This contains the DH parameters and each row is a link, ordered from first link to last link.
%   - q:  This is a cell input containsing Constraints for all variables.
%   - color: This contains a list of 3 numbers indicating the color.
%            (Ex. [0.8500 0.3250 0.0980] --> Orange)

%   Find: 
%   - Workspace of a n-DOF Planar Arm, revolute or prismatic.

%%%%%% END TASK %%%%%%

% Define the function plotworkspace.
function Workspace(DH,q, color)
    % Plot workspace
    %{
        This function plots a workspace for a planar n-DOF revolute or prismatic
        given DH parameters and the constraints of all variables.

        This function uses Robotics Toolbox by Peter Corke which can be
        downloaded from :
            https://petercorke.com/wordpress/toolboxes/robotics-toolbox
        
        ----------------------------------------------
        INPUTS:
        - DH: This contains the DH parameters and each row is a link, ordered from first link to last link.
        - q:  This is a cell input containsing constraints for all variables.
        - color: This contains a list of 3 numbers indicating the color.
                 (Ex. [0.8500 0.3250 0.0980] --> Orange)
        ---------------------------------------------------------------------
    %}

    % Create a robot object using the DH parameters.
    r = SerialLink(DH);
    r.display()
    [~,n] = size(DH);
    
    % Create symbolic variables for joint angles.
    var = sym('q',[n 1]);
    assume(var,'real')
    
    % Generate a grid of joint angles values.
    [Q{1:numel(q)}] = ndgrid(q{:}); 
    T = simplify(vpa(r.fkine(var),3));
    Pos = T.tv;
    x(var(:)) = Pos(1);
    X = matlabFunction(x);
    X = X(Q{:});
    y(var(:)) = Pos(2);
    Y = matlabFunction(y);
    Y = Y(Q{:});
    
    % Plot the workspace points.
    plot(X(:),Y(:),'MarkerEdge',color, 'LineWidth', 1, 'Marker','o')
    xlabel('X')
    ylabel('Y')
    hold on
    
    end
%%%%%% END PROGRAM %%%%%%


