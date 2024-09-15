function range = Find_Range_Numerical(J)
    % Function to compute the range of the Jacobian matrix using singular value decomposition.
    %
    % Parameters:
    %   - J: The Jacobian matrix.
    %
    % Returns:
    %   - range: The range of the Jacobian matrix.

    % Perform singular value decomposition (SVD) of the Jacobian matrix
    [~, S, ~] = svd(J);

    % Compute the singular values
    singular_values = diag(S);

    % Compute the rank of the Jacobian (number of non-zero singular values)
    rank_J = nnz(singular_values);

    % Compute the range of the Jacobian
    range = rank_J;
end
