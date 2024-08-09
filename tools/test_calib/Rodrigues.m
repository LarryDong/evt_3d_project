function R = Rodrigues(om)
    % This function converts a rotation vector (Rodrigues vector) into a rotation matrix.
    theta = norm(om);
    if theta < eps
        R = eye(3);
        return;
    end

    k = om / theta;
    K = [0, -k(3), k(2); 
         k(3), 0, -k(1); 
         -k(2), k(1), 0];

    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K * K);
end