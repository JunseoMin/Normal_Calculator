function [U, lam] = find_transform_cone_cam(V, D)
    mu1 = D(1,1);
    mu2 = D(2,2);
    mu3 = D(3,3);
    f1 = V(:, 1);
    f2 = V(:, 2);
    f3 = V(:, 3);
    z_con = [0;0;1];
    % determine lam3 with differece of sign of lambda (mud)
    if sign(mu1) == sign(mu2)
        lam3 = mu3;

        if dot(f3, z_con) > 0
            e3 = f3;
        else
            e3 = -f3;
        end

        if abs(mu1) < abs(mu2)
            lam2 = mu1;
            e2 = f1;
            lam1 = mu2;
            e1 = cross(e2, e3);
        elseif abs(mu1) > abs(mu2)
            lam2 = mu2;
            e2 = f2;
            lam1 = mu1;
            e1 = cross(e2, e3);
        elseif abs(mu1) == abs(mu2)
            lam2 = mu2;
            e2 = f2;
            lam1 = mu1;
            e1 = cross(e2, e3);
        end

    elseif sign(mu1) == sign(mu3)
        lam3 = mu2;
        if dot(f2, z_con) > 0
            e3 = f2;
        else
            e3 = -f2;
        end

        if abs(mu1) < abs(mu3)
            lam2 = mu1;
            e2 = f1;
            lam1 = mu3;
            e1 = cross(e2, e3);
        elseif abs(mu1) > abs(mu3)
            lam2 = mu3;
            e2 = f3;
            lam1 = mu1;
            e1 = cross(e2, e3);
        elseif abs(mu1) == abs(mu3)
            lam2 = mu3;
            e2 = f3;
            lam1 = mu1;
            e1 = cross(e2, e3);
        end

    elseif sign(mu2) == sign(mu3)
        lam3 = mu1;
        if dot(f1, z_con) > 0
            e3 = f1;
        else
            e3 = -f1;
        end

        if abs(mu2) < abs(mu3)
            lam2 = mu2;
            e2 = f2;
            lam1 = mu3;
            e1 = cross(e2, e3);
        elseif abs(mu2) > abs(mu3)
            lam2 = mu3;
            e2 = f3;
            lam1 = mu2;
            e1 = cross(e2, e3);
        elseif abs(mu2) == abs(mu3)
            lam2 = mu3;
            e2 = f3;
            lam1 = mu2;
            e1 = cross(e2, e3);
        end
    end

    % %Transform matrix of cone frame w.r.t camera frame P
    % e1 = [cos(theta); sin(theta); 0];
    % e2 = [-sin(theta); cos(theta); 0];
    % e3 = cross(e1, e2);

    U = [e1 e2 e3];
    % kx ky kz is ellipse semi-major, semi-minor point in image plane w.r.t
    % cone frame origin
    kx = 1/sqrt(abs(lam1));
    ky = 1/sqrt(abs(lam2));
    kz = 1/sqrt(abs(lam3));
    k = [kx, ky, kz];
    lam = [lam1, lam2, lam3];
    % lam = [1, 0.5, 0.1];  %%MJ
end