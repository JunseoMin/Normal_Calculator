function [surface_normals] = find_circle_surface_normal(U, lam)
    lam1 = abs(lam(1));
    lam2 = abs(lam(2));
    lam3 = abs(lam(3));

    lam_e1_sqrt = sqrt( (lam1 - lam2) / (lam1 + lam3) );
    lam_e3_sqrt = sqrt( (lam2 + lam3) / (lam1 + lam3) );

    surface_normal_1 = U * [lam_e1_sqrt; 0; -lam_e3_sqrt];
    surface_normal_2 = U * [-lam_e1_sqrt; 0; -lam_e3_sqrt];

    surface_normals = [surface_normal_1, surface_normal_2];

end