function [circle_centers] = find_circle_center_position(U, lam, R)
    %1st circle center solution
    lam1 = abs(lam(1));
    lam2 = abs(lam(2));
    lam3 = abs(lam(3));

    lam_e1_sqrt = sqrt( ( lam3 * (lam1 - lam2) ) / ( lam1 * (lam1 - lam3) ) );
    lam_e3_sqrt = sqrt( ( lam1 * (lam2 + lam3) ) / ( lam3 * (lam1 + lam3) ) );

    circle_center_1 = U * [R * lam_e1_sqrt; 0; R * lam_e3_sqrt];
    circle_center_2 = U * [(-1) * R * lam_e1_sqrt; 0; R * lam_e3_sqrt];

    circle_centers = [circle_center_1 circle_center_2];
end