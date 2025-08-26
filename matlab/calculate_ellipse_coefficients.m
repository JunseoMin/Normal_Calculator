function [A, B, C, D, E, F] = calculate_ellipse_coefficients(h, k, a, b, th)
    % Convert angle A from degrees to radians
    ra = deg2rad(th);
    A = cos(ra)^2 / a^2 + sin(ra)^2 / b^2;
    B = sin(ra)^2 / a^2 + cos(ra)^2 / b^2;
    C = 2 * cos(ra) * sin(ra) / a^2 - 2 * cos(ra) * sin(ra) / b^2 ;
    D = -2*A*h - C*k;
    E = -2 * B * k - C * h;
    F = A * h^2 + B * k^2 + C*h*k - 1;
end