%perspective one circle main function
function [PM_normal_cam, PM_center_cam, Window_normal_cam, Window_center_cam, C_pm, C_win, angle_diff] = perspective_two_circle(ellipses_params, R, intrinsics_K)
    C_quad = cell(1, 2);
    circle_centers = cell(1, 2);
    surface_normals = cell(1, 2);
    for i = 1:length(R)
        k = ellipses_params(i, 1); %y
        h = ellipses_params(i, 2); %x
        a = ellipses_params(i, 3)/2; %a
        b = ellipses_params(i, 4)/2; %b
        angle = 90-ellipses_params(i, 5); %angle (deg)
        
        % ellipse equation to quadratic form 
        [A, B, C, D, E, F] = calculate_ellipse_coefficients(h, k, a, b, angle);
        C_quad{i} = [A, C/2, D/2; C/2, B, E/2; D/2, E/2, F]; 
    
    %% Cone Equation
        Q = intrinsics_K'* C_quad{i} * intrinsics_K;
        
        % eigen value and normalized eigen vector of Q
        [V, D] = eig(Q); % eigen value D, eigen vector V
        % Normalize the eigenvectors
        for j = 1:size(V, 2)
            V(:, j) = V(:, j) / norm(V(:, j));
        end
        
        [U, lam] = find_transform_cone_cam(V, D);
        
        % circle 3d position
        % circle center has two solution
        circle_centers{i} = find_circle_center_position(U, lam, R(i));
        
        % surface normal has two solution corresponding 
        surface_normals{i} = find_circle_surface_normal(U, lam);
    
    end

    C_pm = C_quad{1};
    C_win = C_quad{2};
    
    %Choose the right circles
    dotProducts = [
        dot(surface_normals{1}(:, 1), surface_normals{2}(:, 1))
        dot(surface_normals{1}(:, 2), surface_normals{2}(:, 1))
        dot(surface_normals{1}(:, 1), surface_normals{2}(:, 2))
        dot(surface_normals{1}(:, 2), surface_normals{2}(:, 2))
    ];
    
    [maxDotProduct, maxIndex] = max(dotProducts);
    
    switch maxIndex
        case 1
            PM_normal_cam = surface_normals{1}(:, 1);
            PM_center_cam = circle_centers{1}(:, 1);
            Window_normal_cam = surface_normals{2}(:, 1);
            Window_center_cam = circle_centers{2}(:, 1);
        case 2
            PM_normal_cam = surface_normals{1}(:, 2);
            PM_center_cam = circle_centers{1}(:, 2);
            Window_normal_cam = surface_normals{2}(:, 1);
            Window_center_cam = circle_centers{2}(:, 1);
        case 3
            PM_normal_cam = surface_normals{1}(:, 1);
            PM_center_cam = circle_centers{1}(:, 1);
            Window_normal_cam = surface_normals{2}(:, 2);
            Window_center_cam = circle_centers{2}(:, 2);
        case 4
            PM_normal_cam = surface_normals{1}(:, 2);
            PM_center_cam = circle_centers{1}(:, 2);
            Window_normal_cam = surface_normals{2}(:, 2);
            Window_center_cam = circle_centers{2}(:, 2);
    end

    angle_diff = acos(maxDotProduct)*180/pi;
    
end