function [selected_cameras, objective_value, solve_time] = ilp_camera_optimization_with_accuracy(wpk, P, f, Ro, No, options)
 
if nargin < 6
    options = struct();
end

% Default options
if ~isfield(options, 'min_coverage'), options.min_coverage = 3; end
if ~isfield(options, 'max_time'), options.max_time = 300; end
if ~isfield(options, 'solver'), options.solver = 'intlinprog'; end
if ~isfield(options, 'bh_ratio_weight'), options.bh_ratio_weight = 0.05; end
if ~isfield(options, 'gsd_penalty_weight'), options.gsd_penalty_weight = 0.05; end
if ~isfield(options, 'accuracy_penalty_weight'), options.accuracy_penalty_weight = 0.1; end
if ~isfield(options, 'bh_min'), options.bh_min = 0.2; end
if ~isfield(options, 'bh_max'), options.bh_max = 0.6; end
if ~isfield(options, 'noise_threshold'), options.noise_threshold = 0.05; end
if ~isfield(options, 'accuracy_sample_ratio'), options.accuracy_sample_ratio = 0.1; end

fprintf('=== ENHANCED ILP CAMERA OPTIMIZATION (with Accuracy Estimation) ===\n');
tic;

% Extract camera parameters
w2 = wpk(:,1); p2 = wpk(:,2); k2 = wpk(:,3);
Tx = wpk(:,4); Ty = wpk(:,5); Tz = wpk(:,6);

n_cameras = length(Tx);
n_points = size(P, 1);

fprintf('Cameras: %d, Points: %d\n', n_cameras, n_points);

% Step 1: Compute visibility matrix
fprintf('Computing visibility matrix...\n');
[~, ~, vib, ~] = visibilitytestingmarkus(Tx, Ty, Tz, w2, p2, k2, P, f, Ro, No);
V = zeros(n_cameras, n_points);
for k = 1:size(vib, 1)
    camera_idx = vib(k, 1);
    point_idx = vib(k, 2);
    if camera_idx <= n_cameras && point_idx <= n_points
        V(camera_idx, point_idx) = 1;
    end
end
fprintf('Visibility matrix: %d cameras, %d points, %d visible pairs\n', ...
    n_cameras, n_points, nnz(V));

% Step 2: Compute B/H penalty matrix
P_bh = compute_bh_penalty(Tx, Ty, Tz, P, V, options.bh_min, options.bh_max);

% Step 3: Compute GSD penalty
G_gsd = compute_gsd_penalty(Tx, Ty, Tz, P, V);

% Step 4: Compute accuracy penalty matrix (NEW)
fprintf('Computing accuracy penalties using multi-view intersection...\n');
A_acc = compute_accuracy_penalty(wpk, P, V, f, options.accuracy_sample_ratio);

% Step 5: Remove noisy points with too few possible coverages
coverage_per_point = sum(V, 1);
valid_points = find(coverage_per_point >= options.min_coverage);
removed_points = find(coverage_per_point < options.min_coverage);

V_clean = V(:, valid_points);
P_bh_clean = P_bh(:, valid_points);
G_clean = G_gsd(:, valid_points);
A_acc_clean = A_acc(:, valid_points);
n_points_clean = length(valid_points);

fprintf('Valid points (>= %d cameras): %d\n', options.min_coverage, n_points_clean);
fprintf('Noisy points (< %d cameras): %d\n', options.min_coverage, length(removed_points));

% Step 6: Build and solve ILP with all penalties
fprintf('Solving enhanced ILP with accuracy, B/H, and GSD penalties...\n');

% Compute all penalties per camera
bh_penalty = sum(P_bh_clean, 2);
gsd_penalty = sum(G_clean, 2);
accuracy_penalty = sum(A_acc_clean, 2);

% Enhanced objective function with accuracy term
f_obj = ones(n_cameras,1) ...
       + options.bh_ratio_weight * bh_penalty ...
       + options.gsd_penalty_weight * gsd_penalty ...
       + options.accuracy_penalty_weight * accuracy_penalty;

A_coverage = -V_clean';
b_coverage = -options.min_coverage * ones(n_points_clean, 1);
lb = zeros(n_cameras, 1);
ub = ones(n_cameras, 1);
intcon = 1:n_cameras;

opts = optimoptions('intlinprog', ...
    'Display', 'iter', ...
    'MaxTime', min(options.max_time, 120), ...
    'IntegerTolerance', 1e-6, ...
    'RelativeGapTolerance', 0.02, ...
    'CutGeneration', 'advanced', ...
    'Heuristics', 'advanced');

tic;
[x, ~, exitflag, ~] = intlinprog(f_obj, intcon, A_coverage, b_coverage, [], [], lb, ub, opts);
solve_time = toc;

if exitflag == 1 || exitflag == 0
    selected_cameras = round(x);
    objective_value = sum(selected_cameras);
    fprintf('✓ Enhanced ILP solution found: %d cameras selected\n', objective_value);
    
    % Display penalty contributions
    fprintf('Penalty contributions per camera (average):\n');
    fprintf('  B/H penalty: %.4f\n', mean(bh_penalty));
    fprintf('  GSD penalty: %.4f\n', mean(gsd_penalty));
    fprintf('  Accuracy penalty: %.4f\n', mean(accuracy_penalty));
else
    fprintf('ILP failed (exitflag: %d)\n', exitflag);
    selected_cameras = [];
    objective_value = 0;
end
end

function A_penalty = compute_accuracy_penalty(wpk, P, V, f, sample_ratio)
% Compute accuracy penalty matrix using multi-view intersection
% This function estimates the 3D accuracy for each point-camera combination

n_cameras = size(wpk, 1);
n_points = size(P, 1);
A_penalty = zeros(n_cameras, n_points);

% Sample points for accuracy estimation (to reduce computational load)
n_sample = max(1, round(n_points * sample_ratio));
if n_sample < n_points
    sample_indices = randperm(n_points, n_sample);
    fprintf('Sampling %d out of %d points for accuracy estimation\n', n_sample, n_points);
else
    sample_indices = 1:n_points;
end

% Progress tracking
fprintf('Estimating accuracy for %d sample points...\n', length(sample_indices));
progress_step = max(1, floor(length(sample_indices) / 10));

for idx = 1:length(sample_indices)
    point_idx = sample_indices(idx);
    
    if mod(idx, progress_step) == 0
        fprintf('Progress: %d/%d points processed\n', idx, length(sample_indices));
    end
    
    % Find cameras that can see this point
    visible_cameras = find(V(:, point_idx));
    
    if length(visible_cameras) >= 3
        % Estimate accuracy using all possible camera combinations
        accuracy_estimates = estimate_point_accuracy_combinations(wpk, P(point_idx, :), ...
                                                                visible_cameras, f);
        
        % Assign penalties based on accuracy estimates
        for i = 1:length(visible_cameras)
            cam_idx = visible_cameras(i);
            % Higher penalty for cameras that contribute to poor accuracy
            % Use inverse relationship: worse accuracy = higher penalty
            if accuracy_estimates(i) > 0
                A_penalty(cam_idx, point_idx) = 1.0 / accuracy_estimates(i);
            else
                A_penalty(cam_idx, point_idx) = 1000; % Very high penalty for problematic cases
            end
        end
    end
end

% Normalize penalties to reasonable range [0, 1]
if max(A_penalty(:)) > 0
    A_penalty = A_penalty / max(A_penalty(:));
end

% Interpolate for non-sampled points using nearest neighbor approach
if n_sample < n_points
    fprintf('Interpolating accuracy penalties for remaining points...\n');
    non_sample_indices = setdiff(1:n_points, sample_indices);
    
    for point_idx = non_sample_indices
        % Find nearest sampled point
        distances = sqrt(sum((P(sample_indices, :) - P(point_idx, :)).^2, 2));
        [~, nearest_idx] = min(distances);
        nearest_sample_idx = sample_indices(nearest_idx);
        
        % Copy penalty pattern
        A_penalty(:, point_idx) = A_penalty(:, nearest_sample_idx);
    end
end

fprintf('Accuracy penalty computation completed.\n');
end

function accuracy_estimates = estimate_point_accuracy_combinations(wpk, point_3d, visible_cameras, f)
% Estimate accuracy for different camera combinations for a single 3D point
% Returns accuracy estimate for each camera's contribution

n_visible = length(visible_cameras);
accuracy_estimates = zeros(n_visible, 1);

if n_visible < 3
    accuracy_estimates(:) = 0;
    return;
end

% Generate simulated image coordinates for this 3D point
[xp_sim, yp_sim] = simulate_image_coordinates(wpk, point_3d, visible_cameras, f);

% Try different camera combinations to assess each camera's contribution
min_combinations = min(10, nchoosek(n_visible, 3)); % Limit number of combinations

for cam_idx = 1:n_visible
    current_cam = visible_cameras(cam_idx);
    accuracies = [];
    
    % Test this camera with different combinations of other cameras
    other_cameras = visible_cameras(visible_cameras ~= current_cam);
    
    if length(other_cameras) >= 2
        % Try multiple combinations including this camera
        n_combinations = min(5, nchoosek(length(other_cameras), 2));
        
        for combo = 1:n_combinations
            if length(other_cameras) >= 2
                % Select 2 other cameras randomly
                selected_others = randsample(other_cameras, 2);
                test_cameras = [current_cam; selected_others];
                
                % Find indices in visible_cameras array
                test_indices = arrayfun(@(x) find(visible_cameras == x), test_cameras);
                
                % Extract parameters for these cameras
                wpk_test = wpk(test_cameras, :);
                xp_test = xp_sim(test_indices);
                yp_test = yp_sim(test_indices);
                
                % Perform multi-view intersection
                try
                    [~, sx, sy, sz, ~] = mv_intersect_ilp(wpk_test, f, xp_test, yp_test);
                    
                    % Compute 3D accuracy measure (combined standard deviation)
                    accuracy_3d = sqrt(sx^2 + sy^2 + sz^2);
                    
                    if ~isnan(accuracy_3d) && ~isinf(accuracy_3d) && accuracy_3d > 0
                        accuracies = [accuracies; accuracy_3d];
                    end
                catch
                    % Skip this combination if it fails
                    continue;
                end
            end
        end
    end
    
    % Assign accuracy estimate for this camera
    if ~isempty(accuracies)
        accuracy_estimates(cam_idx) = mean(accuracies);
    else
        accuracy_estimates(cam_idx) = 1.0; % Default moderate accuracy
    end
end

% Ensure all estimates are positive
accuracy_estimates(accuracy_estimates <= 0) = 1.0;
end

function [xp_sim, yp_sim] = simulate_image_coordinates(wpk, point_3d, visible_cameras, f)
% Simulate image coordinates for a 3D point in visible cameras
% This is needed since we only have 3D coordinates, not actual image measurements

n_cameras = length(visible_cameras);
xp_sim = zeros(n_cameras, 1);
yp_sim = zeros(n_cameras, 1);

for i = 1:n_cameras
    cam_idx = visible_cameras(i);
    
    % Extract camera parameters
    omega = wpk(cam_idx, 1);
    phi = wpk(cam_idx, 2);
    kappa = wpk(cam_idx, 3);
    Xo = wpk(cam_idx, 4);
    Yo = wpk(cam_idx, 5);
    Zo = wpk(cam_idx, 6);
    
    % Compute rotation matrix
    mw = [1 0 0; 0 cos(omega) sin(omega); 0 -sin(omega) cos(omega)];
    mp = [cos(phi) 0 -sin(phi); 0 1 0; sin(phi) 0 cos(phi)];
    mk = [cos(kappa) sin(kappa) 0; -sin(kappa) cos(kappa) 0; 0 0 1];
    M = mk * mp * mw;
    
    % Transform 3D point to camera coordinate system
    X = point_3d(1) - Xo;
    Y = point_3d(2) - Yo;
    Z = point_3d(3) - Zo;
    
    % Apply rotation
    point_cam = M * [X; Y; Z];
    
    % Project to image coordinates using collinearity equations
    if abs(point_cam(3)) > 1e-10 % Avoid division by zero
        xp_sim(i) = -f * point_cam(1) / point_cam(3);
        yp_sim(i) = -f * point_cam(2) / point_cam(3);
    else
        % Point is at infinity or behind camera
        xp_sim(i) = 0;
        yp_sim(i) = 0;
    end
    
    % Add small amount of realistic noise to simulate measurement uncertainty
    noise_std = 0.005; % 5 micrometers standard deviation
    xp_sim(i) = xp_sim(i) + noise_std * randn();
    yp_sim(i) = yp_sim(i) + noise_std * randn();
end
end

% Include the original mv_intersect_ilp function
function[XYZ,sx,sy,sz,sigm]=mv_intersect_ilp(wpk,f,xp,yp)
% Multi-photo intersection with least squares adjustment
% Enhanced version of the original function

format short g 
XYZ=[0 0 0 ];

no=size(wpk,1); % no. of images match
ext=wpk ;

for i=1:no
    omega(i,1)=ext(i,1);phi(i,1)=ext(i,2);kappa(i,1)=ext(i,3);...
        xo(i,1)=ext(i,4);yo(i,1)=ext(i,5);zo(i,1)=ext(i,6);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Rotation Matrix M %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    mw=[1 0 0;0 cos(omega(i,1)) sin(omega(i,1)) ;0 -sin(omega(i,1)) cos(omega(i,1))];
    mp=[cos(phi(i,1)) 0 -sin(phi(i,1));0 1 0;sin(phi(i,1)) 0 cos(phi(i,1))];
    mk=[cos(kappa(i,1)) sin(kappa(i,1)) 0;-sin(kappa(i,1)) cos(kappa(i,1)) 0;0 0 1];
    m=mk*mp*mw;   
    m11(i,1)=  m(1,1);
    m12(i,1)=  m(1,2);
    m13(i,1)=  m(1,3);
    m21(i,1)=  m(2,1);
    m22(i,1)=  m(2,2);
    m23(i,1)=  m(2,3);
    m31(i,1)=  m(3,1);
    m32(i,1)=  m(3,2);
    m33(i,1)=  m(3,3);
end
xl=xp;yl=yp;B=[0 0 0];F=0;

for i=1:size(xp,1)
    b11(i,1)=(xl(i,1)*m31(i,1)+(f*m11(i,1)));
    b12(i,1)=(xl(i,1)*m32(i,1)+(f*m12(i,1)));
    b13(i,1)=(xl(i,1)*m33(i,1)+(f*m13(i,1)));
    b21(i,1)=(yl(i,1)*m31(i,1)+(f*m21(i,1)));
    b22(i,1)=(yl(i,1)*m32(i,1)+(f*m22(i,1)));
    b23(i,1)=(yl(i,1)*m33(i,1)+(f*m23(i,1)));
    fx1(i,1)=-((-f*m11(i,1)-xl(i,1)*m31(i,1))*xo(i,1)-(f*m13(i,1)+xl(i,1)*m33(i,1))*zo(i,1)-(f*m12(i,1)+xl(i,1)*m32(i,1))*yo(i,1)) ;
    fy1(i,1)=-((-f*m21(i,1)-yl(i,1)*m31(i,1))*xo(i,1)-(f*m23(i,1)+yl(i,1)*m33(i,1))*zo(i,1)-(f*m22(i,1)+yl(i,1)*m32(i,1))*yo(i,1)) ;
    B=[B;[b11(i,1) b12(i,1) b13(i,1);b21(i,1) b22(i,1) b23(i,1)]];
    F=[F;fx1(i,1);fy1(i,1)];
end
F(1,:)=[];B(1,:)=[];
ff=F;b=B; 

%  %%%%%%%%%%%%%%%%%%%%%   least square     %%%%%%%%%%%%%%%%%%%%%%%%%%%%

btb=inv(b'*b); 
btf=b'*ff;

delta=btb*btf   ;
res=b*delta-ff;
sigm=(res'*res)/(size(b,1)-size(b,2)) ;
vcov= sigm* (btb);

sx=sqrt( (vcov(1,1)));
sy=sqrt( (vcov(2,2)));
sz=sqrt( (vcov(3,3)));

xx = delta(1,1);
yy = delta(2,1);
zz = delta(3,1);

XYZ=[XYZ;xx,yy,zz]; 
XYZ(1,:)=[];
end
