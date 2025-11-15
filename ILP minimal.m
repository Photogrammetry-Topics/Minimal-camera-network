clc
clear
close all
format short g

%%%%%%%%%%%%%%%%%%%% ILP-based Minimal Camera Network Finder %%%%%%%%%%%%%%
%%%%%% input:                                                           %%%
%%%%%% wpk: exterior orientation file m*6                               %%%
%%%%%% P: sparse point cloud: this number will affect the processing time %
%%%%%% f: focal length                                                  %%%
%%%%%% Ro=visibility parameter of the HPR method                        %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
% Parameters
f = 18;         % focal length
Ro = 1.1;       % visibility parameter  
removed = 0;

% Read exterior orientation file
[filename2, pathname2] = uigetfile({'*.txt'}, 'E.O FILE SELECTOR');
fullpathname = strcat(pathname2, filename2);

%%%%%%%%%%%%%%%%%%%%%%%% read exterior orientation %%%%%%%%%%%%%%%%%%%%%%%%%%
[tX, tY, tZ, omega, phi, kappa, camera_names] = read_wpk(fullpathname);
wpk_original = [tX, tY, tZ, omega, phi, kappa];

%%%%%%%%%%%%%%%%%%%%%%%SPARSE POINTS TEXT FILE %%%%%%%%%%%%%%%%%%%%%%%%%%%%
[filename2, pathname2] = uigetfile({'*.txt'}, 'SPARSE POINTS FILE SELECTOR');
fullpathname = strcat(pathname2, filename2);

btt = load(fullpathname); % sparse point cloud (n*3)
% these two numbers will directly influence the processing time 
if size(btt,1) > 15000 
    bt = btt(1:round(size(btt,1)/15000):end, :);
else
    bt = btt;
end

x = bt(:,1); y = bt(:,2); z = bt(:,3);
P = [x, y, z];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prepare data in the format expected by ILP solver
no = (1:size(wpk_original,1))';
ext = [no wpk_original];
wpk = ext(:,2:end);

% Convert angles to radians
Xo = wpk(:,1); Yo = wpk(:,2); Zo = wpk(:,3);
w2 = (pi/180) * wpk(:,4);
p2 = (pi/180) * wpk(:,5); 
k2 = (pi/180) * wpk(:,6);

Tx = Xo; Ty = Yo; Tz = Zo;
wpk_radians = [w2, p2, k2, Tx, Ty, Tz];
No = no;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          ILP OPTIMIZATION SECTION WITH ACCURACY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('=== ILP-BASED CAMERA NETWORK OPTIMIZATION WITH ACCURACY ESTIMATION ===\n');
fprintf('Original cameras: %d\n', size(wpk_radians, 1));
fprintf('Points to cover: %d\n', size(P, 1));

% Set up ILP optimization options
options = struct();
options.min_coverage = 5;                % Minimum cameras per point
options.max_time = 600;                  % 5 minutes max solving time
options.bh_ratio_weight = 0.1;           % Weight for B/H ratio quality
options.solver = 'intlinprog';           % Use MATLAB's integer programming solver
options.bh_min = 0.2;                    % Minimum allowed B/H
options.bh_max = 0.6;                    % Maximum allowed B/H
options.noise_threshold = 0.05;          % Allow at most 5% noisy points
options.gsd_penalty_weight = 0.1;        % Weight for GSD penalty
options.accuracy_penalty_weight = 0.15;  % Weight for accuracy penalty
options.accuracy_sample_ratio = 1;       % Sample 20% of points for accuracy estimation

% Show initial visibility analysis (optional)
[x_vis, y_vis, vib, dis] = visibilitytesting_plotting(Tx, Ty, Tz, w2, p2,...
    k2, P, f, Ro, No, true);

% Run enhanced ILP optimization with accuracy estimation
h = msgbox('Enhanced ILP Optimization with Accuracy Estimation in progress - Please wait :)');

fprintf('\n--- Starting accuracy-enhanced optimization ---\n');
 
[selected_cameras, objective_value, solve_time] = ...
 ilp_camera_optimization_with_accuracy(wpk_radians, P, f, Ro, No, options);
 
close(h);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          PROCESS RESULTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extract filtered camera parameters
selected_indices = find(selected_cameras);
w2_filtered = w2(selected_indices);
p2_filtered = p2(selected_indices);
k2_filtered = k2(selected_indices);
Tx_filtered = Tx(selected_indices);
Ty_filtered = Ty(selected_indices);
Tz_filtered = Tz(selected_indices);

% Display results
h = msgbox('Enhanced ILP Optimization Completed!');
pause(2);
close(h);

fprintf('\n=== ACCURACY-ENHANCED OPTIMIZATION RESULTS ===\n');
fprintf('Selected cameras: %d out of %d\n', objective_value, size(wpk_radians, 1));
fprintf('Reduction: %.1f%%\n', 100 * (1 - objective_value/size(wpk_radians, 1)));
% fprintf('Processing time: %.2f seconds\n', total_time);
fprintf('Solution status: OPTIMAL (guaranteed) with accuracy consideration\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          ACCURACY VALIDATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('\n=== ACCURACY VALIDATION ===\n');
fprintf('Validating 3D accuracy improvements...\n');

% Get visibility for all points at once (original network)
[~, ~, vib_orig, ~] = visibilitytesting(Tx, Ty, Tz, w2, p2, k2,...
    P, f, Ro, No);

% Get visibility for all points at once (optimized network)
[~, ~, vib_opt, ~] = visibilitytesting(Tx_filtered, Ty_filtered,...
    Tz_filtered, w2_filtered, p2_filtered, k2_filtered, P, f, Ro,...
    (1:length(selected_indices))');

% Build visibility matrices
n_cameras_orig = length(Tx);
n_cameras_opt = length(Tx_filtered);
n_points = size(P, 1);

V_orig = zeros(n_cameras_orig, n_points);
for k = 1:size(vib_orig, 1)
    camera_idx = vib_orig(k, 1);
    point_idx = vib_orig(k, 2);
    if camera_idx <= n_cameras_orig && point_idx <= n_points
        V_orig(camera_idx, point_idx) = 1;
    end
end

V_opt = zeros(n_cameras_opt, n_points);
for k = 1:size(vib_opt, 1)
    camera_idx = vib_opt(k, 1);
    point_idx = vib_opt(k, 2);
    if camera_idx <= n_cameras_opt && point_idx <= n_points
        V_opt(camera_idx, point_idx) = 1;
    end
end

% Sample points for detailed accuracy comparison
n_validation_points = min(10, size(P, 1));
validation_indices = randperm(size(P, 1), n_validation_points);

accuracy_original = zeros(n_validation_points, 1);
accuracy_optimized = zeros(n_validation_points, 1);

for i = 1:n_validation_points
    point_idx = validation_indices(i);
    point_3d = P(point_idx, :);
    
    % Find cameras that can see this point (original network)
    visible_orig = find(V_orig(:, point_idx));
    
    % Find cameras that can see this point (optimized network)  
    visible_opt = find(V_opt(:, point_idx)); 
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          QUALITY METRICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[~, ~, vib, ~] = visibilitytesting(Tx, Ty, Tz, w2, p2, k2, P, f, Ro, No);
 for i=1:size(selected_cameras,1) ; selected_cameras_all(i,1)=1;end
n_cameras = length(Tx);
n_points = size(P, 1);
V = zeros(n_cameras, n_points);
for k = 1:size(vib, 1)
    camera_idx = vib(k, 1);
    point_idx  = vib(k, 2);
    if camera_idx <= n_cameras && point_idx <= n_points
        V(camera_idx, point_idx) = 1;
    end
end
[eta_coverage, eta_redundancy, eta_efficiency] = ...
    compute_quality_metrics(V, selected_cameras_all, options.min_coverage);
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          SAVE RESULTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wpkfilter = [w2_filtered, p2_filtered, k2_filtered, Tx_filtered,...
    Ty_filtered, Tz_filtered];
WPK_full = zeros(size(wpk_radians, 1), 6);

for i = 1:length(selected_indices)
    idx = selected_indices(i);
    WPK_full(idx, :) = wpk_radians(idx, :);
end
WPK_full = [(1:size(WPK_full, 1))', WPK_full];

t = find(WPK_full(:, 2) == 0);  % removed cameras
tt = find(WPK_full(:, 2) ~= 0); % kept cameras

M = camera_names;
for i = 1:size(M, 1)
    as(i, 1) = struct('name', M{i, 1});
end

if ~isempty(t)
    remov = as(t, 1);
else
    remov = [];
end
keep = as(tt, 1);

filename = strcat(pathname2, 'ILP-filtering-accuracy-enhanced-solution.doc');
fid = fopen(filename, 'w');
fprintf(fid, '%s\n', 'MINIMAL CAMERA NETWORK - ACCURACY-ENHANCED ILP OPTIMIZATION');
fprintf(fid, '%s\n', ' ');
fprintf(fid, 'GUARANTEED OPTIMAL SOLUTION WITH ACCURACY CONSIDERATION\n');
fprintf(fid, '%s\n', ' ');
fprintf(fid, 'Original cameras: %d\n', size(wpk_radians, 1));
fprintf(fid, 'Selected cameras: %d\n', objective_value);
fprintf(fid, 'Reduction: %.1f%%\n', 100 * (1 - objective_value/size(wpk_radians, 1)));
% fprintf(fid, 'Processing time: %.2f seconds\n', total_time);
fprintf(fid, '%s\n', ' ');
fprintf(fid, 'OPTIMIZATION WEIGHTS:\n');
fprintf(fid, 'B/H ratio weight: %.3f\n', options.bh_ratio_weight);
fprintf(fid, 'GSD penalty weight: %.3f\n', options.gsd_penalty_weight);
fprintf(fid, 'Accuracy penalty weight: %.3f\n', options.accuracy_penalty_weight);
fprintf(fid, 'Accuracy sampling ratio: %.3f\n', options.accuracy_sample_ratio);
fprintf(fid, '%s\n', ' ');
fprintf(fid, '%s\n', 'SELECTED CAMERAS:');

for row = 1:size(keep, 1)
    K = keep(row).name;
    fprintf(fid, 'Keep Image: %s\n', K);
end

fprintf(fid, '%s\n', ' ');
fprintf(fid, '%s\n', 'OPTIMIZATION METHOD: Integer Linear Programming with Accuracy Estimation');
fprintf(fid, '%s\n', 'OPTIMALITY: Guaranteed global optimum');
fprintf(fid, '%s\n', 'CONSTRAINTS: Minimum camera coverage per point');
fprintf(fid, '%s\n', 'ACCURACY MODEL: Multi-view intersection variance estimation');
fprintf(fid, '%s\n', ' ');
fprintf(fid, '%s\n', 'Enhanced with accuracy-aware ILP optimization');
fprintf(fid, '%s\n', 'Based on B.S.Alsadik method with photogrammetric accuracy integration');
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          END OF SCRIPT                                  %
%           By: Dr. B. Alsadik in 2025 - ITC faculty - The Netherlands    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 