function [x, y, vib, dis] = visibilitytesting_plotting(Tx, Ty, Tz, w2, p2, k2, P, f, Ro, No, plotResults)
    % Pre-allocate for better performance with aggressive optimization
    numCameras = size(Tx, 1);
    numPoints = size(P, 1);
    
    % Adaptive pre-allocation based on expected visibility
    estVisiblePoints = ceil(min(numPoints * numCameras * 0.3, 1e6)); % More conservative estimate
    vib = zeros(estVisiblePoints, 8);
    
    % Ensure P has only 3 columns (x,y,z)
    if size(P, 2) > 3
        P = P(:, 1:3);
    end
    
    % Add point indices to P - precompute once
    PP = [P, (1:numPoints)'];
    
    % Initialize displacement vector
    dis = zeros(numCameras, 1);
    
    % Track position in vib for appending
    vibIdx = 1;
    
    % Pre-compute constants for image boundary check
    x_bound = 11.4;
    y_bound = 7.3;
    
    % Pre-compute rotation matrices for all cameras at once
    cosW = cos(w2); sinW = sin(w2);
    cosP = cos(p2); sinP = sin(p2);
    cosK = cos(k2); sinK = sin(k2);
    
    % Store last valid x and y for return values
    lastValidX = [];
    lastValidY = [];
    
    % Create a figure for visualization if plotting is requested
    % if nargin > 10 && plotResults
        % figure('Name', 'Camera Visibility Analysis', 'Position', [100, 100, 1200, 800]);
        
        % Plot all points in gray first
        % subplot(1, 2, 1);
        % scatter3(PP(:,1), PP(:,2), PP(:,3), 5, [0.7 0.7 0.7], 'filled');
        % hold on;
        
        % Plot camera positions
        % scatter3(Tx, Ty, Tz, 100, 'k', 'filled', 'MarkerEdgeColor', 'k');
        % 
        % % Label cameras
        % for i = 1:numCameras
        %     text(Tx(i), Ty(i), Tz(i), sprintf(' Cam %d', No(i)), 'FontSize', 10);
        % end
        % 
        % title('Point Cloud and Camera Positions');
        % xlabel('X'); ylabel('Y'); zlabel('Z');
        % grid on; axis equal;
        
        % Create second subplot for visible points
        % subplot(1, 2, 2);
        scatter3(PP(:,1), PP(:,2), PP(:,3), 5, [0.9 0.9 0.9], 'filled', 'MarkerFaceAlpha', 0.1);
        hold on;
        
        % Plot camera positions
        scatter3(Tx, Ty, Tz, 100, 'k', 'filled', 'MarkerEdgeColor', 'k');
        
        % Label cameras
        % for i = 1:numCameras
        %     text(Tx(i), Ty(i), Tz(i), sprintf(' Cam %d', No(i)), 'FontSize', 10);
        % end
        
        title('Visible Points by Camera');
        xlabel('X'); ylabel('Y'); zlabel('Z');
        grid on; axis equal;
        
        % Create colormap for cameras
        colormap(jet(numCameras));
        cameraColors = jet(numCameras);
    % end
    
    % Store visible points for each camera separately
    visiblePointsByCam = cell(numCameras, 1);
    
    % Process each camera
    for j = 1:numCameras
        % Camera position
        C = [Tx(j), Ty(j), Tz(j)];
        
        % Hidden point removal - find visible points
        visiblePtInds = HPR(PP(:, 1:3), C, Ro);
         
        % Skip if no visible points
        if isempty(visiblePtInds)
            continue;
        end
        
        % Extract visible points
        Pk = PP(visiblePtInds, :);
        
        % Store visible points for this camera
        if nargin > 10 && plotResults
            visiblePointsByCam{j} = Pk;
        end
        
        % Build rotation matrix efficiently using pre-computed values
        mw = [1 0 0; 0 cosW(j) sinW(j); 0 -sinW(j) cosW(j)];
        mp = [cosP(j) 0 -sinP(j); 0 1 0; sinP(j) 0 cosP(j)];
        mk = [cosK(j) sinK(j) 0; -sinK(j) cosK(j) 0; 0 0 1];
        
        % Combined rotation matrix - compute only once per camera
        m = mk * mp * mw;
        
        % Vectorized computation of camera-to-point vectors
        dx1 = Pk(:, 1) - Tx(j);
        dy1 = Pk(:, 2) - Ty(j);
        dz1 = Pk(:, 3) - Tz(j);
        
        % Compute distance vector once (reused later)
        D = sqrt(dx1.^2 + dy1.^2 + dz1.^2);
        
        % Vectorized computation of rotated coordinates
        pts = [dx1, dy1, dz1];
        rotated = pts * m'; % Transpose m for efficient matrix multiplication
        
        % Extract components directly
        r = rotated(:, 1);
        s = rotated(:, 2);
        q = rotated(:, 3);
        
        % Compute image coordinates efficiently
        validQ = (abs(q) > 1e-10);
        
        % Initialize with NaN
        x_img = nan(size(q));
        y_img = nan(size(q));
        
        % Compute only for valid q values
        x_img(validQ) = -f * (r(validQ) ./ q(validQ));
        y_img(validQ) = -f * (s(validQ) ./ q(validQ));
        
        % Find points within image bounds - use logical indexing
       %validIdx = validQ & (abs(x_img) < x_bound) & (abs(y_img) < y_bound);
       % Then check image bounds
validIdx = validQ & (x_img > -x_bound) & (x_img < x_bound) & (y_img > -y_bound) & (y_img < y_bound);

% Only store points that are both visible via HPR AND within camera frame
if any(validIdx)
    visiblePointsByCam{j} = Pk(validIdx, :);
else
    visiblePointsByCam{j} = [];
end
  
        % Extract valid points efficiently
        if any(validIdx)
            validPoints = Pk(validIdx, :);
            validX = x_img(validIdx);
            validY = y_img(validIdx);
            validD = D(validIdx);
            
            % Compute mean distance for valid points
            dis(j) = mean(validD);
            
            % Number of visible points for this camera
            numVisible = size(validPoints, 1);
            
            % Check if we need to resize vib
            if vibIdx + numVisible > size(vib, 1)
                vib = [vib; zeros(estVisiblePoints, 8)];
            end
            
            % Create batch update for vib
            vibBatch = [
                No(j) * ones(numVisible, 1), validPoints(:, 4), 2 * ones(numVisible, 1), ...
                validPoints(:, 1:3), validX, validY
            ];
            
            % Batch update vib
            vib(vibIdx:(vibIdx+numVisible-1), :) = vibBatch;
            vibIdx = vibIdx + numVisible;
            
            % Store last valid coordinates
            lastValidX = validX;
            lastValidY = validY;
        else
            % If camera has no valid points, use mean of all distances
            if ~isempty(D)
                dis(j) = mean(D);
            else
                dis(j) = 0;
            end
        end
    end
    x_all = nan(vibIdx - 1, 1);  % same length as vib
    y_all = nan(vibIdx - 1, 1);

    % Trim vib to actual size
    if vibIdx > 1
        vib = vib(1:vibIdx-1, :);
        x = lastValidX;
        y = lastValidY;
    else
        vib = zeros(0, 8);
        x = [];
        y = [];
    end
    
    % Plot visible points for each camera with different colors
    if nargin > 10 && plotResults
        % subplot(1, 2, 2);
        
        % Plot visible points for each camera with a unique color
        % for j = 1:numCameras
        %     if ~isempty(visiblePointsByCam{j})
        %         % Get data for this camera's visible points
        %         camPoints = visiblePointsByCam{j};
        % 
        %         % Plot visible points with camera-specific color
        %         % scatter3(camPoints(:,1), camPoints(:,2), camPoints(:,3), 15, ...
        %             % repmat(cameraColors(j,:), size(camPoints,1), 1), 'filled');
        %     end
        % end
        
        % Create custom legend
        % legendItems = cell(1, numCameras);
        % for j = 1:numCameras
        %     legendItems{j} = sprintf('Camera %d', No(j));
        % end
        % legend(legendItems, 'Location', 'eastoutside');
        
        % Add coverage percentage information
        totalPoints = size(PP, 1);
        coverageText = 'Camera Coverage:';
        for j = 1:numCameras
            if ~isempty(visiblePointsByCam{j})
                coveragePct = 100 * size(visiblePointsByCam{j}, 1) / totalPoints;
                coverageText = [coverageText, sprintf('\nCam %d: %.1f%%', No(j), coveragePct)];
            else
                coverageText = [coverageText, sprintf('\nCam %d: 0.0%%', No(j))];
            end
        end
         
        % annotation('textbox', [0.02, 0.02, 0.2, 0.2], 'String', coverageText, ...
        %     'FitBoxToText', 'on', 'BackgroundColor', 'white');
        
        % Create a third subplot for summary visualization (heat map of coverage)
       % figure('Name', 'Coverage Analysis', 'Position', [100, 100, 800, 600]);
        
        % Count how many cameras can see each point
        coverageCount = zeros(numPoints, 1);
        
        for j = 1:numCameras
            if ~isempty(visiblePointsByCam{j})
                % Get indices of points visible to this camera
                visibleIndices = visiblePointsByCam{j}(:, 4);
                
                % Increment counter for each visible point
                coverageCount(visibleIndices) = coverageCount(visibleIndices) + 1;
            end
        end
         
        % Plot points colored by coverage count
        scatter3(PP(:,1), PP(:,2), PP(:,3), 15, coverageCount, 'filled');hold on
        %scatter3(validPoints(:,1), validPoints(:,2), validPoints(:,3), 20,  'filled'); 
        colorbar;
        colormap(parula);
        clim([0 max(coverageCount)]);
        title('Number of Cameras Seeing Each Point');
        xlabel('X'); ylabel('Y'); zlabel('Z');
        grid on; axis equal;
        
        % Add camera positions
        % hold on;
        % scatter3(Tx, Ty, Tz, 100, 'k', 'filled', 'MarkerEdgeColor', 'k');
        
        % Label cameras
        % for i = 1:numCameras
        %     text(Tx(i), Ty(i), Tz(i), sprintf(' Cam %d', No(i)), 'FontSize', 10);
        % end
    end
end
