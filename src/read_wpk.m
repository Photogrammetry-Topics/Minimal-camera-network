function [x,y,z,omega,phi,kappa,camera_names] = read_wpk(fullpathname)

%%%%%%%%%%%%%%%%%%%%%%%% read exterior orientation %%%%%%%%%%%%%%%%%%%%%%%%%%
wpk = fileread(fullpathname); % exterior orientation (no headers, then PhotoID, X, Y, Z, Omega, Phi, Kappa)
% Split the string into lines
lines = strsplit(wpk, '\n');

% Count non-empty lines to pre-allocate the array
lineCount = 0;
for i = 1:length(lines)
    if ~isempty(strtrim(lines{i}))
        lineCount = lineCount + 1;
    end
end

% Initialize data array and camera names cell array
data = zeros(lineCount, 6);
camera_names = cell(lineCount, 1);

% Process each line
validLineIndex = 1;
for i = 1:length(lines)
    line = strtrim(lines{i});
    if ~isempty(line)
        % Use regexp to split by comma or tab
        values = regexp(line, '[,\t]', 'split');
        if length(values) == 7
            % First value is the camera name/number
            camera_names{validLineIndex} = values{1};
            % Convert remaining values to numbers
            for j = 2:7
                data(validLineIndex, j-1) = str2double(values{j});
            end
            validLineIndex = validLineIndex + 1;
        end
    end
end

% Extract individual columns
x = data(:, 1);
y = data(:, 2);
z = data(:, 3);
omega = data(:, 4);
phi = data(:, 5);
kappa = data(:, 6);

% Display confirmation
fprintf('Loaded %d camera positions\n', validLineIndex - 1);
