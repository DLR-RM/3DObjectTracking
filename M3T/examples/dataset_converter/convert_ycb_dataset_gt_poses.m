function convert_ycb_dataset_gt_poses(dataset_directory, external_directory)

% Read class names
filename = fullfile(dataset_directory, 'image_sets', 'classes.txt');
file_id = fopen(filename, 'r');
c = textscan(file_id, '%s');
classes = c{1};
fclose(file_id);

% Load keyframe indexes
filename = fullfile(dataset_directory, 'image_sets', 'keyframe.txt');
file_id = fopen(filename, 'r');
k = textscan(file_id, '%s');
keyframes = k{1};
fclose(file_id);

% Create folder
ground_truth_directory = fullfile(external_directory, 'poses', 'ground_truth');
mkdir(ground_truth_directory);

% For each image
for i = 1:numel(keyframes)
    % Parse keyframe name
    name = keyframes{i};
    pos = strfind(name, '/');
    sequence_name = name(1:pos-1);
    sequence_id = str2double(sequence_name);
    frame_id = str2double(name(pos+1:end));
    
    % Load ground truth data
    filename = fullfile(dataset_directory, 'data', ...
        sprintf('%04d/%06d-meta.mat', sequence_id, frame_id));
    disp(filename);
    gt = load(filename);
    
    % For each object
    for j = 1:numel(gt.cls_indexes)
        % Parse data
        class_index = gt.cls_indexes(j);
        class_name = classes{class_index};
               
        % Calculate pose
        q = rotm2quat(gt.poses(1:3, 1:3, j));
        t = gt.poses(1:3, 4, j);
        pose = [q'; t];
        
        % Write pose to file
        filename = fullfile(ground_truth_directory, ...
            sprintf('%s_%s.txt', sequence_name, class_name));
        file_id = fopen(filename, 'a');
        fprintf(file_id,'%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', pose);
        fclose(file_id);
    end
end