function convert_choi_dataset(dataset_directory, external_directory)

% Global settings
sequences = {
    'seq_synth_kinect_box_kitchen'
    'seq_synth_milk_kitchen'
    'seq_synth_orange_juice_kitchen'
    'seq_synth_tide_kitchen'};
n_frames = 1000;

% For each sequence
for i = 1 : length(sequences)
    sequence = sequences(i);
    
    % Create folder
    sequence_directory = fullfile(external_directory, sequence);
    mkdir(sequence_directory);
    
    % For each pointcloud
    for i_frame = 0 : n_frames - 1
        % Read pointcloud
        filename = fullfile(dataset_directory, sequence, ...
            sprintf('cloud%04d.pcd',i_frame));
        disp(filename);
        point_cloud = pcread(filename);
        
        % Convert pointcloud to images
        color_image = point_cloud.Color;
        depth_image_meter = point_cloud.Location(:, :, 3);
        depth_image = uint16(depth_image_meter * 10000);
        
        % Save images
        color_path = strcat(sequence_directory, ...
            sprintf('color%04d.png',i_frame));
        depth_path = strcat(sequence_directory, ...
            sprintf('depth%04d.png',i_frame));
        imwrite(color_image, color_path);
        imwrite(depth_image, depth_path);
    end
end