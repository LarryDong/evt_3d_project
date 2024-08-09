clc;clear;close all;

%% settings
camera_width = 1280;
camera_height = 720;

projector_width_full = 720;     % projector is rotated
projector_height_full = 1280;
pattern = 6;

projector_width_grid_size = 2 ^ (ceil(log2(projector_width_full)) - pattern);
projector_height_grid_size =  2 ^ (ceil(log2(projector_height_full)) - pattern);
projector_ds_resolution_x = projector_width_full / projector_width_grid_size;
projector_ds_resolution_y = projector_height_full / projector_height_grid_size;



%% Calibration data from calib-gui software output.
% Projector-Camera Stereo calibration parameters:
% Intrinsic parameters of camera:
fc_left = [ 1937.091372 1935.416307 ]; % Focal Length
cc_left = [ 342.643937 628.507759 ]; % Principal point
alpha_c_left = [ 0.000000 ]; % Skew
kc_left = [ -0.282564 0.385625 -0.000266 0.001436 0.000000 ]; % Distortion
% kc_left = [0,0,0,0,0]; % Distortion

% Intrinsic parameters of projector:
fc_right = [ 111.636340 55.139592 ]; % Focal Length
cc_right = [ 21.629579 14.270182 ]; % Principal point
alpha_c_right = [ 0.000000 ]; % Skew
kc_right = [ -0.185840 1.534320 -0.033168 -0.009801 0.000000 ]; % Distortion

% Extrinsic parameters (position of projector wrt camera):
om = [ -0.101791 -0.189853 -0.012500 ]; % Rotation vector
T = [ 112.240724 -31.229141 -19.591589 ]; % Translation vector



%% deal with resolution downsampling.
Dx = projector_width_grid_size;
Dy = projector_height_grid_size;
fc_right = [fc_right(1)*Dx, fc_right(2)*Dy];
cc_right = [cc_right(1)*Dx, cc_right(2)*Dy];
kc_right = [kc_right(1)/(Dx^2), kc_right(2)/(Dy^2), kc_right(3)/Dx, kc_right(4)/Dy, kc_right(5)];
T = T*1e-3;


%% Load images
image2 = imread('own_013.bmp');
image2 = flip(image2');
image1 = imread('graycode_24.png');


%% calibration test

% Camera matrices
camera_K = [fc_left(1), alpha_c_left, cc_left(1);
      0, fc_left(2), cc_left(2);
      0, 0, 1];

projector_K = [fc_right(1), alpha_c_right, cc_right(1);
      0, fc_right(2), cc_right(2);
      0, 0, 1];

% Distortion coefficients
camera_D = kc_left;
projector_D = kc_right;

% Convert rotation vector to rotation matrix
R = rotationVectorToMatrix(om);

% Stereo rectification
cameraParams1 = cameraParameters( ...
    'IntrinsicMatrix', camera_K', ...
    'RadialDistortion', camera_D(1:2), ...
    'TangentialDistortion', camera_D(3:4), ...
    'ImageSize', size(image1(:,:,1)));

cameraParams2 = cameraParameters( ...
    'IntrinsicMatrix', projector_K', ...
    'RadialDistortion', projector_D(1:2), ...
    'TangentialDistortion', projector_D(3:4), ...
    'ImageSize', size(image2(:,:,1)));

stereoParams = stereoParameters(cameraParams1, cameraParams2, R, T);

[J1, J2] = rectifyStereoImages(image1, image2, stereoParams,  'OutputView', 'full');


% Display rectified images
figure;
subplot(1,2,1), imshow(J1), title('Camera-Rectified');
subplot(1,2,2), imshow(J2), title('Projector-Rectified');