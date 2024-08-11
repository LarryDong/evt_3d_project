clc;clear;close all;

%% settings
camera_width = 720;
camera_height = 1280;

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
% 投影仪"相对于"相机的外参，即相机"位于"投影仪下的参数

% Convert rotation vector to rotation matrix
T = T*1e-3;
R_pro_cam = rotationVectorToMatrix(om);
t_pro_cam = T';
R_cam_pro = R_pro_cam';
t_cam_pro = -R_pro_cam'*t_pro_cam;
% T应该是：相机在投影仪坐标系下的坐标。R*x_camera + T = x_projectgor


%% deal with resolution downsampling.
Dx = projector_width_grid_size;
Dy = projector_height_grid_size;
fc_right = [fc_right(1)*Dx, fc_right(2)*Dy];
cc_right = [cc_right(1)*Dx, cc_right(2)*Dy];
kc_right = [kc_right(1)/(Dx^2), kc_right(2)/(Dy^2), kc_right(3)/Dx, kc_right(4)/Dy, kc_right(5)];
T = T*1e-3;

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

camera_Intrinsics = cameraIntrinsics( ...
    fc_left, cc_left, [camera_height, camera_width], ...
    'RadialDistortion', camera_D(1:2), ...
    'TangentialDistortion', camera_D(3:4));


%% Stereo rectification test 双目矫正测试。
% % Load images
% image2 = imread('own_013.bmp');
% image2 = flip(image2');
% image1 = imread('graycode_24.png');
% 
% cameraParams1 = cameraParameters( ...
%     'IntrinsicMatrix', camera_K', ...
%     'RadialDistortion', camera_D(1:2), ...
%     'TangentialDistortion', camera_D(3:4), ...
%     'ImageSize', size(image1(:,:,1)));
% 
% cameraParams2 = cameraParameters( ...
%     'IntrinsicMatrix', projector_K', ...
%     'RadialDistortion', projector_D(1:2), ...
%     'TangentialDistortion', projector_D(3:4), ...
%     'ImageSize', size(image2(:,:,1)));
% 
% stereoParams = stereoParameters(cameraParams1, cameraParams2, R_pro_cam, t_pro_cam);    % 双目标定，第3/4参数为2相对于1的姿态，即1变化为2的姿态。
% 
% [J1, J2] = rectifyStereoImages(image1, image2, stereoParams,  'OutputView', 'full');
% 
% % Display rectified images
% figure;
% subplot(1,2,1), imshow(J1), title('Camera-Rectified');
% subplot(1,2,2), imshow(J2), title('Projector-Rectified');


%% 测试相机视角变换结果
% 加载相机拍摄的图像
chessboard_image = imread('graycode_00.png');
chessboard_image = undistortImage(chessboard_image, camera_Intrinsics); 
% imshow(chessboard_image_undistorted)

% 棋盘格的基本参数
square_size = 0.0235; % 每个方块的边长，单位为米
num_corners_x = 6; % 棋盘格内横向角点数
num_corners_y = 9; % 棋盘格内纵向角点数
board_size = [num_corners_x+1, num_corners_y+1];

% 棋盘格3D世界坐标系下的角点坐标
[X, Y] = meshgrid(0:num_corners_x-1, 0:num_corners_y-1);
object_points = myGeneratePoints(board_size, square_size);  % object_points是世界系下所有角点的坐标。顺序和image的相同，但坐标系是左上角作为世界系原点。


% 检测图像中的棋盘格角点
[image_points, board_size] = detectCheckerboardPoints(chessboard_image);

% 估计相机1的外参
[R_w_cam, t_w_cam] = estimateWorldCameraPose(image_points, object_points, cameraParameters('IntrinsicMatrix', camera_K')); % image_poinst和obj_points[x,y]
t_w_cam = t_w_cam';

R_cam_w = R_w_cam';
t_cam_w = -R_w_cam' * t_w_cam;
% Projector的外参 (R2, T2)，由标定结果提供。
% T_p_w = T_p_c * T_c_w, --> t_p_w = R_p_c*t_w_c + t_p_c
R_pro_w = R_pro_cam * R_cam_w;
t_pro_w = R_pro_cam * t_cam_w + t_pro_cam;


% 将世界系下的棋盘格角点，投影到projector中。
object_points_camera2 = (R_pro_w * object_points' + t_pro_w)';

% 计算棋盘格角点在Camera2图像中的投影坐标
projected_points_camera2 = projector_K * object_points_camera2';

% 归一化为像素坐标
projected_points_camera2 = projected_points_camera2 ./ projected_points_camera2(3, :);

% 可视化投影坐标
figure;
plot(projected_points_camera2(1, :), projected_points_camera2(2, :), 'ro');
title('Projected Points on Camera 2');
xlabel('X [pixels]');
ylabel('Y [pixels]');
axis equal;
grid on;







%% 函数
% 根据当前摆放方式，生成世界系棋盘格布局。
function points = myGeneratePoints(board_size, square_size)
    corner_x_num = board_size(1)-1;
    corner_y_num = board_size(2)-1;
    points = zeros(corner_x_num * corner_y_num, 3);
    for i = 1:corner_y_num
        for j = 1:corner_x_num
               idx = (i-1)*corner_x_num + j;
               points(idx,1:2) = [(corner_x_num-j), (i-1)] * square_size;
        end
    end
end