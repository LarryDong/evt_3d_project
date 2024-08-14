clc;clear;close all;

%% 1、这部分参数是计算投影仪等效分辨率的，如果不改安装和投影仪的pattern数，这部分不用修改。
camera_width = 720;
camera_height = 1280;

projector_width_full = 720;     % projector is rotated
projector_height_full = 1280;
pattern = 6;

projector_width_grid_size = 2 ^ (ceil(log2(projector_width_full)) - pattern);
projector_height_grid_size =  2 ^ (ceil(log2(projector_height_full)) - pattern);
projector_ds_resolution_x = projector_width_full / projector_width_grid_size;
projector_ds_resolution_y = projector_height_full / projector_height_grid_size;



%% 2、Calibration data from calib-gui software output. 这部分直接复制粘贴gui软件标定得出的参数，calibration.m文件
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

%% 3、根据2的部分，计算一些参数转化。
% Convert rotation vector to rotation matrix
T = T*1e-3;  % mm转m
R_pro_cam = rotationVectorToMatrix(om);
R_pro_cam = R_pro_cam';       % 这里要加1个转置，是因为matlab2022起坐标系变化由右乘变成了左乘，因此推测之前gui软件生成的旋转向量也是右乘形式。
t_pro_cam = T';
R_cam_pro = R_pro_cam';
t_cam_pro = -R_pro_cam'*t_pro_cam;  % T应该是：相机在投影仪坐标系下的坐标。R*x_camera + T = x_projectgor

% deal with resolution downsampling. 恢复projector的原始分辨率。
Dx = projector_width_grid_size;
Dy = projector_height_grid_size;
fc_right = [fc_right(1)*Dx, fc_right(2)*Dy];
cc_right = [cc_right(1)*Dx, cc_right(2)*Dy];
kc_right = [kc_right(1)/(Dx^2), kc_right(2)/(Dy^2), kc_right(3)/Dx, kc_right(4)/Dy, kc_right(5)];

% Camera matrices 构建相机、投影仪的内参矩阵，方便后续计算
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


%% Stereo rectification test 双目矫正测试。 直接矫正双目图像
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
% stereoParams = stereoParameters(cameraParams1, cameraParams2, R_cam_pro, t_cam_pro);    % 双目标定，第3/4参数为2相对于1的姿态，即1变化为2的姿态。
% [J1, J2] = rectifyStereoImages(image1, image2, stereoParams,  'OutputView', 'full');
% 
% % Display rectified images
% figure;
% subplot(1,2,1), imshow(J1), title('Camera-Rectified');
% subplot(1,2,2), imshow(J2), title('Projector-Rectified');


%% 4、测试相机视角变换结果
% 加载相机拍摄的图像
chessboard_image = imread('001 - share/merge/graycode_00.png');
chessboard_image = undistortImage(chessboard_image, camera_Intrinsics); 
imshow(chessboard_image)

% 棋盘格的基本参数
square_size = 0.0235; % 每个方块的边长，单位为米
num_corners_x = 6; % 棋盘格内横向角点数
num_corners_y = 9; % 棋盘格内纵向角点数
board_size = [num_corners_x+1, num_corners_y+1];

% 棋盘格3D世界坐标系下的角点坐标
object_points = myGeneratePoints(board_size, square_size);  % object_points是世界系下所有角点的坐标。顺序和image的相同，但坐标系是左上角作为世界系原点。

% 检测图像中的棋盘格角点
[image_points, board_size] = detectCheckerboardPoints(chessboard_image);

% 估计相机1的外参
% [R_w_cam, t_w_cam] = estimateWorldCameraPose(image_points, object_points, cameraParameters('IntrinsicMatrix', camera_K')); % image_poinst和obj_points[x,y]
% 注意，不能用这个函数！Starting in R2022b, most Computer Vision Toolbox™ functions create and perform geometric transformations using the premultiply convention. However, the estimateWorldCameraPose function uses the postmultiply convention. 
% 采用estworldpose是正确的坐标系变化左乘。

worldPose = estworldpose(image_points, object_points, camera_Intrinsics);
R_w_cam = worldPose.R;
t_w_cam = worldPose.Translation;
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
figure("Name", "Chessboard in Projector");
plot(projected_points_camera2(1, :), projected_points_camera2(2, :), 'ro');
title('Projected Points on Camera 2');
xlabel('X [pixels]');
ylabel('Y [pixels]');
axis equal;
grid on;


%% 显示系统的3D视角
figure;
hold on;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');

% 绘制所有角点（红色圆球）
ball_size = 23.5 * 1e-3 / 2 / 2;
for i = 1:size(object_points, 1)
    [X, Y, Z] = sphere(20);  % 生成一个球面
    X = X * ball_size + object_points(i, 1);  % 缩放并平移
    Y = Y * ball_size + object_points(i, 2);
    Z = Z * ball_size + object_points(i, 3);
    surf(X, Y, Z, 'FaceColor', 'r', 'EdgeColor', 'none');  % 绘制球面
end
% draw center.
[X, Y, Z] = sphere(20);
X = X * ball_size;
Y = Y * ball_size;
Z = Z * ball_size;
surf(X, Y, Z, 'FaceColor', 'g', 'EdgeColor', 'none');  % 绘制球面

R1 = R_cam_w';
R2 = R_pro_w';
t1 = -R_cam_w' * t_cam_w;
t2 = -R_pro_w' * t_pro_w;
camera_shape_scale = 0.03;

% X 方向长度为 1，Y 方向长度为 2
camera_body = [-1  1  1 -1 -1; 
              -1 -1 2 2 -1;    % Y 方向长度是 X 方向的 2 倍
              -1  -1  -1  -1  -1] * camera_shape_scale / 2;

% 定义相机朝向的形状（表示相机的前面）
camera_front = [-1  1  1 -1 -1;
               -1 -1 2 2 -1;    % Y 方向长度是 X 方向的 2 倍
               1  1  1  1  1] * camera_shape_scale;

% 定义相机坐标轴
axis_length = 2 * camera_shape_scale; % 坐标轴长度
x_axis = [0, axis_length; 0, 0; 0, 0];
y_axis = [0, 0; 0, axis_length; 0, 0];
z_axis = [0, 0; 0, 0; 0, axis_length];

% 绘制相机1及其坐标轴
camera1_body = R1 * camera_body + t1;
camera1_front = R1 * camera_front + t1;
fill3(camera1_body(1,:), camera1_body(2,:), camera1_body(3,:), 'b', 'FaceAlpha', 0.8);
fill3(camera1_front(1,:), camera1_front(2,:), camera1_front(3,:), 'b', 'FaceAlpha', 0.2);
line([camera1_body(1,1), camera1_front(1,1)], ...
     [camera1_body(2,1), camera1_front(2,1)], ...
     [camera1_body(3,1), camera1_front(3,1)], 'Color', 'b');
line([camera1_body(1,2), camera1_front(1,2)], ...
     [camera1_body(2,2), camera1_front(2,2)], ...
     [camera1_body(3,2), camera1_front(3,2)], 'Color', 'b');
line([camera1_body(1,3), camera1_front(1,3)], ...
     [camera1_body(2,3), camera1_front(2,3)], ...
     [camera1_body(3,3), camera1_front(3,3)], 'Color', 'b');
line([camera1_body(1,4), camera1_front(1,4)], ...
     [camera1_body(2,4), camera1_front(2,4)], ...
     [camera1_body(3,4), camera1_front(3,4)], 'Color', 'b');
text(t1(1), t1(2), t1(3), 'Camera', 'FontSize', 12, 'Color', 'b');

% 绘制相机1的坐标轴
x_axis_cam1 = R1 * x_axis + t1;
y_axis_cam1 = R1 * y_axis + t1;
z_axis_cam1 = R1 * z_axis + t1;
line(x_axis_cam1(1,:), x_axis_cam1(2,:), x_axis_cam1(3,:), 'Color', 'r', 'LineWidth', 2);
line(y_axis_cam1(1,:), y_axis_cam1(2,:), y_axis_cam1(3,:), 'Color', 'g', 'LineWidth', 2);
line(z_axis_cam1(1,:), z_axis_cam1(2,:), z_axis_cam1(3,:), 'Color', 'b', 'LineWidth', 2);

% 绘制相机2及其坐标轴
camera2_body = R2 * camera_body + t2;
camera2_front = R2 * camera_front + t2;
fill3(camera2_body(1,:), camera2_body(2,:), camera2_body(3,:), 'g', 'FaceAlpha', 0.8);
fill3(camera2_front(1,:), camera2_front(2,:), camera2_front(3,:), 'g', 'FaceAlpha', 0.2);
line([camera2_body(1,1), camera2_front(1,1)], ...
     [camera2_body(2,1), camera2_front(2,1)], ...
     [camera2_body(3,1), camera2_front(3,1)], 'Color', 'g');
line([camera2_body(1,2), camera2_front(1,2)], ...
     [camera2_body(2,2), camera2_front(2,2)], ...
     [camera2_body(3,2), camera2_front(3,2)], 'Color', 'g');
line([camera2_body(1,3), camera2_front(1,3)], ...
     [camera2_body(2,3), camera2_front(2,3)], ...
     [camera2_body(3,3), camera2_front(3,3)], 'Color', 'g');
line([camera2_body(1,4), camera2_front(1,4)], ...
     [camera2_body(2,4), camera2_front(2,4)], ...
     [camera2_body(3,4), camera2_front(3,4)], 'Color', 'g');
text(t2(1), t2(2), t2(3), 'Projector', 'FontSize', 12, 'Color', 'g');

% 绘制相机2的坐标轴
x_axis_cam2 = R2 * x_axis + t2;
y_axis_cam2 = R2 * y_axis + t2;
z_axis_cam2 = R2 * z_axis + t2;
line(x_axis_cam2(1,:), x_axis_cam2(2,:), x_axis_cam2(3,:), 'Color', 'r', 'LineWidth', 2);
line(y_axis_cam2(1,:), y_axis_cam2(2,:), y_axis_cam2(3,:), 'Color', 'g', 'LineWidth', 2);
line(z_axis_cam2(1,:), z_axis_cam2(2,:), z_axis_cam2(3,:), 'Color', 'b', 'LineWidth', 2);

% 设置视角和显示
view(3);
axis('equal');
% xlim([-0.1, 0.2]);
% ylim([-0.1, 0.3]);
% zlim([-0.6, 0.1]);
hold off;





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