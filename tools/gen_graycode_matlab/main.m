
clc;clear;close all;


% pattern是"用多少位"进行编码，pattern=4是4位格林码，可以编码最大分辨率2^4=16像素；
% 如果投影仪分辨率大于pattern编码后的数值，则相当于降采样。
pattern_number = 8;
proj_width = 1280;			% 真时的投影仪分辨率
proj_height = 720;
output_folder = "./output/";		% 输出路径

% 满足2^n覆盖整个投影仪的分辨率
proj_full_resolution_width =  2^ceil(log2(proj_width));
proj_full_resolution_height = 2^ceil(log2(proj_height));


% 生成pattern_number位格林码，存到矩阵。
graycode_matrix = generateGraycodeMatrix(pattern_number);
fprintf("Gray Code: \n");
graycode_matrix


% 计算分辨率下每个格林码条纹的最小宽度。软件中pattern_number的数值能够到99，远超实际的分辨率。超过分辨率以后，就按照pixel
graycode_width_minsize = proj_full_resolution_width / 2^pattern_number;
graycode_height_minsize = proj_full_resolution_height / 2^pattern_number;
% 如果pattern很大，则分辨率就是pixel
if graycode_width_minsize < 1
	graycode_width_minsize = 1;
end
if graycode_height_minsize < 1
	graycode_height_minsize = 1;
end


% 所有格林码栅格的图片存在这里
proj_patterns_cell = {};	

% 生成纯白、纯黑投影
proj_patterns_cell{1} = ones(proj_height, proj_width);	% 第一张纯白
proj_patterns_cell{2} = zeros(proj_height, proj_width);	% 第二张纯黑
proj_patterns_cell_index = 3;			% 后续不断增加

% 生成纵向栅格
for ver_idx = 1:min(pattern_number, ceil(log2(proj_width)))		% 如果pattern数量大于投影分辨率，则按照最大分辨率
	one_code = graycode_matrix(:,ver_idx);
	img = graycode2VerImage(one_code, graycode_width_minsize, proj_width, proj_height);
	% imshow(img);
	proj_patterns_cell{proj_patterns_cell_index} = img;
	proj_patterns_cell_index = proj_patterns_cell_index + 1;
	proj_patterns_cell{proj_patterns_cell_index} = ones(size(img)) - img;	% 反色
	proj_patterns_cell_index = proj_patterns_cell_index + 1;
end
% 生成横向栅格
for hor_idx = 1:min(pattern_number, ceil(log2(proj_height)))
	one_code = graycode_matrix(:,hor_idx);
	img = graycode2HorImage(one_code, graycode_height_minsize, proj_width, proj_height);
	proj_patterns_cell{proj_patterns_cell_index} = img;
	proj_patterns_cell_index = proj_patterns_cell_index + 1;
	proj_patterns_cell{proj_patterns_cell_index} = ones(size(img)) - img;	% 反色
	proj_patterns_cell_index = proj_patterns_cell_index + 1;
end
	
% view all grid pattern
figure("Name", "Patterns");
for i = 1:length(proj_patterns_cell)
	imshow(proj_patterns_cell{i});
	% waitforbuttonpress;
end

% 保存
for i = 1:length(proj_patterns_cell)
	img = proj_patterns_cell{i};
	index_str = sprintf('%03d', i);
	imwrite(img, output_folder + "own_" +  index_str + ".bmp");
end



%% 测试：与gui软件截图进行对比，作差比较是否一致。
% 
% clc;clear;close all;
% 
% % 定义文件夹路径
% folder1 = './output/'; % 替换为folder1的实际路径
% folder2 = './output/'; % 替换为folder2的实际路径
% 
% % 获取folder1中的所有图片文件
% files1 = dir(fullfile(folder1, 'own_*.bmp'));
% % 获取folder2中的所有图片文件
% files2 = dir(fullfile(folder2, 'Snipaste_*.png'));
% 
% % 初始化cell数组来保存图像
% images1 = cell(length(files1), 1);
% images2 = cell(length(files2), 1);
% 
% % 读取folder1中的所有图片
% for i = 1:length(files1)
% 	img = imread(fullfile(folder1, files1(i).name));
% 	if size(img, 3) == 3
% 		if size(img, 3) == 3
% 			img= rgb2gray(img);
% 		end
% 	end
%     images1{i} = img;
% 
% end
% 
% % 读取folder2中的所有图片
% for i = 1:length(files2)	
% 	img = imread(fullfile(folder2, files2(i).name));
% 	if size(img, 3) == 3
% 		if size(img, 3) == 3
% 			img = rgb2gray(img);
% 		end
% 	end
%     images2{i} = img;
% end
% 
% % 检查两文件夹中的图片数量是否相同
% if length(images1) ~= length(images2)
%     error('两文件夹中的图片数量不相等，无法进行两两图片做差。');
% end
% 
% % 创建一个figure来显示图像
% figure;
% 
% % 进行两两图片做差并绘制
% for i = 1:length(images1)
%     % 确保图像大小相同
%     if size(images1{i}) ~= size(images2{i})
%         error('第 %d 对图像大小不一致，无法进行做差。', i);
%     end
% 
%     % 计算两张图像的差分
%     diffImage = imabsdiff(images1{i}, images2{i});
% 
%     % 显示原始图像和差分图像
%     imshow(diffImage);
% 	waitforbuttonpress;
% end
% 



%% 函数

% 生成垂直方向条纹
function image = graycode2VerImage(graycode, graycode_scale, image_width, image_height)
	n = length(graycode);
	fullsize_width = n * graycode_scale;
	fullsize_image = zeros(image_height, fullsize_width);
    for i = 1:n
        if graycode(i) == 1
            fullsize_image(:, (i-1)*graycode_scale + 1:i*graycode_scale) = 1;  % 画白色（值为1）
        else
            fullsize_image(:, (i-1)*graycode_scale + 1:i*graycode_scale) = 0;  % 画黑色（值为0）
        end
	end
	% crop fullsize to target size:
	margin = (fullsize_width - image_width)/2;
	image = fullsize_image(:, margin+1:end-margin);
end
% 生成水平方向条纹
function image = graycode2HorImage(graycode, graycode_scale, image_width, image_height)
	n = length(graycode);
	fullsize_height = n * graycode_scale;
	fullsize_image = zeros(fullsize_height, image_width);
	for i = 1:n
        if graycode(i) == 1
            fullsize_image((i-1)*graycode_scale + 1:i*graycode_scale, :) = 1;  % 画白色（值为1）
        else
            fullsize_image((i-1)*graycode_scale + 1:i*graycode_scale, :) = 0;  % 画黑色（值为0）
        end
	end
	% crop fullsize to target size:
	margin = (fullsize_height - image_height)/2;
	image = fullsize_image(margin+1:end-margin, :);
end



function gray_code = generateGraycodeMatrix(N)
    if N == 1
        gray_code = [0; 1];
    else
        previous_gray_code = generateGraycodeMatrix(N - 1);
        gray_code = [zeros(2^(N-1), 1), previous_gray_code; ones(2^(N-1), 1), flipud(previous_gray_code)];
    end
end

% 
% % 生成N位格林码
% function gray_codes = generateGraycodeMatrix(n)
%     if n == 0
%         gray_codes = {'0'};
%     elseif n == 1
%         gray_codes = {'0', '1'};
%     else
%         prev_gray_codes = generateGraycodeMatrix(n - 1);
%         gray_codes = [strcat('0', prev_gray_codes), strcat('1', flip(prev_gray_codes))];
% 	end
% 
% 	% 获取数组的大小
%     num_rows = length(gray_code);  % 行数即为cell数组的长度
%     num_cols = length(gray_code{1});  % 列数即为每个cell的字符串长度
% 
%     % 初始化一个全零矩阵
%     gray_matrix = zeros(num_rows, num_cols);
% 
%     % 遍历每个cell，将其转换成矩阵中的对应行
%     for i = 1:num_rows
%         for j = 1:num_cols
%             % 将字符'0'或'1'转换成数值0或1
%             gray_matrix(i, j) = str2double(gray_code{i}(j));
%         end
% 	end
% end
% 
% % 将格林码
% function gray_matrix = gray_code_to_matrix(gray_code)
% 
% end
