
clc;clear;close all;

%% basic parameters
Camera_Width = 1280;	% 相机分辨率
Camera_Height = 720;
Binary_Threshold = 2;		% 多少个事件点认为有效数据？
Accumualted_Delta_T = 400*1e3;	% 每张图片对应的事件的时间长度，单位：us

%% File input and output
dat_file = "E:\codeGit\evt_3d_project\checkboard.dat";	% 输入文件
output_folder = "E:\codeGit\evt_3d_project\output\";			% 输出图片路径

fprintf("--> Loading file: %s \n", dat_file);
data = load_cd_events(dat_file);
fprintf("<-- Done. Data length: %d \n", size(data.ts,1));

%%
% 提取文件
ts_list = data.ts;
x_list = data.x;
y_list = data.y;
p_list = data.p;

evts = [data.ts, data.x, data.y, data.p];
N = size(evts, 1);

slice_idx = 1;
next_ts = slice_idx * Accumualted_Delta_T;
curr_ts = (slice_idx-1) * Accumualted_Delta_T + 1;
last_event_idx = 1;

figure("Name", "Visualization");
on_image = zeros()
for idx = 1:N
	if(evts(idx, 1) > next_ts)
		extractted_evts = evts(last_event_idx :idx, :);
		
		% 获取累计图
		accumulated_image = zeros(Camera_Height, Camera_Width);
		on_image = zeros(Camera_Height, Camera_Width);
		off_image = zeros(Camera_Height, Camera_Width);
		for jj = 1:size(extractted_evts, 1)
			e = extractted_evts(jj, :);
			accumulated_image(e(3)+1, e(2)+1) = accumulated_image(e(3)+1, e(2)+1) + 1;
			if(e(4)==1)
				on_image(e(3)+1, e(2)+1) = on_image(e(3)+1, e(2)+1) + 1;
			else
				off_image(e(3)+1, e(2)+1) = off_image(e(3)+1, e(2)+1) + 1;
			end
		end
		accumulated_image_show = uint8(on_image*40);		% *10 增强对比度
		subplot(2,2,1);
		imshow(accumulated_image_show);
		% title("强度图");
		title("on");

		subplot(2,2,4);
		off_image = uint8(off_image*40);		% *10 增强对比度
		imshow(off_image);
		title("off");

		% 二值化
		binary_image = imbinarize(accumulated_image, Binary_Threshold);
		subplot(2,2,2);
		imshow(binary_image);
		title("二值化图");

		% 开闭运算，避免噪点。先闭后开，消除噪声。
		se = strel('disk', 3);
		binary_image = imclose(binary_image, se);
		dst_img = imopen(binary_image, se);
		subplot(2,2,3);
		imshow(dst_img);
		title("开闭运算结果");
		
		% 按键保存检测
		waitforbuttonpress;
    	% 检查按下的键
    	key = get(gcf, 'CurrentCharacter');
    	
		% 如果按下的是 's' 键，则保存图像
		if key == 's'
			output_filename = output_folder + last_event_idx + ".bmp";
			imwrite(dst_img, output_filename);
			fprintf("Image saved: %s \n", output_filename);
		end
		% fprintf("Time duration: %d ~ %d \n", curr_ts, next_ts);
		% fprintf("Event slice length: %d \n", size(extractted_evts, 1));
		
		curr_ts = (slice_idx-1) * Accumualted_Delta_T + 1;
		slice_idx = slice_idx + 1;
		next_ts = slice_idx * Accumualted_Delta_T;
		last_event_idx = idx;
	end
end