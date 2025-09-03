%% 图像读取

function Q_real = extract_boundary(image_path)

% img = imread(image_path);
img = imread('half_butterfly.png');
gray_img = rgb2gray(img);
bw_img = imbinarize(gray_img);
bw_img = imcomplement(bw_img); % 反相确保轨迹为白色

%% 边界点提取

boundaries = bwareaopen(bw_img,50);
boundaries = imfill(boundaries,'holes');
skeleton = bwmorph(boundaries,'skel', Inf);% 提取骨架，直至无法继续优化
skeleton = bwmorph(skeleton, 'spur', 10);% 修剪短于10个像素的分支
[endpoints_y, endpoints_x] = find(bwmorph(skeleton, 'endpoints'));
selectBoundary = bwtraceboundary(skeleton, [endpoints_y(1), endpoints_x(1)], 'W', 8, Inf, 'counterclockwise');

%% 同步CNC坐标

% 假设实际加工尺寸为100*100
scale_x = 100 / size(bw_img,2);
scale_y = 100 / size(bw_img,1);
Q_real = [selectBoundary(:,2)*scale_x, (size(bw_img,1)-selectBoundary(:,1))*scale_y];% y轴翻转适应CNC坐标系

%% 生成连续小线段

line_segment = [];
for i = 1:length(Q_real)-1
    segment = [Q_real(i,:),Q_real(i+1,:)];
    line_segment = [line_segment;segment];
end

%% 滑动滤波，避免随机抖动导致凹凸性判断崩溃
window_size = 5;
Q_real = movmean(Q_real,window_size);

figure;
plot(Q_real(:,1),Q_real(:,2),'r.-');
title('原始轨迹点集');
axis equal