function select_and_save_bbox_interactive()
% SELECT_AND_SAVE_BBOX_INTERACTIVE 交互式选点并保存 BoundingBox
% MATLAB R2022a
% 读取 .laz 点云，高比例降采样后交互式选点，记录多次点击位置，计算 Bounding Box，
% 并将坐标保存到文本文件 “bounding_box.txt”。
%
% 使用方法：
%   1. 将本文件保存为 select_and_save_bbox_interactive.m
%   2. 修改下面的 lazFilename 和 gridSize 为你的环境配置
%   3. 在 MATLAB Command Window 中运行：
%         select_and_save_bbox_interactive

% ------------- 用户配置 ----------------
lazFilename    = 'Cloud.laz';   % <-- 替换为你的 .laz 文件路径
gridSize       = 1.5;           % 降采样网格尺寸（米），可调大以加速可视化
outputFilename = 'bounding_box.txt';
% ----------------------------------------

% 1. 读取点云并降采样
if ~isfile(lazFilename)
    error('找不到指定的 .laz 文件：%s', lazFilename);
end
lasReader   = lasFileReader(lazFilename);
ptCloudFull = readPointCloud(lasReader);
ptCloudDisp = pcdownsample(ptCloudFull, 'gridAverage', gridSize);

% 2. 初始化位置列表
posList = [];

% 3. 显示点云并设置数据光标模式
fig = figure('Name', 'ROI 交互式选择', 'NumberTitle', 'off');
pcshow(ptCloudDisp);
title(sprintf('降采样 (%.1fm)，请点击多次边界点，点击 Done 完成', gridSize));
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal;

% 设置数据光标，并指定回调函数
dcm = datacursormode(fig);
set(dcm, 'Enable', 'on', 'SnapToDataVertex', 'on', 'DisplayStyle', 'datatip');
set(dcm, 'UpdateFcn', @storeCursor);

% 添加 Done 按钮
uicontrol('Style', 'pushbutton', 'String', 'Done', ...
    'Units', 'normalized', 'Position', [0.01,0.01,0.1,0.05], ...
    'FontSize', 11, 'Callback', @(~,~) uiresume(gcbf));

% 阻塞，等待用户点击 Done
uiwait(fig);

% 4. 关闭图窗
close(fig);

% 检查点击点数
if size(posList,1) < 2
    error('至少需要点击两个点以定义 Bounding Box');
end

% 5. 计算 Bounding Box
X_vals = posList(:,1);
Y_vals = posList(:,2);
Z_vals = posList(:,3);
xmin = min(X_vals); xmax = max(X_vals);
ymin = min(Y_vals); ymax = max(Y_vals);
zmin = min(Z_vals); zmax = max(Z_vals);

% 6. 保存到文本
fid = fopen(outputFilename, 'w');
fprintf(fid, 'X_min %.6f\n', xmin);
fprintf(fid, 'X_max %.6f\n', xmax);
fprintf(fid, 'Y_min %.6f\n', ymin);
fprintf(fid, 'Y_max %.6f\n', ymax);
fprintf(fid, 'Z_min %.6f\n', zmin);
fprintf(fid, 'Z_max %.6f\n', zmax);
fclose(fid);

fprintf('已保存 bounding box 到 %s：\n', outputFilename);
fprintf('  X_min=%.6f, X_max=%.6f\n', xmin, xmax);
fprintf('  Y_min=%.6f, Y_max=%.6f\n', ymin, ymax);
fprintf('  Z_min=%.6f, Z_max=%.6f\n', zmin, zmax);

% 回调函数：记录每次点击的位置
    function txt = storeCursor(~, event)
        pos = event.Position;
        posList(end+1, :) = pos;  %#ok<AGROW>
        txt = {sprintf('X: %.3f', pos(1)), ...
               sprintf('Y: %.3f', pos(2)), ...
               sprintf('Z: %.3f', pos(3))};
    end
end
