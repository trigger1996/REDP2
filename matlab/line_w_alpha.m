% 创建一个新的figure窗口
figure;
% X和Y数据点
x = [0, 1];
y = [0, 1];
% 对应的颜色数据 (红色)
cdata = [1 0 0;
1 0 0];
% 对应的透明度数据
alphaData = [0; 1];
% 使用patch函数
h = patch('XData', x, 'YData', y, 'FaceVertexCData', cdata, 'FaceColor', 'none', 'EdgeColor', 'interp', 'LineWidth', 2);
% 设置透明度数据
set(h, 'FaceVertexAlphaData', alphaData, 'EdgeAlpha', 'interp');