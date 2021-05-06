function extremities(x, y, z)
    ax = gca;
    plot3(x(1), y(1), z(1), 'o', 'Color', [0.6510    0.6510    0.6510], ...
        'MarkerSize', 10, 'MarkerFaceColor', [0.6510    0.6510    0.6510])
    text(x(1)+(ax.XLim(2)-ax.XLim(1))*0.01, ...
                y(1)+(ax.YLim(2)-ax.YLim(1))*0.01, ...
                z(1)+(ax.ZLim(2)-ax.ZLim(1))*0.01, ...
                "START", 'Color', 'k')

    plot3(x(end), y(end), z(end), 'o', 'Color', 'k', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'black')
    text(x(end)+(ax.XLim(2)-ax.XLim(1))*0.01, ...
                y(end)+(ax.YLim(2)-ax.YLim(1))*0.01, ...
                z(end)+(ax.ZLim(2)-ax.ZLim(1))*0.01, ...
                "END", 'Color', 'k')
end