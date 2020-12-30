function [] = specialpzmap(sys)
    ax = gca;
    poles = pole(sys);
    zeros = zero(sys);
    
    xlim_prev = xlim;
    ylim_prev = ylim;
    
    color_idx = ax.ColorOrderIndex;
    colors = colororder();
    
    plot(real(poles), imag(poles), 'x', 'Color', colors(color_idx,:), ...
                'LineWidth', 2, 'MarkerSize', 10); hold on;
    plot(real(zeros), imag(zeros), 'o', 'Color', colors(color_idx,:), ...
                'LineWidth', 2, 'MarkerSize', 7);
    
    STRETCH = 1.1;        
    maxdim = STRETCH*max(abs([real(poles); real(zeros); ...
                              imag(poles); imag(zeros)]));
            
    xlabel('Re', 'Fontsize', 10); ylabel('Im', 'Fontsize', 10);
    set(ax, 'XAxisLocation', 'origin');
    set(ax, 'YAxisLocation', 'origin');
    set(ax, 'XMinorTick', 'on');
    set(ax, 'YMinorTick', 'on');
    set(ax, 'Position', [0.05, 0.05, 0.9, 0.8]);
    
    xlim([min(xlim_prev(1), -maxdim), max(xlim_prev(2), maxdim)]);
    ylim([min(ylim_prev(1), -maxdim), max(ylim_prev(2), maxdim)]);

    axis square
    
    [wn, zeta, pole_list] = damp(sys);
        
    infostring = '';
    
    for i = 1:numel(pole_list)
       infostring = [infostring, ...
            sprintf('\\underline{$%.3g %+.3gj$}\n$\\omega$ = %.3g rad/s\n$\\zeta$ = %.2g\n\n', ...
                     round(real(pole_list(i)), 3), ...
                     round(imag(pole_list(i)), 3), ...
                     round(wn(i), 2), ...
                     round(zeta(i), 3))];
    end
    annotation('textbox', [0.78 0 0.2 0.8], ...
               'String', infostring, ...
               'EdgeColor','none', 'interpreter', 'tex', ...
               'HorizontalAlignment', 'right', ...
               'VerticalAlignment', 'top', ...
               'FontSize', 10, ...
               'Interpreter', 'latex');
           
    title('\textbf{Pole}-\textbf{zero map}');
end

