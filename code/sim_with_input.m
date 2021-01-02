function ax = sim_with_input(sys, tin, x0)
    if nargin == 2
        x0 = zeros(order(sys), 1);
    end            
    
    STRETCH = 1.1;
    set(gcf, 'Position', [12.356, 5.82, 15.58, 6.3923]);
    
    yyaxis left
    set(gca, 'YColor', 'black')

    inp = ones(size(tin));
    y = lsim(sys, inp, tin, x0);
    if sys.Ts ~= 0
        stairs(tin, y(:,1));
    else
        plot(tin, y(:,1));
    end
    xlim([0, max(tin)]);
    ylim([STRETCH*min(y(:,1)), STRETCH*max(y(:,1))]);
    ylabel('System output');
    
    yyaxis right
    set(gca, 'Position', [0.105, 0.143, 0.765, 0.693]);
    set(gca, 'YColor', 'black')
    if sys.Ts ~= 0
        stairs(tin, y(:,2));
    else
        plot(tin, y(:,2));
    end
    xlim([0, max(tin)]);
    ylim([STRETCH*min(y(:,2)), STRETCH*max(y(:,2))]);
    ylabel('Controller effort')
    xlabel('Time (s)');
    [~, max_input_idx] = max(abs(y(:,2))); hold on;
    scatter(tin(max_input_idx), y(max_input_idx, 2), 50, 's', 'filled', ...
                'handlevisibility', 'off');
    subtitle(sprintf('$u_\\mathrm{max}$ = %.4g', y(max_input_idx, 2)));
    legend('System output', 'Controller effort', 'Location', 'best')
end

