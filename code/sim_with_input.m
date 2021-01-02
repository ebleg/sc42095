function ax = sim_with_input(sys, tin, x0)
    if nargin == 2
        x0 = zeros(order(sys), 1);
    end            
    
    STRETCH = 1.1;
    set(gcf, 'Position', get(gcf, 'Position').*[1 1 1.3 0.8]);
    
    inp = ones(size(tin));
    y = lsim(sys, inp, tin, x0);
    stairs(tin, y(:,1));
    xlim([0, max(tin)]);
    ylim([STRETCH*min(y(:,1)), STRETCH*max(y(:,1))]);
    ylabel('System output');
    
    yyaxis right
    set(gca, 'Position', get(gca, 'Position').*[1.05 1.1 0.9 0.9]);
    set(gca, 'YColor', 'black')
    stairs(tin, y(:,2));
    xlim([0, max(tin)]);
    ylim([STRETCH*min(y(:,2)), STRETCH*max(y(:,2))]);
    ylabel('Controller effort')
    
    [~, max_input_idx] = max(abs(y(:,2))); hold on;
    scatter(tin(max_input_idx), y(max_input_idx, 2), 50, 's', 'filled');
    subtitle(sprintf('$u_\\mathrm{max}$ = %.4g', y(max_input_idx, 2)));
    legend('System output', 'Controller effort', 'Location', 'best')
end

