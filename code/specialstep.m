function [y, tout] = specialstep(sys, varargin)
    % Good-looking step plot for SISO
    
    % Check SISO
    assert(all(size(sys) == [1 1]), 'System must be SISO for specialstep');
    
    p = inputParser;
    validBoolean = @(str) any(contains({'on', 'off'}, str));
    validTime = @(vec) isvector(vec) && isnumeric(vec) && all(diff(vec) > 0);

    addParameter(p, 'ShowGoodies', 'off', validBoolean);
    addOptional(p, 't', nan, validTime);
    parse(p, varargin{:});
    t = p.Results.t;
    show_goodies = strcmp(p.Results.ShowGoodies, 'on');
    
    % Parse input arguments
    if ~isnan(t)
        [y, tout] = step(sys, t);
    else
        [y, tout] = step(sys);
    end
    
    ax = gca;
    color_idx = ax.ColorOrderIndex;
    colors = colororder();
    
    % Get previous limits to avoid cropping of  previous plots
    if ~isempty(ax.Children)
        ylim_prev = ylim;
        xlim_prev = xlim;
    else
        ylim_prev = [0 0]; xlim_prev = [0 0];
    end
    
    % Plot the step response
    if sys.Ts == 0 % continuous
        plot(ax, tout, y);
    else % discrete
        stairs(ax, tout, y);
    end
    
    xlabel('Time (s)');
    
    % Fix x and y limits
    STRETCH = 1.1; % Stretch
    ylim([min(ylim_prev(1), STRETCH*min(y)), ...
          max(ylim_prev(2), STRETCH*max(y))]);
    xlim([min(xlim_prev(1), tout(1)), ...
          max(xlim_prev(2), tout(end))]);
  
    % Get step info
    info = stepinfo(sys, 'SettlingTimeThreshold', 0.01);
    
    % Plot step info
    steady_state_value = dcgain(sys);
    
    % Only show info if sys is stable & asked by user
    if isfinite(steady_state_value) && show_goodies
        % Steady-state value
        line([tout(1), tout(end)], [steady_state_value, steady_state_value], ...
             'LineStyle', ':', 'Color', colors(color_idx,:), ...
             'Handlevisibility', 'off');
        
        text(tout(end), steady_state_value, ...
              sprintf('\\textbf{Steady-state %.3g }\n', steady_state_value), ...
              'Color', colors(color_idx,:), ...
              'HorizontalAlignment', 'right', ...
              'FontName', ax.FontName, ...
              'FontWeight', 'bold');
        
        % Overshoot
        if info.Overshoot > 0
            text(info.PeakTime, info.Peak, ...
                 sprintf('\\textbf{%.2g\\%%}\n', info.Overshoot), ...
                 'HorizontalAlignment', 'center', ...
                 'Color', colors(color_idx,:), ...
                 'FontName', ax.FontName, ...
                 'FontWeight', 'bold');
        end
        
        % Settling time
        sttxt = text(info.SettlingTime, steady_state_value, ...
                     sprintf('\\textbf{Settling time %.3gs}', info.SettlingTime), ...
                     'HorizontalAlignment', 'left', ...
                     'Color', colors(color_idx,:), ...
                     'FontName', ax.FontName, ...
                     'FontWeight', 'bold');
        sttxt.Units = 'characters';
        sttxt.Position = sttxt.Position - [0 1 0];
    end
    
end

