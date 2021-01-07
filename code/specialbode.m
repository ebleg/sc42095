function [mag, phase, wout, tile] = specialbode(sys, varargin)
    % Check SISO
    assert(all(size(sys) == [1 1]), 'System must be SISO for specialstep');
    
    % Parse inputs? 
    p = inputParser;
    validBoolean = @(str) any(contains({'on', 'off'}, str));
    validW = @(vec) isvector(vec) && isnumeric(vec) && all(diff(vec) > 0);

    addParameter(p, 'ShowGoodies', 'off', validBoolean);
    addParameter(p, 'PhaseVisible', 'on', validBoolean);
    addOptional(p, 'win', nan, validW);
    parse(p, varargin{:});
    
    win = p.Results.win;
    show_goodies = strcmp(p.Results.ShowGoodies, 'on');
    phase_visible = strcmp(p.Results.PhaseVisible, 'on');
    
    if ~isnan(win) % win specified
        [mag, phase, wout] = bode(sys, win);
    else
        [mag, phase, wout] = bode(sys);
    end
    
    % Only applies to this figure
%     set(0, 'defaultAxesMinorTickDir', 'in');
%     set(0, 'defaultAxesGridLineStyle', '-');
%     set(0, 'defaultAxesMinorGridLineStyle', '-');
    
    STRETCH = 0.1;
    
    [gain_m, phase_m, wcg, wcp] = margin(sys); gain_m = 20*log10(gain_m);
    
    fig = gcf;
    
    if phase_visible
        try check = strcmp(fig.Children.Type, 'tiledlayout');
        catch; check = false;
        end

        if check; tile = fig.Children;
        else; tile = tiledlayout(2, 1);
        end
    end

    % -------------------------- MAG ------------------------------
    
    if phase_visible; nexttile(1); end 
    
    ax = gca;

    % Get previous limits to avoid cropping of  previous plots
    ylim_prev = ylim; xlim_prev = xlim;
    color_idx = ax.ColorOrderIndex;
    colors = colororder();

    semilogx(wout, 20*log10(squeeze(mag)));
    hold on;
    ylabel('Magnitude (dB)');

    % Determine maximum distance in data
    spread = max(20*log10(mag)) - min(20*log10(mag));

    % Set limits accordingly
    ylim([min(ylim_prev(1), min(20*log10(mag)) - STRETCH*spread), ...
          max(ylim_prev(2), max(20*log10(mag)) + STRETCH*spread)]);
    xlim([min(xlim_prev(1), wout(1)), ...
       max(xlim_prev(2), wout(end))]);

    semilogx([wout(1), wout(end)], [0 0], 'Color', '#7f8c8d', ...
             'LineWidth', 1, 'HandleVisibility', 'off');

    ax.ColorOrderIndex = ax.ColorOrderIndex - 1;

    if show_goodies && isfinite(gain_m)
       gmtxt = text(ax, wcg, -gain_m, ...
                    sprintf('\\textbf{GM %.3g dB}', gain_m), ...
                    'Color', colors(color_idx,:), ...
                    'FontName', ax.FontName, ...
                    'FontWeight', 'bold');
        gmtxt.Units = 'characters';
        gmtxt.Position = gmtxt.Position + [5 -2 0];
        line([wcg, wcg], [0, -gain_m], ...
             'LineStyle', ':', 'Color', colors(color_idx,:), ...
             'HandleVisibility', 'off');
    end

    % ------------------------- PHASE -----------------------------
    if phase_visible
        nexttile(2);
        
        ax = gca;
        % Get previous limits to avoid cropping of  previous plots
        if ~isempty(ax.Children)
            ylim_prev = ylim;
            xlim_prev = xlim;
        else
            ylim_prev = [-180 -180]; xlim_prev = [0 0];
        end
        
        semilogx(wout, squeeze(phase));
        hold on; 
        ylabel('Phase (deg)');
        
        % Determine maximum distance in data
        spread = max(phase) - min(phase);

        % Set limits accordingly
        ylim([min(ylim_prev(1), min(phase) - STRETCH*spread), ...
              max(ylim_prev(2), max(phase) + STRETCH*spread)]);
        xlim([min(xlim_prev(1), wout(1)), ...
              max(xlim_prev(2), wout(end))]);

        semilogx([wout(1), wout(end)], [-180 -180], 'Color', '#7f8c8d', ...
                 'LineWidth', 1, 'HandleVisibility', 'off');

        ax.ColorOrderIndex = ax.ColorOrderIndex - 1;

        if show_goodies && isfinite(phase_m)
            pmtxt = text(ax, wcp, max([-180, -180+phase_m, max(phase)]), ...
                         sprintf(' \\textbf{PM %.3g deg}', phase_m), ...
                         'Color', colors(color_idx,:), ...
                         'FontName', ax.FontName, ...
                         'FontWeight', 'bold');
            pmtxt.Units = 'characters';
            pmtxt.Position = pmtxt.Position + [0 1 0];
            line([wcp, wcp], [-180, -180+phase_m], ...
                 'LineStyle', ':', 'Color', colors(color_idx,:), ...
                 'HandleVisibility', 'off');
        end
        
    end

    if phase_visible
        tile.Title.FontSize = 11.5;
        tile.XLabel.FontSize = 11; tile.YLabel.FontSize = 11; 
        tile.XLabel.Interpreter = 'latex';
        tile.YLabel.Interpreter = 'latex';
        tile.Title.Interpreter = 'latex';
        tile.Title.String = '\textbf{Bode plot}';
        xlabel(tile, 'Frequency (rad/s)');
    else
        title('\textbf{Bode plot}');
        tile = gca;
        xlabel('Frequency (rad/s)');
    end
end

