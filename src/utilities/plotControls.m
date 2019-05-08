function plotStates(N_gates, solution, Quad)
    f = figure('DefaultAxesFontSize', 16);
    f.Name = 'Controls';
    subplot(2,2,1)
    title('U1')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).control(:,1), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('U1')

    subplot(2,2,2)
    title('U2')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).control(:,2), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('U2')

    subplot(2,2,3)
    title('U3')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).control(:,3), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('U3')

    subplot(2,2,4)
    title('U4')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).control(:,4), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('U4')
end
