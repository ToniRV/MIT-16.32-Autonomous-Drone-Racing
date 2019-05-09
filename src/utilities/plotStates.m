function plotStates(N_gates, solution, Quad)
    f = figure('DefaultAxesFontSize', 16);
    f.Name = 'MultiPhase';
    subplot(4,3,1)
    title('X Numerical')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).state(:,1), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('x')

    subplot(4,3,2)
    title('Y Numerical')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).state(:,2), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('y')

    subplot(4,3,3)
    title('Z Numerical')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).state(:,3), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('z')

    subplot(4,3,4)
    title('Velocity X Numerical')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).state(:,4), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('Vel X')

    subplot(4,3,5)
    title('Velocity Y Numerical')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).state(:,5), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('Vel Y')

    subplot(4,3,6)
    title('Velocity Z Numerical')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).state(:,6), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('Vel Z')

    subplot(4,3,7)
    title('Phi Numerical')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).state(:,7), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('Phi'), ylim([Quad.phi_min Quad.phi_max])

    subplot(4,3,8)
    title('Theta Numerical')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).state(:,8), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('Theta'), ylim([Quad.theta_min Quad.theta_max])

    subplot(4,3,9)
    title('Psi Numerical')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).state(:,9), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('Psi'), ylim([Quad.psi_min Quad.psi_max])

    subplot(4,3,10)
    title('p Numerical')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).state(:,10), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('p'), ylim([Quad.p_min Quad.p_max]) 

    subplot(4,3,11)
    title('q Numerical')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).state(:,11), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('q'), ylim([Quad.q_min Quad.q_max])

    subplot(4,3,12)
    title('r Numerical')
    hold on
    for p = 1:N_gates
      plot(solution.phase(p).time, solution.phase(p).state(:,12), 'LineWidth', 2)
    end
    xlabel('t'), ylabel('r'), ylim([Quad.r_min Quad.r_max])
    
    hold off;
end

