function run_advanced_mpc()
%% ========================================================================
%  ADVANCED MPC FOR PATH TRACKING (SINGLE-FILE IMPLEMENTATION)
% =========================================================================
%
% DESCRIPTION:
% This self-contained script simulates an autonomous vehicle tracking a 
% complex reference trajectory using a Linear Time-Varying Model Predictive
% Controller (LTV-MPC). The optimization is solved efficiently at each 
% step using a Quadratic Programming (QP) solver.
%
% REQUIREMENTS: MATLAB Optimization Toolbox (for 'quadprog')
%
% =========================================================================

    %% 1. Configuration and Setup
    % ---------------------------------------------------------------------
    clear; clc; close all;
    fprintf('Initializing Advanced MPC Simulation...\n');

    % Simulation Parameters
    sim_params.Ts = 0.1;
    sim_params.simulation_time = 20;
    sim_params.t = 0:sim_params.Ts:sim_params.simulation_time;

    % Vehicle Parameters
    vehicle_params.L = 2.5;
    vehicle_params.v = 5.0;
    vehicle_params.steer_limit = 0.5;

    % MPC Parameters
    mpc_params.N = 15;
    mpc_params.Q = diag([1.0, 0.1]);
    mpc_params.R = 0.05;
    mpc_params.P = 1.0 * mpc_params.Q;

    %% 2. Reference Trajectory Generation
    % ---------------------------------------------------------------------
    fprintf('Generating reference trajectory...\n');
    trajectory = generate_reference_trajectory();

    %% 3. Run MPC Simulation
    % ---------------------------------------------------------------------
    [state_log, control_log, error_log] = run_simulation();

    %% 4. Performance Analysis and Visualization
    % ---------------------------------------------------------------------
    fprintf('Plotting results...\n');
    plot_results(state_log, control_log, error_log);

%% ========================================================================
%  5. Nested Helper Functions
%  (These functions can access the parameters defined above)
% =========================================================================

    function traj_out = generate_reference_trajectory()
        path = [0.5 * sim_params.t; 0.2 * sin(0.3 * sim_params.t) + 0.1 * sin(0.5 * sim_params.t)];
        dx = gradient(path(1,:), sim_params.Ts);
        dy = gradient(path(2,:), sim_params.Ts);
        yaw = atan2(dy, dx);
        
        traj_out.path = path;
        traj_out.yaw = yaw;
    end

    % ---------------------------------------------------------------------

    function [states, controls, errors] = run_simulation()
        fprintf('Starting main simulation loop...\n');
        
        initial_state = [trajectory.path(1,1); trajectory.path(2,1); trajectory.yaw(1)];
        z_current = initial_state;
        
        num_steps = length(sim_params.t);
        states = zeros(3, num_steps);
        states(:, 1) = z_current;
        controls = zeros(1, num_steps - 1);
        errors = zeros(2, num_steps - 1);
        
        nonlinear_model = @(z, u) [
            z(1) + vehicle_params.v * cos(z(3)) * sim_params.Ts;
            z(2) + vehicle_params.v * sin(z(3)) * sim_params.Ts;
            z(3) + (vehicle_params.v / vehicle_params.L) * tan(u) * sim_params.Ts
        ];

        for k = 1:num_steps - 1
            [cte, epsi, ref_idx] = calculate_errors(z_current);
            
            ref_yaw = trajectory.yaw(ref_idx);
            Ad = [1,  sim_params.Ts * vehicle_params.v; 0, 1];
            Bd = [0; sim_params.Ts * vehicle_params.v / (vehicle_params.L * cos(ref_yaw)^2)];
            
            [H, f] = form_qp_matrices(Ad, Bd, [cte; epsi]);
            
            A_ineq = [eye(mpc_params.N); -eye(mpc_params.N)];
            b_ineq = [ones(mpc_params.N, 1) * vehicle_params.steer_limit; ones(mpc_params.N, 1) * vehicle_params.steer_limit];
            
            options = optimoptions('quadprog','Display','none');
            optimal_steer_sequence = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], [], options);
            
            u_optimal = optimal_steer_sequence(1);
            z_current = nonlinear_model(z_current, u_optimal);
            
            states(:, k + 1) = z_current;
            controls(k) = u_optimal;
            errors(:, k) = [cte; epsi];
            
            if mod(k, 20) == 0
                fprintf('Simulation step %d/%d\n', k, num_steps-1);
            end
        end
    end

    % ---------------------------------------------------------------------

    function [cte, epsi, ref_idx] = calculate_errors(z)
        dx = z(1) - trajectory.path(1, :);
        dy = z(2) - trajectory.path(2, :);
        [~, ref_idx] = min(sqrt(dx.^2 + dy.^2));
        
        path_angle = trajectory.yaw(ref_idx);
        error_vec = [z(1) - trajectory.path(1, ref_idx); z(2) - trajectory.path(2, ref_idx)];
        cte = -sin(path_angle) * error_vec(1) + cos(path_angle) * error_vec(2);
        
        epsi = z(3) - path_angle;
        epsi = atan2(sin(epsi), cos(epsi));
    end

    % ---------------------------------------------------------------------

    function [H, f] = form_qp_matrices(Ad, Bd, x0)
        A_bar = Ad;
        B_bar = zeros(size(Bd,1), mpc_params.N*size(Bd,2));
        B_bar(:,1:size(Bd,2)) = Bd;

        for i = 2:mpc_params.N
            A_bar = [A_bar; Ad^i];
            for j = 1:i
                B_bar((i-1)*size(Ad,1)+1:i*size(Ad,1), (j-1)*size(Bd,2)+1:j*size(Bd,2)) = Ad^(i-j)*Bd;
            end
        end

        Q_bar = kron(eye(mpc_params.N-1), mpc_params.Q);
        Q_bar = blkdiag(Q_bar, mpc_params.P);
        R_bar = kron(eye(mpc_params.N), mpc_params.R);
        
        H = B_bar' * Q_bar * B_bar + R_bar;
        f = (x0' * A_bar' * Q_bar * B_bar)';
        
        H = (H + H') / 2; % Enforce symmetry to handle numerical inaccuracies
    end

    % ---------------------------------------------------------------------

    function plot_results(states, controls, errors)
        figure('Name', 'MPC Professional Performance Dashboard', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);

        subplot(2, 2, 1);
        plot(trajectory.path(1, :), trajectory.path(2, :), 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');
        hold on;
        plot(states(1, :), states(2, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Vehicle Path');
        title('Path Tracking Performance'); xlabel('X Position [m]'); ylabel('Y Position [m]');
        legend('Location', 'northwest'); axis equal; grid on;

        subplot(2, 2, 2);
        plot(sim_params.t(1:end-1), controls, 'b-', 'LineWidth', 1.5);
        hold on;
        plot(sim_params.t, ones(size(sim_params.t)) * vehicle_params.steer_limit, 'k:', 'LineWidth', 1);
        plot(sim_params.t, -ones(size(sim_params.t)) * vehicle_params.steer_limit, 'k:', 'LineWidth', 1);
        title('Steering Input (Control Effort)'); xlabel('Time [s]'); ylabel('Steering Angle [rad]');
        ylim([-vehicle_params.steer_limit-0.1, vehicle_params.steer_limit+0.1]); grid on;

        subplot(2, 2, 3);
        plot(sim_params.t(1:end-1), errors(1,:), 'g-', 'LineWidth', 1.5);
        title('Cross-Track Error (CTE)'); xlabel('Time [s]'); ylabel('Error [m]'); grid on;

        subplot(2, 2, 4);
        plot(sim_params.t(1:end-1), rad2deg(errors(2,:)), 'm-', 'LineWidth', 1.5);
        title('Heading Error'); xlabel('Time [s]'); ylabel('Error [degrees]'); grid on;
    end

end