%% ── Gráficas Individuales de Cinemática y Dinámica ───────────────────────
fprintf('Generando gráficas individuales de alta calidad...\n');

% Definir una paleta de colores profesional
colores = lines(4);

% =========================================================================
% FIGURA 1: Torques Individuales por Eje (Subplots 2x2)
% =========================================================================
fig_tau = figure('Name', 'Torques por Eje', 'Color', 'white', ...
    'Units', 'centimeters', 'Position', [2 2 22 16]);

titulos_tau = {'\tau_1 (Columna Base)', '\tau_2 (Eslabón Inclinado)', ...
    '\tau_3 (Link Arm)', '\tau_4 (Antebrazo)'};

for i = 1:4
    subplot(2, 2, i);
    plot(t_s, tau_traj(i,:), 'LineWidth', 1.8, 'Color', colores(i,:));
    hold on;

    % Líneas verticales para marcar las fases (Home -> Pick -> Place -> Home)
    xline(1, '--k', 'Pick', 'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom'); 
    xline(2, '--k', 'Place', 'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom');

    title(titulos_tau{i}, 'FontName', 'Times New Roman', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tiempo (s)', 'FontName', 'Times New Roman', 'FontSize', 10);
    ylabel('Torque (N\cdotm)', 'FontName', 'Times New Roman', 'FontSize', 10);
    grid on;
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 9);
end

sgtitle('Dinámica: Requerimiento de Torque Individual por Eje', ...
    'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');

exportgraphics(fig_tau, 'kr300pa_torques_individuales.pdf', ...
    'ContentType', 'vector', 'BackgroundColor', 'white');

% =========================================================================
% FIGURA 2: Perfiles Cinemáticos (q, dq, ddq)
% =========================================================================
fig_cin = figure('Name', 'Perfiles Cinemáticos', 'Color', 'white', ...
    'Units', 'centimeters', 'Position', [5 2 20 20]);

% 1. Posición Articular
subplot(3, 1, 1);
hold on;
for i=1:4, plot(t_s, q_traj(i,:), 'LineWidth', 1.5, 'Color', colores(i,:)); end
xline(1,'--k'); xline(2,'--k');
title('Posición Articular (\mathbf{q})', 'FontName', 'Times New Roman', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Ángulo (deg)', 'FontName', 'Times New Roman', 'FontSize', 10);
legend({'A_1', 'A_2', 'A_3', 'A_4'}, 'Location', 'best', 'NumColumns', 4, 'FontName', 'Times New Roman');
grid on; set(gca, 'FontName', 'Times New Roman', 'FontSize', 9);

% 2. Velocidad Articular
subplot(3, 1, 2);
hold on;
% Convertimos rad/s a deg/s para que la lectura sea intuitiva
for i=1:4, plot(t_s, rad2deg(dq_traj(i,:)), 'LineWidth', 1.5, 'Color', colores(i,:)); end
xline(1,'--k'); xline(2,'--k');
title('Velocidad Articular (\dot{\mathbf{q}})', 'FontName', 'Times New Roman', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Velocidad (deg/s)', 'FontName', 'Times New Roman', 'FontSize', 10);
grid on; set(gca, 'FontName', 'Times New Roman', 'FontSize', 9);

% 3. Aceleración Articular
subplot(3, 1, 3);
hold on;
% Convertimos rad/s^2 a deg/s^2
for i=1:4, plot(t_s, rad2deg(ddq_traj(i,:)), 'LineWidth', 1.5, 'Color', colores(i,:)); end
xline(1,'--k'); xline(2,'--k');
title('Aceleración Articular (\ddot{\mathbf{q}})', 'FontName', 'Times New Roman', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Tiempo (s)', 'FontName', 'Times New Roman', 'FontSize', 10);
ylabel('Aceleración (deg/s^2)', 'FontName', 'Times New Roman', 'FontSize', 10);
grid on; set(gca, 'FontName', 'Times New Roman', 'FontSize', 9);

sgtitle('Cinemática: Trayectoria Pick-and-Place', ...
    'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');

exportgraphics(fig_cin, 'kr300pa_perfiles_cinematicos.pdf', ...
    'ContentType', 'vector', 'BackgroundColor', 'white');

fprintf('Gráficas generadas exitosamente.\n');