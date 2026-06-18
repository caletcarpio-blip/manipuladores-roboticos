% =========================================================================
% TRAYECTORIA Y DINÁMICA REALISTA - KUKA KR 300-2 PA
% Incluye: Perfiles de velocidad suaves (5to orden), Fricción Articular y
%          gráficas con intérprete de LaTeX corregido.
% =========================================================================
clc; clear; close all;

fprintf('1. Inicializando parámetros del KUKA KR 300-2 PA...\n');
% Parámetros geométricos [m] y dinámicos [kg]
d1 = 0.846; a2 = 0.560; a3 = 1.314; a4 = 1.080; d5 = 0.321;
m_dyn = [903, 602, 430, 151]; % Distribución de la masa de 2150 kg
g_vec = [0; 0; -9.81];

% Parámetros de fricción estimada (Viscosa y Coulomb)
Fv = diag([15, 25, 20, 10]); % N*m*s/rad
Fc = [10; 15; 15; 5];        % N*m

% Vectores DH para evaluación geométrica
al_vis = [pi/2, 0, 0, 0];
a_vis  = [0, a2, a3, a4];
d_vis  = [d1, 0, 0, 0];

fprintf('2. Generando trayectoria Pick-and-Place (Curvas en S)...\n');
q_home  = [  0,   0,   0,   0];
q_pick  = [ 30, -60,  90, -30];
q_place = [-45, -50,  80, -30];

npts = 60; 
tau_norm = linspace(0, 1, npts)';

% Polinomio de 5to orden para suavizar el movimiento (elimina aceleración infinita)
s = 10*tau_norm.^3 - 15*tau_norm.^4 + 6*tau_norm.^5;

% Interpolación suave entre los puntos
t1 = (1-s).*q_home  + s.*q_pick;
t2 = (1-s).*q_pick  + s.*q_place;
t3 = (1-s).*q_place + s.*q_home;

q_traj = [t1; t2; t3]'; % Matriz 4 x 180 puntos
ntot = size(q_traj, 2);
t_s = linspace(0, 3, ntot); % Vector de tiempo de 0 a 3s
dt = t_s(2) - t_s(1);

q_traj_rad = deg2rad(q_traj);
dq_traj = zeros(size(q_traj_rad));
ddq_traj = zeros(size(q_traj_rad));

% Derivación numérica para obtener velocidad y aceleración
for i = 1:4
    dq_traj(i,:) = gradient(q_traj_rad(i,:), dt);
    ddq_traj(i,:) = gradient(dq_traj(i,:), dt);
end

fprintf('3. Calculando dinámica paso a paso (Lagrange + Fricción)...\n');
tau_traj = zeros(4, ntot);

% Mostrar progreso en consola
reverseStr = '';
for k_idx = 1:ntot
    q_k = q_traj_rad(:, k_idx) + deg2rad([0; 90; 0; 0]); % Offset DH
    dq_k = dq_traj(:, k_idx);
    ddq_k = ddq_traj(:, k_idx);
    
    % --- Evaluar Matrices M y G ---
    T_k = eye(4); p_com_k = zeros(3,4);
    for i=1:4
        ct=cos(q_k(i)); st=sin(q_k(i)); ca=cos(al_vis(i)); sa=sin(al_vis(i));
        Ai=[ct,-st*ca,st*sa,a_vis(i)*ct; st,ct*ca,-ct*sa,a_vis(i)*st; 0,sa,ca,d_vis(i); 0,0,0,1];
        T_half=T_k*[eye(3),[0.5*a_vis(i)*ct;0.5*a_vis(i)*st;0.5*d_vis(i)];0 0 0 1];
        p_com_k(:,i)=T_half(1:3,4);
        T_k=T_k*Ai;
    end
    
    M_k = zeros(4,4); G_k = zeros(4,1);
    for k=1:4
        Jv=zeros(3,4); T_a=eye(4); z_p=[0;0;1]; p_p=[0;0;0];
        for i=1:4
            if i<=k, Jv(:,i)=cross(z_p, p_com_k(:,k)-p_p); end
            ct=cos(q_k(i)); st=sin(q_k(i)); ca=cos(al_vis(i)); sa=sin(al_vis(i));
            T_a=T_a*[ct,-st*ca,st*sa,a_vis(i)*ct; st,ct*ca,-ct*sa,a_vis(i)*st; 0,sa,ca,d_vis(i); 0,0,0,1];
            z_p=T_a(1:3,3); p_p=T_a(1:3,4);
        end
        M_k = M_k + m_dyn(k)*(Jv'*Jv);
        G_k = G_k - m_dyn(k)*(Jv'*g_vec);
    end
    
    % --- Evaluar Coriolis Numéricamente ---
    C_dq = zeros(4,1);
    epsilon = 1e-5;
    for i = 1:4
        q_pert = q_k; q_pert(i) = q_pert(i) + epsilon;
        T_p = eye(4); p_com_p = zeros(3,4);
        for j=1:4
            ct=cos(q_pert(j)); st=sin(q_pert(j)); ca=cos(al_vis(j)); sa=sin(al_vis(j));
            Ai=[ct,-st*ca,st*sa,a_vis(j)*ct; st,ct*ca,-ct*sa,a_vis(j)*st; 0,sa,ca,d_vis(j); 0,0,0,1];
            T_half=T_p*[eye(3),[0.5*a_vis(j)*ct;0.5*a_vis(j)*st;0.5*d_vis(j)];0 0 0 1];
            p_com_p(:,j)=T_half(1:3,4);
            T_p=T_p*Ai;
        end
        M_pert = zeros(4,4);
        for k=1:4
            Jv=zeros(3,4); T_a=eye(4); z_p=[0;0;1]; p_p=[0;0;0];
            for j=1:4
                if j<=k, Jv(:,j)=cross(z_p, p_com_p(:,k)-p_p); end
                ct=cos(q_pert(j)); st=sin(q_pert(j)); ca=cos(al_vis(j)); sa=sin(al_vis(j));
                T_a=T_a*[ct,-st*ca,st*sa,a_vis(j)*ct; st,ct*ca,-ct*sa,a_vis(j)*st; 0,sa,ca,d_vis(j); 0,0,0,1];
                z_p=T_a(1:3,3); p_p=T_a(1:3,4);
            end
            M_pert = M_pert + m_dyn(k)*(Jv'*Jv);
        end
        dM_dqi = (M_pert - M_k) / epsilon;
        C_dq = C_dq + (dM_dqi * dq_k - 0.5 * dq_k' * dM_dqi * dq_k) * dq_k(i);
    end

    % --- Agregar modelo de Fricción ---
    tau_friccion = Fv * dq_k + Fc .* sign(dq_k);

    % --- Ecuación de Movimiento Completa ---
    tau_traj(:, k_idx) = M_k * ddq_k + C_dq + G_k + tau_friccion;
    
    % Actualizar consola
    msg = sprintf('Progreso: %d / %d...', k_idx, ntot);
    fprintf([reverseStr, msg]);
    reverseStr = repmat(sprintf('\b'), 1, length(msg));
end
fprintf('\n');

% Definimos los colores una sola vez para que ambas gráficas los usen
colores = lines(4);

fprintf('4. Exportando PDF 1: Torques Realistas...\n');
fig_tau = figure('Name', 'Torques por Eje', 'Color', 'white', 'Units', 'centimeters', 'Position', [2 2 22 16]);
titulos_tau = {'\tau_1 (Columna Base)', '\tau_2 (Eslabón Inclinado)', '\tau_3 (Link Arm)', '\tau_4 (Antebrazo)'};
for i = 1:4
    subplot(2, 2, i);
    plot(t_s, tau_traj(i,:), 'LineWidth', 1.8, 'Color', colores(i,:)); hold on;
    xline(1, '--k', 'Pick', 'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom'); 
    xline(2, '--k', 'Place', 'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom');
    title(titulos_tau{i}, 'FontName', 'Times New Roman', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Tiempo (s)', 'FontName', 'Times New Roman', 'FontSize', 10);
    ylabel('Torque (N\cdotm)', 'FontName', 'Times New Roman', 'FontSize', 10);
    grid on; set(gca, 'FontName', 'Times New Roman', 'FontSize', 9);
end
sgtitle('Dinámica Realista: Requerimiento de Torque Individual por Eje', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
exportgraphics(fig_tau, 'kr300pa_torques_realistas.pdf', 'ContentType', 'vector', 'BackgroundColor', 'white');

fprintf('5. Exportando PDF 2: Cinemática Suavizada...\n');
fig_cin = figure('Name', 'Perfiles Cinemáticos', 'Color', 'white', 'Units', 'centimeters', 'Position', [5 2 20 20]);

% --- Gráfica 1: Posición ---
subplot(3, 1, 1); hold on;
for i=1:4, plot(t_s, q_traj(i,:), 'LineWidth', 1.5, 'Color', colores(i,:)); end
xline(1,'--k'); xline(2,'--k');
title('Posición Articular: Evolución de los ángulos de cada motor', 'Interpreter', 'latex', 'FontSize', 12);
ylabel('Ángulo (deg)', 'FontName', 'Times New Roman', 'FontSize', 10);
legend({'A_1', 'A_2', 'A_3', 'A_4'}, 'Location', 'best', 'NumColumns', 4, 'FontName', 'Times New Roman');
grid on; set(gca, 'FontName', 'Times New Roman', 'FontSize', 9);

% --- Gráfica 2: Velocidad ---
subplot(3, 1, 2); hold on;
for i=1:4, plot(t_s, rad2deg(dq_traj(i,:)), 'LineWidth', 1.5, 'Color', colores(i,:)); end
xline(1,'--k'); xline(2,'--k');
title('Velocidad Articular: Perfil suave en forma de campana (Curva S) - Curva en Campana}', 'Interpreter', 'latex', 'FontSize', 12);
ylabel('Velocidad (deg/s)', 'FontName', 'Times New Roman', 'FontSize', 10);
grid on; set(gca, 'FontName', 'Times New Roman', 'FontSize', 9);

% --- Gráfica 3: Aceleración ---
subplot(3, 1, 3); hold on;
for i=1:4, plot(t_s, rad2deg(ddq_traj(i,:)), 'LineWidth', 1.5, 'Color', colores(i,:)); end
xline(1,'--k'); xline(2,'--k');
title('Aceleración Articular: Cambios de velocidad controlados sin picos infinitos', 'Interpreter', 'latex', 'FontSize', 12);
xlabel('Tiempo (s)', 'FontName', 'Times New Roman', 'FontSize', 10);
ylabel('Aceleración (deg/s^2)', 'FontName', 'Times New Roman', 'FontSize', 10);
grid on; set(gca, 'FontName', 'Times New Roman', 'FontSize', 9);

sgtitle('Cinemática Industrial Suavizada (Polinomio de 5to Orden)', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
exportgraphics(fig_cin, 'kr300pa_perfiles_suavizados.pdf', 'ContentType', 'vector', 'BackgroundColor', 'white');

fprintf('=== PROCESO COMPLETADO EXITOSAMENTE ===\n');