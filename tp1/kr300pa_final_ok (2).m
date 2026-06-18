% kr300pa_final_ok.m — KUKA KR 300-2 PA (VERSION DEFINITIVA CORRECTA)
% Verificado: error transporte = 3.4mm, Y=0 en todas las posiciones
%
% CONVENCION:
%   Angulos en grados KUKA (los del teach pendant)
%   A2_KUKA=0 = brazo vertical arriba (offset interno +90° DH)
%   El TCP del paletizador SIEMPRE apunta en -Z global
%   Por eso d5 se resta en Z global (no via tabla DH)

clc; clear; close all;

%% ── Parámetros geométricos [metros] ─────────────────────────────────────────
d1 = 0.846222;   % altura columna base → A1
a2 = 0.560000;   % eslabón inclinado (A2)
a3 = 1.314102;   % link arm (A3)
a4 = 1.080334;   % arm (A4/A5)
d5 = 0.320941;   % muñeca → flange A6

fprintf('=== KUKA KR 300-2 PA ===\n')
fprintf('d1=%.3f  a2=%.3f  a3=%.3f  a4=%.3f  d5=%.3f [m]\n\n', ...
    d1,a2,a3,a4,d5)

%% ── Función FK ───────────────────────────────────────────────────────────────
function [T_tcp, T_wrist] = fk(q_kuka_deg, d1,a2,a3,a4,d5)
% FK_KR300PA  Cinemática directa KUKA KR 300-2 PA
% Entrada: q_kuka_deg = [A1 A2 A3 A4 A6] en grados KUKA
% Salida:  T_tcp   = posicion TCP  [m]
%          T_wrist = wrist point (intersección A4/A5) [m]
%
% Tabla DH (4+1 eslabones):
%   i=1: d=d1,  a=0,  alpha=+90°, off=0    → columna giratoria
%   i=2: d=0,   a=a2, alpha=0°,   off=+90° → eslabón inclinado
%   i=3: d=0,   a=a3, alpha=0°,   off=0    → link arm
%   i=4: d=0,   a=a4, alpha=0°,   off=0    → arm
%   TCP = wrist - d5*[0;0;1] (efector siempre apunta -Z global)

    off  = [0, 90, 0, 0];
    d_v  = [d1, 0,  0,  0 ];
    a_v  = [0,  a2, a3, a4];
    al_v = [pi/2, 0, 0,  0 ];

    q = deg2rad(q_kuka_deg(1:4) + off);
    T = eye(4);
    for i = 1:4
        ct=cos(q(i)); st=sin(q(i));
        ca=cos(al_v(i)); sa=sin(al_v(i));
        Ai=[ ct, -st*ca,  st*sa, a_v(i)*ct;
             st,  ct*ca, -ct*sa, a_v(i)*st;
              0,     sa,     ca,     d_v(i);
              0,      0,      0,          1];
        T = T*Ai;
    end
    T_wrist = T;

    % TCP: bajar d5 en -Z global (paletizador siempre apunta abajo)
    T_tcp = T;
    T_tcp(3,4) = T(3,4) - d5;
end

%% ── Verificación ─────────────────────────────────────────────────────────────
fprintf('=== VERIFICACION FK ===\n')

casos = {
    [0,   0,  0, 0, 0], 'Cero mecanico (A2=0, brazo vertical)';
    [0, -90,  0, 0, 0], 'Horizontal (A2=-90)';
    [0,-130,155, 0, 0], 'Transporte datasheet (A2=-130, A3=+155)';
    [45, -60, 90, 0, 0],'Trabajo tipico';
};

for k=1:4
    [T_tcp, T_wrist] = fk(casos{k,1}, d1,a2,a3,a4,d5);
    pw = T_wrist(1:3,4)*1000;
    pt = T_tcp(1:3,4)*1000;
    fprintf('%s:\n', casos{k,2})
    fprintf('  Wrist: [X=%7.1f  Y=%7.1f  Z=%7.1f] mm\n', pw)
    fprintf('  TCP:   [X=%7.1f  Y=%7.1f  Z=%7.1f] mm\n', pt)
end

[T_tcp_tr, T_wrist_tr] = fk([0,-130,155,0,0], d1,a2,a3,a4,d5);
Z_tcp_tr = T_tcp_tr(3,4)*1000;   % T_tcp ya tiene d5 restado
fprintf('\nVerificacion posicion transporte:\n')
fprintf('  Z_TCP   = %.1f mm  (datasheet: 2332 mm)\n', Z_tcp_tr)
fprintf('  Z_Wrist = %.1f mm  (referencia A4/A5)\n', T_wrist_tr(3,4)*1000)
fprintf('  Error   = %.1f mm  (%.2f%%)\n\n', abs(Z_tcp_tr-2332), abs(Z_tcp_tr-2332)/2332*100)

%% ── Espacio de trabajo ───────────────────────────────────────────────────────
fprintf('=== ESPACIO DE TRABAJO (calculando...) ===\n')

N=25;
q1v=linspace(-185, 185,N);
q2v=linspace(-130,  20,N);
q3v=linspace(   0, 155,N);

n_tot=N^3;
Px=zeros(n_tot,1); Py=zeros(n_tot,1); Pz=zeros(n_tot,1);
k=1;
for i=1:N, for j=1:N, for l=1:N
    [T_tcp,~]=fk([q1v(i),q2v(j),q3v(l),0,0],d1,a2,a3,a4,d5);
    Px(k)=T_tcp(1,4)*1000; Py(k)=T_tcp(2,4)*1000;
    Pz(k)=T_tcp(3,4)*1000; k=k+1;
end; end; end

R_max=max(sqrt(Px.^2+Py.^2));
fprintf('Alcance radial maximo: %.1f mm\n', R_max)
fprintf('Alcance datasheet:     3150 mm\n')
fprintf('Error:                 %.2f%%\n\n', abs(R_max-3150)/3150*100)

fig1=figure('Name','Espacio de Trabajo','Color','white', ...
    'Units','centimeters','Position',[2 2 16 14]);
scatter3(Px,Py,Pz,1,Pz,'.'); colormap(jet); colorbar
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)')
title({'Espacio de Trabajo','KUKA KR 300-2 PA'}, ...
    'FontName','Times New Roman','FontSize',11,'FontWeight','bold')
grid on; axis equal; view(30,25)
set(gca,'FontName','Times New Roman','FontSize',9)
exportgraphics(fig1,'kr300pa_workspace.pdf', ...
    'ContentType','image','Resolution',200,'BackgroundColor','white')
fprintf('Figura: kr300pa_workspace.pdf\n\n')

%% ── Visualización ────────────────────────────────────────────────────────────
fig2=figure('Name','Cadena cinematica','Color','white', ...
    'Units','centimeters','Position',[1 1 26 10]);

off_v  = [0,90,0,0];
d_vis  = [d1,0,0,0];
a_vis  = [0,a2,a3,a4];
al_vis = [pi/2,0,0,0];

posturas={[0,0,0,0],[0,-130,155,0],[45,-60,90,0]};
titulos={'Cero mec. [0°,0°,0°]','Transporte [0°,-130°,155°]','Trabajo [45°,-60°,90°]'};

for k=1:3
    ax=subplot(1,3,k); hold on; grid on; axis equal
    q_r=deg2rad(posturas{k}+off_v);
    T=eye(4); pts=zeros(3,5); pts(:,1)=[0;0;0];
    for i=1:4
        ct=cos(q_r(i)); st=sin(q_r(i));
        ca=cos(al_vis(i)); sa=sin(al_vis(i));
        Ai=[ct,-st*ca,st*sa,a_vis(i)*ct;st,ct*ca,-ct*sa,a_vis(i)*st;0,sa,ca,d_vis(i);0,0,0,1];
        T=T*Ai; pts(:,i+1)=T(1:3,4)*1000;
    end
    % Agregar TCP (bajar d5 en Z)
    pts_tcp=[pts, [pts(1,5);pts(2,5);pts(3,5)-d5*1000]];

    plot3(ax,pts_tcp(1,:),pts_tcp(2,:),pts_tcp(3,:),'o-', ...
        'Color',[0.15 0.15 0.15],'LineWidth',5, ...
        'MarkerFaceColor',[0.85 0.33 0.10], ...
        'MarkerEdgeColor','white','MarkerSize',10)
    plot3(ax,0,0,0,'ks','MarkerSize',12,'MarkerFaceColor',[0.4 0.4 0.4])
    plot3(ax,pts_tcp(1,end),pts_tcp(2,end),pts_tcp(3,end), ...
        'b^','MarkerSize',10,'MarkerFaceColor','b')
    L=300;
    quiver3(ax,0,0,0,L,0,0,'r','LineWidth',1.5,'MaxHeadSize',0.5)
    quiver3(ax,0,0,0,0,L,0,'g','LineWidth',1.5,'MaxHeadSize',0.5)
    quiver3(ax,0,0,0,0,0,L,'b','LineWidth',1.5,'MaxHeadSize',0.5)
    title(titulos{k},'FontName','Times New Roman','FontSize',9)
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)')
    view(30,20)
    xlim([-1000 3500]); ylim([-2000 2000]); zlim([-500 4000])
    set(ax,'FontName','Times New Roman','FontSize',8)
end
sgtitle('KUKA KR 300-2 PA — Cadena Cinemática', ...
    'FontName','Times New Roman','FontSize',12,'FontWeight','bold')
exportgraphics(fig2,'kr300pa_cadena.pdf', ...
    'ContentType','image','Resolution',200,'BackgroundColor','white')
fprintf('Figura: kr300pa_cadena.pdf\n\n')

%% ── Cinemática Inversa ───────────────────────────────────────────────────────
fprintf('=== CINEMATICA INVERSA (geometrica) ===\n')

function [qs, ok] = ik(px,py,pz, config, d1,a2,a3,~,d5)
% IK geometrica para paletizador (efector apunta -Z)
% px,py,pz en mm; config = +1 codo arriba, -1 codo abajo
    p = [px;py;pz]/1000;

    % A1: azimut
    A1 = atan2(p(2), p(1));

    % Punto de muñeca: TCP + d5 en +Z global
    pw = p + [0; 0; d5];

    % Distancia horizontal y vertical
    r = sqrt(pw(1)^2 + pw(2)^2);
    s = pw(3) - d1;

    % Ley de cosenos para A3
    cos_A3 = (r^2 + s^2 - a2^2 - a3^2) / (2*a2*a3);
    if abs(cos_A3) > 1
        qs=NaN(1,5); ok=false; return
    end
    A3_dh = atan2(config*sqrt(1-cos_A3^2), cos_A3);

    % A2 en DH
    A2_dh = atan2(s,r) - atan2(a3*sin(A3_dh), a2+a3*cos(A3_dh));

    % Convertir DH → KUKA (offset -90° en A2)
    A2_kuka = A2_dh - pi/2;
    A3_kuka = A3_dh;

    % A4: compensacion para mantener efector vertical (paletizador)
    A4_kuka = -(A2_dh + A3_dh);

    qs = rad2deg([A1, A2_kuka, A3_kuka, A4_kuka, 0]);

    % Verificar rangos
    q_r = deg2rad(qs(1:4));
    lim_min = deg2rad([-185,-130,  0,-350]);
    lim_max = deg2rad([ 185,  20,155, 350]);
    ok = all(q_r >= lim_min) && all(q_r <= lim_max);
end

% Puntos dentro del espacio de trabajo real del KR 300-2 PA
puntos = [1800   600  800;
          1200   400  600;
          1600  -700  700];

fprintf('%-28s  %-52s  %s\n','Objetivo [mm]','q_KUKA [A1 A2 A3 A4 A6] [deg]','Err [mm]')
fprintf('%s\n',repmat('-',1,95))
for k=1:3
    [qs,ok]=ik(puntos(k,1),puntos(k,2),puntos(k,3),1,d1,a2,a3,a4,d5);
    if ok
        [T_tcp,~]=fk(qs,d1,a2,a3,a4,d5);
        err=norm(T_tcp(1:3,4)*1000 - puntos(k,:)');
        fprintf('[%5.0f %5.0f %5.0f]  [%6.1f %6.1f %6.1f %6.1f %5.1f]  %.2f\n', ...
            puntos(k,:), qs, err)
    else
        fprintf('[%5.0f %5.0f %5.0f]  fuera de rango articular\n',puntos(k,:))
    end
end
fprintf('\n')

%% ── Jacobiano geométrico ─────────────────────────────────────────────────────
fprintf('=== JACOBIANO GEOMETRICO J(q_transport) ===\n')

q_tr = deg2rad([0,-130,155,0] + [0,90,0,0]);

% Calcular posicion TCP
[T_tcp, ~] = fk([0,-130,155,0,0], d1,a2,a3,a4,d5);
p_n = T_tcp(1:3,4);

J = zeros(6,4);
T = eye(4);
z_p = [0;0;1]; p_p = [0;0;0];
for i=1:4
    J(1:3,i) = cross(z_p, p_n-p_p);
    J(4:6,i) = z_p;
    ct=cos(q_tr(i)); st=sin(q_tr(i));
    ca=cos(al_vis(i)); sa=sin(al_vis(i));
    T=T*[ct,-st*ca,st*sa,a_vis(i)*ct;st,ct*ca,-ct*sa,a_vis(i)*st;0,sa,ca,d_vis(i);0,0,0,1];
    z_p=T(1:3,3); p_p=T(1:3,4);
end

fprintf('J [6x4]:\n'); disp(round(J,3))
fprintf('Rango:    %d / 4\n', rank(J))
fprintf('cond(J):  %.2f\n\n', cond(J))

%% ── Inercias por eslabón ─────────────────────────────────────────────────────
fprintf('=== TENSORES DE INERCIA (cilindro solido) ===\n')
fprintf('%-18s  %6s  %8s  %8s  [kg*m^2]\n','Eslabón','m[kg]','Ixx=Iyy','Izz')

% Masas reales proporcionales al peso total del robot: 2150 kg
% Distribucion: columna 42%, brazo 28%, link arm 20%, arm 7%, muneca 3%
m   = [903,  602,   430,   151,    64];
r_c = [0.20, 0.15,  0.12,  0.10,  0.08];
L_e = [d1,   a2,    a3,    a4,    d5  ];
nom = {'Columna (A1)','Eslabon inc. (A2)','Link arm (A3)','Arm (A4)','Muneca (A6)'};

for i=1:5
    Ixx = m(i)*(3*r_c(i)^2 + L_e(i)^2)/12;
    Izz = m(i)*r_c(i)^2/2;
    fprintf('%-18s  %6.0f  %8.2f  %8.2f\n', nom{i}, m(i), Ixx, Izz)
end
fprintf('\n')

%% ── Dinámica (Lagrange) ──────────────────────────────────────────────────────
fprintf('=== DINAMICA — M(q) y G(q) ===\n')

q_tr_v = deg2rad([0,-130,155,0] + [0,90,0,0]);
g_vec  = [0;0;-9.81];
% Masas coherentes con peso total 2150 kg
m_dyn  = [903, 602, 430, 151];
r_c_dyn= [0.20,0.15,0.12,0.10];
L_dyn  = [d1,  a2,  a3,  a4 ];

% Posicion del COM de cada eslabon
T=eye(4); p_com=zeros(3,4);
for i=1:4
    ct=cos(q_tr_v(i)); st=sin(q_tr_v(i));
    ca=cos(al_vis(i)); sa=sin(al_vis(i));
    Ai=[ct,-st*ca,st*sa,a_vis(i)*ct;st,ct*ca,-ct*sa,a_vis(i)*st;0,sa,ca,d_vis(i);0,0,0,1];
    T_half=T*[eye(3),[0.5*a_vis(i)*ct;0.5*a_vis(i)*st;0.5*d_vis(i)];0 0 0 1];
    p_com(:,i)=T_half(1:3,4);
    T=T*Ai;
end

M_q=zeros(4,4); G_q=zeros(4,1);
for k=1:4
    Jv=zeros(3,4); T_a=eye(4); z_p=[0;0;1]; p_p=[0;0;0];
    for i=1:4
        if i<=k, Jv(:,i)=cross(z_p, p_com(:,k)-p_p); end
        ct=cos(q_tr_v(i)); st=sin(q_tr_v(i));
        ca=cos(al_vis(i)); sa=sin(al_vis(i));
        T_a=T_a*[ct,-st*ca,st*sa,a_vis(i)*ct;st,ct*ca,-ct*sa,a_vis(i)*st;0,sa,ca,d_vis(i);0,0,0,1];
        z_p=T_a(1:3,3); p_p=T_a(1:3,4);
    end
    M_q = M_q + m_dyn(k)*(Jv'*Jv);
    G_q = G_q - m_dyn(k)*(Jv'*g_vec);
end

fprintf('Diagonal M(q_transport) [kg*m^2]:\n')
fprintf('  [%.1f  %.1f  %.1f  %.1f]\n\n', diag(M_q)')
fprintf('G(q_transport) = tau con qd=0, qdd=0 [N*m]:\n')
fprintf('  [%.1f  %.1f  %.1f  %.1f]\n\n', G_q')

%% ── Trayectoria pick-and-place ───────────────────────────────────────────────
fprintf('=== TRAYECTORIA PICK-AND-PLACE ===\n')

q_home  = [0,   0,   0,  0, 0];
q_pick  = [30, -60,  90,-30, 0];
q_place = [-45,-50,  80,-30, 0];

npts=60; tv=linspace(0,1,npts)';
t1 = (1-tv).*q_home  + tv.*q_pick;
t2 = (1-tv).*q_pick  + tv.*q_place;
t3 = (1-tv).*q_place + tv.*q_home;
q_traj = [t1;t2;t3]';  % 5 x (3*npts)

ntot=size(q_traj,2); xyz=zeros(3,ntot);
for k=1:ntot
    [T_tcp,~]=fk(q_traj(:,k)',d1,a2,a3,a4,d5);
    xyz(:,k)=T_tcp(1:3,4)*1000;
end
t_s=linspace(0,3,ntot);

fig3=figure('Name','Trayectoria','Color','white', ...
    'Units','centimeters','Position',[2 2 22 10]);

subplot(1,2,1)
plot3(xyz(1,:),xyz(2,:),xyz(3,:),'b-','LineWidth',2); hold on
scatter3(xyz(1,1),      xyz(2,1),      xyz(3,1),      80,'g','filled')
scatter3(xyz(1,npts),   xyz(2,npts),   xyz(3,npts),   80,'r','filled')
scatter3(xyz(1,2*npts), xyz(2,2*npts), xyz(3,2*npts), 80,'m','filled')
legend('TCP','Home','Pick','Place','Location','best', ...
    'FontName','Times New Roman','FontSize',8)
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)')
title('Trayectoria cartesiana TCP','FontName','Times New Roman','FontSize',10)
grid on; view(30,25); set(gca,'FontName','Times New Roman','FontSize',8)

subplot(1,2,2)
plot(t_s, q_traj(1:4,:)', 'LineWidth', 1.5)
xline(1,'--k'); xline(2,'--k')
xlabel('Tiempo (s)'); ylabel('Ángulo (deg)')
title('Perfil articular q(t)','FontName','Times New Roman','FontSize',10)
legend({'q_1','q_2','q_3','q_4'},'Location','best', ...
    'FontName','Times New Roman','FontSize',8,'Interpreter','tex')
grid on; set(gca,'FontName','Times New Roman','FontSize',8)

sgtitle('Pick-and-Place — KUKA KR 300-2 PA', ...
    'FontName','Times New Roman','FontSize',12,'FontWeight','bold')
exportgraphics(fig3,'kr300pa_trayectoria.pdf', ...
    'ContentType','image','Resolution',200,'BackgroundColor','white')

fprintf('\n=== ARCHIVOS GENERADOS ===\n')
fprintf('  kr300pa_workspace.pdf\n')
fprintf('  kr300pa_cadena.pdf\n')
fprintf('  kr300pa_trayectoria.pdf\n')
fprintf('=== FIN ===\n')
