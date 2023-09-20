clc
clear all

set(0,'defaultAxesTickLabelInterpreter', 'default');
set(0,'defaultTextInterpreter','latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0,'defaultAxesFontSize',13);

%% Read Nodes and Elements
Nodes = dlmread('./Nodes.dat');
Elements = dlmread('./Elements.dat');
N = size(Nodes,1);  % Number of nodes per side
Ne = size(Elements,1);  % Number of elements

%% Load Matrices
fname = './Modelmats.mat';
load(fname, 'M', 'K', 'R', 'Fv');

% Number of generalized modal DOFs
Nint = size(M,1)-(2*N)*3;

%% Single-Point-Quadrature Matrices
Qm = zeros(Ne, N);  % Each element has one quadrature point
Tm = zeros(N, Ne);  % Each quadrature pt integrated to 4 nodes
Ars = zeros(Ne, 1);  % Vector of areas of each element
for ei=1:Ne
    ndis = Elements(ei, 2:end);  % Nodes of the current element
    Qm(ei, ndis) = ones(1, 4)/4;
                % value @ QP = avg of nodal values

    Ars(ei) = polyarea(Nodes(ndis,1), Nodes(ndis,2));

    Tm(ndis, ei) = ones(4, 1)*Ars(ei)/4;
                % (integrated) nodal value = value @ QP times element Area/4
end
% Qm is quadrature interpolation matrix
% Tm is quadrature integration matrix

Ar_tot = 120e-3*25.4e-3-3*pi*(0.85e-2/2)^2; % True total area
                                            % Rectangle-3*Circles
Ar_avg = Ar_tot/Ne;  % Average element area

%% Contact Relative Displacements
Lz = kron(Qm, [0 0 1]); % Get only normal displacement
Lrel = [Lz -Lz zeros(Ne, Nint)];
% Lrel defd such that Lrel*x>0 implies contact and Lrel*x<0 implies separation.

Gz = kron(Tm, [0; 0; 1]);
Grel = [Gz; -Gz; zeros(Nint, Ne)];

% Nodal relative disp (only for plotting)
Lz_n = kron(eye(N), [0 0 1]); % Get only normal displacement
Lrel_n = [Lz_n -Lz_n zeros(N, Nint)];

%% Remove fixed interface null-space
L1 = null(Lrel);
[V,D] = eigs(L1'*K*L1, L1'*M*L1, 20, 'SM'); % Get first 20 modes
Ln = null(V(:, 1:6)'*L1'*M);  % First six modes are RBMs

Nn = size(Ln, 2);  % Null-reduced DOFs

%% Solve the Nonlinear Static Prestress Problem
bpmag = 12e3;
knl = 5e6/(Ar_tot/Ne); % knl divided by avg element area
U0 = (Ln'*K*Ln + (Grel'*Ln)'*(Lrel*Ln)*knl)\(Ln'*Fv*bpmag);

opt = optimoptions('fsolve', 'specifyObjectiveGradient', true, 'Display', 'iter');
[U0, ~, ~, ~, J0] = fsolve(@(U) RESFUN([U; bpmag], Ln'*K*Ln, Ln'*Fv, Lrel*Ln, Ln'*Grel, knl), U0, opt);

%% Linearized Modal Analysis
[V0, D0] = eigs(J0, Ln'*M*Ln, 10, 'SM');
W0 = sqrt(diag(D0));
disp(table((1:10)', W0/2/pi, 'VariableNames', {'Index', 'Frequency (Hz)'}))

%% Depict Interfacial Displacement Field
A = zeros(N);  % Graph adjacency matrix
for ei=1:Ne
       ndis = Elements(ei, 2:end);  % Nodes of the current element
       A(ndis, ndis([2:end 1])) = A(ndis, ndis([2:end 1])) + eye(4);
       A(ndis([2:end 1]), ndis) = A(ndis([2:end 1]), ndis) + eye(4);
end
G = graph(A);

figure(1)
pos=get(gcf, 'Position');
set(gcf, 'Position', [pos(1:2) 900 320], 'Color', 'white')
clf()
plot(G, 'XData', Nodes(:,1), 'YData', Nodes(:,2), ...
     'NodeCData', Lrel_n*Ln*U0, ...
     'MarkerSize', 4, 'LineWidth', 3)
colormap(jet)
grid on
axis equal
axis tight
set(gca, 'XTick', (-6:3:6)*1e-2)
xlim([-1 1]*6e-2)
ylim([-1 1]*1.27e-2)
xlabel('X Coordinate (m)')
ylabel('Y Coordinate (m)')
xx=colorbar('SouthOutside');
xlabel(xx, 'Nodal Relative Displacement (m)', 'interpreter', 'latex', 'fontsize', 13)

print('./intdisps.png', '-dpng', '-r300')

%% Show Quadrature Nodes
figure(2)
pos=get(gcf, 'Position');
set(gcf, 'Position', [pos(1:2) 900 260], 'Color', 'white')
clf()
plot(G, 'XData', Nodes(:,1), 'YData', Nodes(:,2), ...
     'NodeColor', 'k', 'EdgeColor', 'k'); hold on
plot(Qm*Nodes(:,1), Qm*Nodes(:,2), 'r.')
grid on
axis equal
axis tight
xlim([-1 1]*6e-2)
ylim([-1 1]*1.27e-2)
set(gca, 'XTick', (-6:3:6)*1e-2)
xlabel('X Coordinate (m)')
ylabel('Y Coordinate (m)')

print('./quadpts.png', '-dpng', '-r300')

%% Show Pressure field
pvals = max(knl*Lrel*Ln*U0, 0);  % pressure @ QPs
figure(3)
pos=get(gcf, 'Position');
set(gcf, 'Position', [pos(1:2) 900 320], 'Color', 'white')
clf()
for ei=1:Ne
    ndis = Elements(ei, 2:end);  % Nodes of the current element
    fill(Nodes(ndis,1), Nodes(ndis,2), pvals(ei)); hold on
end
colormap(jet)
grid on
axis equal
axis tight
set(gca, 'XTick', (-6:3:6)*1e-2)
xlabel('X Coordinate (m)')
ylabel('Y Coordinate (m)')
xx=colorbar('SouthOutside');
xlabel(xx, 'Element Normal Pressure (Pa)', 'interpreter', 'latex', 'fontsize', 13)

print('./intpress.png', '-dpng', '-r300')
