clc
clear all

%% Read Nodes and Elements
Nodes = dlmread('./Nodes.dat');
Elements = dlmread('./Elements.dat');
N = size(Nodes,1);  % Number of nodes per side

%% Load Matrices
fname = './Modelmats.mat';
load(fname, 'M', 'K', 'R', 'Fv');
R = R';
Fv = -Fv';

Nint = size(M,1)-(2*N)*3;

%% Contact Relative Displacements
Lz = kron(eye(N), [0 0 1]); % Get only normal displacement
Lrel = [Lz -Lz zeros(N, Nint)];
% Lrel defd such that Lrel*x>0 implies contact and Lrel*x<0 implies separation.

%% Remove null-space
L1 = null(Lrel);
[V,D] = eigs(L1'*K*L1, L1'*M*L1, 20, 'SM'); % Get first 20 modes
Ln = null(V(:, 1:6)'*L1'*M);  % First six modes are RBMs

Nn = size(Ln, 2);  % Null-reduced DOFs

%% Evaluate RESFUN
bpmag = 10e3;
knl = 1e6;
% U0 = (Ln'*K*Ln + (Lrel*Ln)'*(Lrel*Ln)*knl)\(Ln'*Fv*bpmag);

opt = optimoptions('fsolve', 'specifyObjectiveGradient', true, 'Display', 'iter');
U0 = fsolve(@(U) RESFUN([U; bpmag], Ln'*K*Ln, Ln'*Fv, Lrel*Ln, knl), U0, opt);
