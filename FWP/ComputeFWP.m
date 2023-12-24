clear
clc
close all

addpath([pwd() '\..\Visualization']);%,'-end');
addpath([pwd(), '\..\Utils']);%,'-end');
addpath(genpath([pwd() '\..\spatial_v2_extended']));%,'-end');
addpath(genpath([pwd() '\..\Models']));%,'-end')

% https://arxiv.org/pdf/1712.02731.pdf

%% robot specs

params = getMiniCheetahParams();
robot_tree = buildTreeModel(params);
pos = [0,0,0.18]';
eul = zeros(3,1);
qleg = [0,-1.0405,2.2205];

rNominal = zeros(4,3);
% [fr,fl,hr,hl]
for leg = 1:4
    rNominal(leg,:) = forward_kinematics(pos, eul, qleg', leg) - pos;
end
mu = 0.7;

% compute intersection of AWP and CWC in 3D for each foot, 
% then use vertices to go to 6D, 
% and finally Minkowski sum each foot

%% restore path
tbxmanager restorepath

%% intersection of AFP and CWC
% CWC - Contact Wrench Cone
A = [ 1  0 -mu/sqrt(2);
     -1  0 -mu/sqrt(2);
      0  1 -mu/sqrt(2);
      0 -1 -mu/sqrt(2);
      0  0  1
      0  0 -1];
    

b = [0;
     0;
     0;
     0;
     150;
     0];
  
FrictionCone = Polyhedron('A', A, 'b', b);

%% AFP - Actuation Force Polytope
tau_max = 17; % N/m
tau = [ tau_max, tau_max, tau_max;
        tau_max, tau_max,-tau_max;
        tau_max,-tau_max, tau_max;
        tau_max,-tau_max,-tau_max;
       -tau_max, tau_max, tau_max;
       -tau_max, tau_max,-tau_max;
       -tau_max,-tau_max, tau_max;
       -tau_max,-tau_max,-tau_max];
nVertices = size(tau,1);

tic 
% FR
AFPFR_v = zeros(nVertices,3); 
J = compute_foot_jacobian(robot_tree,pos,eul,qleg',1);
for i = 1:nVertices
    AFPFR_v(i,:) = -inv(J(:,7:9))'*tau(i,:)';
end
AFPFR = Polyhedron('V', AFPFR_v);
FFPFR = AFPFR.intersect(FrictionCone);

%FL
AFPFL_v = zeros(nVertices,3); 
J = compute_foot_jacobian(robot_tree,pos,eul,qleg',2);
for i = 1:nVertices
    AFPFL_v(i,:) = -inv(J(:,10:12))'*tau(i,:)';
end
AFPFL = Polyhedron('V', AFPFL_v);
FFPFL = FrictionCone.intersect(AFPFL);

%HR
AFPHR_v = zeros(nVertices,3); 
J = compute_foot_jacobian(robot_tree,pos,eul,qleg',3);
for i = 1:nVertices
    AFPHR_v(i,:) = -inv(J(:,13:15))'*tau(i,:)';
end
AFPHR = Polyhedron('V', AFPHR_v);
FFPHR = FrictionCone.intersect(AFPHR);

%HL
AFPHL_v = zeros(nVertices,3); 
J = compute_foot_jacobian(robot_tree,pos,eul,qleg',4);
for i = 1:nVertices
    AFPHL_v(i,:) = -inv(J(:,16:18))'*tau(i,:)';
end
AFPHL = Polyhedron('V', AFPHL_v);
FFPHL = FrictionCone.intersect(AFPHL);

%% FFP to FWP (3D --> 6D)

FFPFR = FFPFR.minVRep();
VFR = zeros(length(FFPFR.V(:,1)),6);
VFR(1:length(FFPFR.V(:,1)),4:6) = FFPFR.V;

FFPFL = FFPFL.minVRep();
VFL = zeros(length(FFPFL.V(:,1)),6);
VFL(1:length(FFPFL.V(:,1)),4:6) = FFPFL.V;

FFPHR = FFPHR.minVRep();
VHR = zeros(length(FFPHR.V(:,1)),6);
VHR(1:length(FFPHR.V(:,1)),4:6) = FFPHR.V;

FFPHL = FFPHL.minVRep();
VHL = zeros(length(FFPHL.V(:,1)),6);
VHL(1:length(FFPHL.V(:,1)),4:6) = FFPHL.V;

% http://groups.csail.mit.edu/robotics-center/public_papers/Dai16.pdf
for i = 1:length(VFR(:,1))
    VFR(i,1:3) = cross(rNominal(1,:)',VFR(i,4:6));
    VFL(i,1:3) = cross(rNominal(2,:)',VFL(i,4:6));
    VHR(i,1:3) = cross(rNominal(3,:)',VHR(i,4:6));
    VHL(i,1:3) = cross(rNominal(4,:)',VHL(i,4:6));
end

FWPFR = Polyhedron(VFR);
FWPFL = Polyhedron(VFL);
FWPHR = Polyhedron(VHR);
FWPHL = Polyhedron(VHL);

%% Minkowski sum for full FWP

FWP = plus(FWPFR,FWPFL);
FWP = plus(FWP,FWPHR);
FWP = plus(FWP,FWPHL);

toc

%% save FWP 
% not sure why, but saving in vertex form is much more reliable

V_FWP = FWP.V;

% each foot
FWPFR = FWPFR.minVRep();
V_FWPFR = FWPFR.V;

FWPFL = FWPFL.minVRep();
V_FWPFL = FWPFL.V;

FWPHR = FWPHR.minVRep();
V_FWPHR = FWPHR.V;

FWPHL = FWPHL.minVRep();
V_FWPHL = FWPHL.V;


save("FWP.mat",'V_FWP','V_FWPFR','V_FWPFL','V_FWPHR','V_FWPHL');
                 
                 

                 
                 
 




