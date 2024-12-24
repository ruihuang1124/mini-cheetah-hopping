%% ExampleMIP.m
clc; clear; 
addpath(genpath([pwd() '/gurobi1200']));%,'-end');

%% physics
g = [0;0;0; 0;0;9.8];

%% robot specifications
pNominal = [ 0.19, -0.1110, 0;
             0.19,  0.1110, 0;
            -0.19, -0.1110, 0;
            -0.19,  0.1110, 0];
nFeet = length(pNominal(:,1));
pCOM = [0 0 0.18]'; % COM position relative to "contact point"

fzMAX = 500; 
load('FWP/FWP.mat');
scale = 1e-5;
FWP = Polyhedron('V',scale*V_FWP); % need to scale to compute HRep
FWP.minHRep();

h = 2.5e-1;  
m = 9; % mass in kg 
M = diag([m m m]);
I = diag([11253,36203,42673])*1e-6;
H = [I,zeros(3);
     zeros(3),M];

radius = norm(pNominal(1,:) - pCOM');
radius_platform = 0.32;

%% footstep regions
nFootstepRegions = 4;
start_region = 1; % START
yaw_init = 0;
goal_region = 4; % GOAL
edgeLength = 0.65;
gapLength = 0.1;

boxDimensions = [64.5, 55.5, 14] / 100;
angledBoxDimensions = [65, 65, 11] / 100;

xCenters = [0, ...
    edgeLength/2 + gapLength + angledBoxDimensions(1)/2, ...
    edgeLength/2 + gapLength + angledBoxDimensions(1)/2, ...
    edgeLength/2 + gapLength + angledBoxDimensions(1) + 2*gapLength + edgeLength/2];
yCenters = [0, -(gapLength + edgeLength)/2, (gapLength + edgeLength)/2, 0];
zCenters = [0, angledBoxDimensions(3), 0.14, 0];
centers = [xCenters;yCenters;zCenters];

vertices = cell(nFootstepRegions,1);
angles = cell(nFootstepRegions,1);
rotms = cell(nFootstepRegions,1);
pCOMrel = cell(nFootstepRegions,1);
xMins = zeros(nFootstepRegions,1);
yMins = zeros(nFootstepRegions,1);
xMaxs = zeros(nFootstepRegions,1);
yMaxs = zeros(nFootstepRegions,1);

angles{1} = zeros(1,3);
angles{2} = [-10 * pi/180, 0, 0];
angles{3} = zeros(1,3);
angles{4} = zeros(1,3);

figure(1)
clf
for i = 1:nFootstepRegions
    if i ~= 2
        vertices{i} = zeros(4,3);
        vertices{i}(1,:) = [xCenters(i)-edgeLength/2,yCenters(i)-edgeLength/2,zCenters(i)];
        vertices{i}(2,:) = [xCenters(i)+edgeLength/2,yCenters(i)-edgeLength/2,zCenters(i)];
        vertices{i}(3,:) = [xCenters(i)+edgeLength/2,yCenters(i)+edgeLength/2,zCenters(i)];
        vertices{i}(4,:) = [xCenters(i)-edgeLength/2,yCenters(i)+edgeLength/2,zCenters(i)];
    elseif i == 3
        vertices{i} = zeros(4,3);
        vertices{i}(1,:) = [xCenters(i)-boxDimensions(1)/2,yCenters(i)-boxDimensions(2)/2,zCenters(i)];
        vertices{i}(2,:) = [xCenters(i)+boxDimensions(1)/2,yCenters(i)-boxDimensions(2)/2,zCenters(i)];
        vertices{i}(3,:) = [xCenters(i)+boxDimensions(1)/2,yCenters(i)+boxDimensions(2)/2,zCenters(i)];
        vertices{i}(4,:) = [xCenters(i)-boxDimensions(1)/2,yCenters(i)+boxDimensions(2)/2,zCenters(i)];
    else
        vertices{i} = zeros(4,3);
        vertices{i}(1,:) = [xCenters(i)-angledBoxDimensions(1)/2,yCenters(i)-angledBoxDimensions(2)/2,zCenters(i)];
        vertices{i}(2,:) = [xCenters(i)+angledBoxDimensions(1)/2,yCenters(i)-angledBoxDimensions(2)/2,zCenters(i)];
        vertices{i}(3,:) = [xCenters(i)+angledBoxDimensions(1)/2,yCenters(i)+angledBoxDimensions(2)/2,zCenters(i)];
        vertices{i}(4,:) = [xCenters(i)-angledBoxDimensions(1)/2,yCenters(i)+angledBoxDimensions(2)/2,zCenters(i)];
    end
    
    rotms{i} = eul2rotm(flip(angles{i}));
    center = [xCenters(i),yCenters(i),zCenters(i)]';
    for j = 1:4
        vertices{i}(j,:) = center' + (rotms{i}*(vertices{i}(j,:)'-center))';
    end
    
    % store values for constraints
    pCOMrel{i} = rotms{i}*pCOM;
    xMins(i) = min(vertices{i}(:,1)) + radius_platform;
    yMins(i) = min(vertices{i}(:,2)) + radius_platform;
    zMins(i) = min(vertices{i}(:,3));
    xMaxs(i) = max(vertices{i}(:,1)) - radius_platform;
    yMaxs(i) = max(vertices{i}(:,2)) - radius_platform;
    zMaxs(i) = max(vertices{i}(:,3));

    % plot
    fill3(vertices{i}(:,1), vertices{i}(:,2), vertices{i}(:,3), [0.7,0.7,0.7]);
    xlabel('x'); ylabel('y'); zlabel('z'); 
    hold on
    
end
axis square
axis equal

%% add obstacles
nObstacles = 0;
% random cubes(s)
lengthObstacles = 0.1;
posObstacles = cell(nObstacles);
xMinsObstacles = zeros(nObstacles,1);
yMinsObstacles = zeros(nObstacles,1);
zMinsObstacles = zeros(nObstacles,1);
xMaxsObstacles = zeros(nObstacles,1);
yMaxsObstacles = zeros(nObstacles,1);
zMaxsObstacles = zeros(nObstacles,1);
facesObstacles = cell(nObstacles);
for i = 1:nObstacles
    posObstacles{i} = [xCenters(3),yCenters(3),zCenters(3)+0.1]';
    

    xMinsObstacles(i) = posObstacles{i}(1)-lengthObstacles/2;
    yMinsObstacles(i) = posObstacles{i}(2)-lengthObstacles/2;
    zMinsObstacles(i) = posObstacles{i}(3)-lengthObstacles/2;
    xMaxsObstacles(i) = posObstacles{i}(1)+lengthObstacles/2;
    yMaxsObstacles(i) = posObstacles{i}(2)+lengthObstacles/2;
    zMaxsObstacles(i) = posObstacles{i}(3)+lengthObstacles/2;
    
    facesObstacles{i} = zeros(3,4,6); % 3 coord/point, 4 points/face, 6 faces/obs
    % face 1
    facesObstacles{i}(:,1,1) = [posObstacles{i}(1)-lengthObstacles/2,posObstacles{i}(2)-lengthObstacles/2,posObstacles{i}(3)-lengthObstacles/2];
    facesObstacles{i}(:,2,1) = [posObstacles{i}(1)+lengthObstacles/2,posObstacles{i}(2)-lengthObstacles/2,posObstacles{i}(3)-lengthObstacles/2];
    facesObstacles{i}(:,3,1) = [posObstacles{i}(1)+lengthObstacles/2,posObstacles{i}(2)+lengthObstacles/2,posObstacles{i}(3)-lengthObstacles/2];
    facesObstacles{i}(:,4,1) = [posObstacles{i}(1)-lengthObstacles/2,posObstacles{i}(2)+lengthObstacles/2,posObstacles{i}(3)-lengthObstacles/2];
    % face 2
    facesObstacles{i}(:,1,2) = [posObstacles{i}(1)-lengthObstacles/2,posObstacles{i}(2)-lengthObstacles/2,posObstacles{i}(3)+lengthObstacles/2];
    facesObstacles{i}(:,2,2) = [posObstacles{i}(1)+lengthObstacles/2,posObstacles{i}(2)-lengthObstacles/2,posObstacles{i}(3)+lengthObstacles/2];
    facesObstacles{i}(:,3,2) = [posObstacles{i}(1)+lengthObstacles/2,posObstacles{i}(2)+lengthObstacles/2,posObstacles{i}(3)+lengthObstacles/2];
    facesObstacles{i}(:,4,2) = [posObstacles{i}(1)-lengthObstacles/2,posObstacles{i}(2)+lengthObstacles/2,posObstacles{i}(3)+lengthObstacles/2];
    % face 3
    facesObstacles{i}(:,1,3) = [posObstacles{i}(1)-lengthObstacles/2,posObstacles{i}(2)-lengthObstacles/2,posObstacles{i}(3)-lengthObstacles/2];
    facesObstacles{i}(:,2,3) = [posObstacles{i}(1)+lengthObstacles/2,posObstacles{i}(2)-lengthObstacles/2,posObstacles{i}(3)-lengthObstacles/2];
    facesObstacles{i}(:,3,3) = [posObstacles{i}(1)+lengthObstacles/2,posObstacles{i}(2)-lengthObstacles/2,posObstacles{i}(3)+lengthObstacles/2];
    facesObstacles{i}(:,4,3) = [posObstacles{i}(1)-lengthObstacles/2,posObstacles{i}(2)-lengthObstacles/2,posObstacles{i}(3)+lengthObstacles/2];
    % face 4
    facesObstacles{i}(:,1,4) = [posObstacles{i}(1)-lengthObstacles/2,posObstacles{i}(2)+lengthObstacles/2,posObstacles{i}(3)-lengthObstacles/2];
    facesObstacles{i}(:,2,4) = [posObstacles{i}(1)+lengthObstacles/2,posObstacles{i}(2)+lengthObstacles/2,posObstacles{i}(3)-lengthObstacles/2];
    facesObstacles{i}(:,3,4) = [posObstacles{i}(1)+lengthObstacles/2,posObstacles{i}(2)+lengthObstacles/2,posObstacles{i}(3)+lengthObstacles/2];
    facesObstacles{i}(:,4,4) = [posObstacles{i}(1)-lengthObstacles/2,posObstacles{i}(2)+lengthObstacles/2,posObstacles{i}(3)+lengthObstacles/2];
    % face 5
    facesObstacles{i}(:,1,5) = [posObstacles{i}(1)-lengthObstacles/2,posObstacles{i}(2)-lengthObstacles/2,posObstacles{i}(3)-lengthObstacles/2];
    facesObstacles{i}(:,2,5) = [posObstacles{i}(1)-lengthObstacles/2,posObstacles{i}(2)+lengthObstacles/2,posObstacles{i}(3)-lengthObstacles/2];
    facesObstacles{i}(:,3,5) = [posObstacles{i}(1)-lengthObstacles/2,posObstacles{i}(2)+lengthObstacles/2,posObstacles{i}(3)+lengthObstacles/2];
    facesObstacles{i}(:,4,5) = [posObstacles{i}(1)-lengthObstacles/2,posObstacles{i}(2)-lengthObstacles/2,posObstacles{i}(3)+lengthObstacles/2];
    % face 6
    facesObstacles{i}(:,1,6) = [posObstacles{i}(1)+lengthObstacles/2,posObstacles{i}(2)-lengthObstacles/2,posObstacles{i}(3)-lengthObstacles/2];
    facesObstacles{i}(:,2,6) = [posObstacles{i}(1)+lengthObstacles/2,posObstacles{i}(2)+lengthObstacles/2,posObstacles{i}(3)-lengthObstacles/2];
    facesObstacles{i}(:,3,6) = [posObstacles{i}(1)+lengthObstacles/2,posObstacles{i}(2)+lengthObstacles/2,posObstacles{i}(3)+lengthObstacles/2];
    facesObstacles{i}(:,4,6) = [posObstacles{i}(1)+lengthObstacles/2,posObstacles{i}(2)-lengthObstacles/2,posObstacles{i}(3)+lengthObstacles/2];

    for face = 1:6
        fill3(facesObstacles{i}(1,:,face), facesObstacles{i}(2,:,face), facesObstacles{i}(3,:,face), 'r');
        hold on
    end
end

figure(1)
plot3(xCenters(start_region),yCenters(start_region),zCenters(start_region),'g*')
hold on
plot3(xCenters(goal_region),yCenters(goal_region),zCenters(goal_region),'k*')
hold on

%% save info about environment
env.nFootstepRegions = nFootstepRegions;
env.start_region = start_region;
env.goal_region = goal_region;
env.pCOMrel = pCOMrel;
env.vertices = vertices;
env.rotms = rotms;
env.facesObstacles = facesObstacles;
env.g = g;

%% MIP parameters
nPoints = 3;
nHops = 2;

CYawBounds = [-pi,  -7*pi/8; ...
              -7*pi/8,  -pi/8; ...
              -pi/8,     pi/8; ...
               pi/8,     7*pi/8; ...
              7*pi/8,    pi]; 
SYawBounds = [-pi, -5*pi/8; ...
              -5*pi/8,  -3*pi/8; ...
              -3*pi/8,   3*pi/8; ...
               3*pi/8,    5*pi/8; ...
               5*pi/8,    pi];
L = 1;
U = 2;
TBounds = [0,   1/3; ...
           1/3, 2/3];

%% Presolve: enumerate options and eliminate bad ones

disp('Presolve')
nContactOptions = nFootstepRegions^(nHops - 1);
disp(['Number of options before presolve: ',num2str(nContactOptions)])

tic
x = 1:nFootstepRegions;
K = nHops-1;
C = cell(K,1);
[C{:}] = ndgrid(x);
y = cellfun(@(x){x(:)},C);
contactOptions = [y{:}];

goodContactOptions = [];

% reachability
max_jump = 1; 
for i = 1:size(contactOptions,1)
    reachable = true;
    j = contactOptions(i,1);
    k = contactOptions(i,end);
    if j ~= start_region
        if norm([xCenters(j), yCenters(j), zCenters(j)] - [xCenters(start_region), yCenters(start_region), zCenters(start_region)]) >= max_jump
            reachable = false;
        end
    end
    if k ~= goal_region
        if norm([xCenters(goal_region), yCenters(goal_region), zCenters(goal_region)] - [xCenters(k), yCenters(k), zCenters(k)]) >= max_jump
            reachable = false;
        end
    end
    for hop = 1:nHops-2
        j = contactOptions(i,hop);
        k = contactOptions(i,hop+1);
        if j ~= k
            if norm([xCenters(j), yCenters(j), zCenters(j)] - [xCenters(k), yCenters(k), zCenters(k)]) >= max_jump
                reachable = false;
            end
        end
    end
    if reachable
        goodContactOptions = [goodContactOptions;contactOptions(i,:)];
    end
end

goodContactOptionsUnique = cell(nHops-1,1);
nContactOptionsEachHop = zeros(nHops-1,1);
for i = 1:nHops-1
    goodContactOptionsUnique{i} = unique(goodContactOptions(:,i));
    nContactOptionsEachHop(i) = length(goodContactOptionsUnique{i});
end

toc
nContactOptions = 1;
for i = 1:nHops-1
    nContactOptions = nContactOptions * nContactOptionsEachHop(i);
end
disp(['Number of options after presolve: ',num2str(nContactOptions)])

% return

%% MIP variables
qCOM = sdpvar(3,nPoints,nHops,'full');  % COM position
if nObstacles ~= 0
    zCOM = binvar(6,nPoints*nHops,nObstacles, 'full'); % binaries to restrict COM position
end

qTD = sdpvar(6,nHops+1,'full');    % Contact points
zTD = binvar(sum(nContactOptionsEachHop),1,'full'); % binaries to restrict footstep positions

qdotTO = sdpvar(6,nHops+1,'full'); % [take off velocity]

Tair = sdpvar(1,nHops,'full');   % time spent in air for each hop 
T2 = sdpvar(1,nHops,'full');
zT2 = binvar(2,nHops,'full'); % 2 linear pieces

F = sdpvar(6,nHops+1,'full');

C = sdpvar(1,nHops,'full'); % piecewise cos
zC = binvar(5,nHops,'full'); % binaries for piecewise cos
S = sdpvar(1,nHops,'full'); % piecewise sin
zS = binvar(5,nHops,'full'); % binaries piecewise sin

%% constraints
constr = [];

% start and end constraints
constr = [constr, ...
          qTD(:,1) == [angles{start_region}(1),angles{start_region}(2),yaw_init, ...
                       xCenters(start_region),yCenters(start_region),zCenters(start_region)]', ... % Start point
          qCOM(:,1,1) == qTD(4:6,1) + pCOMrel{start_region},...
          qTD(1,end) == angles{goal_region}(1), ...
          qTD(2,end) == angles{goal_region}(2), ...
          qTD(4,end) >= xMins(goal_region), ... % Constrain last hop to final platform
          qTD(4,end) <= xMaxs(goal_region), ...
          qTD(5,end) >= yMins(goal_region), ...
          qTD(5,end) <= yMaxs(goal_region), ...
          qdotTO(:,end) == [0;0;0;0;0;0], ...
          qCOM(:,end,nHops) == qTD(4:6,end) + pCOMrel{goal_region},...
          pCOMrel{goal_region}(1)*(qTD(4,end)-xCenters(goal_region)) + ...
              pCOMrel{goal_region}(2)*(qTD(5,end)-yCenters(goal_region)) + ...
              pCOMrel{goal_region}(3)*(qTD(6,end)-zCenters(goal_region)) == 0
          ];

% piecewise linear approximation of T^2
constr = [constr, ...
          0 <= Tair(:) <= 0.55, ...
          0 <= T2(:) <= 0.55^2];
for i = 1:nHops
    T2_1 = implies(zT2(1,i), [TBounds(1,L) <= Tair(i) <= TBounds(1,U), T2(i) == 1/3*Tair(i)]);
    T2_2 = implies(zT2(2,i), [TBounds(2,L) <= Tair(i) <= TBounds(2,U), T2(i) == Tair(i) - 2/9]);
    constr = [constr, ...
              T2_1, T2_2, sum(zT2(:,i)) == 1]; 
end

% constrain qTD
constr = [constr, ...
          qTD(1,1:nHops) <= pi/4, ...
          qTD(1,1:nHops) >= -pi/4, ...
          qTD(2,1:nHops) <= pi/4, ...
          qTD(2,1:nHops) >= -pi/4, ...
          qTD(4,1:nHops) >= min(xMins), ...
          qTD(4,1:nHops) <= max(xMaxs), ...
          qTD(5,1:nHops) >= min(yMins), ...
          qTD(5,1:nHops) <= max(yMaxs), ...
          qTD(6,:) >= min(zMins), ...
          qTD(6,:) <= max(zMaxs)
          ];
 
% constraint qCOM
for i = 1:nHops
    for j = 1:nPoints
        constr = [constr, ...
                  qCOM(:,j,i) >= [min(xMins); min(yMins); min(zMins)], ...
                  qCOM(:,j,i) <= [max(xMaxs); max(yMaxs); max(zMaxs) + 1]
                  ];
    end
end
 
% constrain qdotTO
for i = 1:nHops+1
    constr = [constr, ...
              qdotTO(:,i) >= [-5; -5; -5; -5; -5; 0], ...
              qdotTO(:,i) <= [5; 5; 5; 5; 5; 5]];
end
 
% flight kinematic constraints
for i = 1:nHops
    constr = [constr, ...
              qTD(1:3,i+1) == qTD(1:3,i) + qdotTO(1:3,i)*Tair(i)];
           
    for j = 1:nPoints-1
        constr = [constr, ...
                  qCOM(:,j+1,i) == qCOM(:,1,i) + qdotTO(4:6,i)*j*Tair(i)/(nPoints-1) - g(4:6)*(j/(nPoints-1))^2 * T2(i) / 2];
    end
    if i ~= nHops
        constr = [constr, ...
                  qCOM(:,1,i+1) == qCOM(:,end,i)];
    end
end
 
% piecewise sinusoids
for i = 1:nHops
    % constrain yaw 
    constr = [constr, ...
              qTD(3,i) >= -pi, ....
              qTD(3,i) <= pi];
    % constrain C and S to [-1,1]
    constr = [constr, ...
              S(i) <= 1, ...
              S(i) >= -1, ...
              C(i) <= 1, ...
              C(i) >= -1];
    % piecewise cos
    C1 = implies(zC(1,i),[CYawBounds(1,L) <= qTD(3,i) <= CYawBounds(1,U), C(i) == -1]);
    C2 = implies(zC(2,i),[CYawBounds(2,L) <= qTD(3,i) <= CYawBounds(2,U), C(i) == 8/(3*pi)*qTD(3,i) + 4/3]);
    C3 = implies(zC(3,i),[CYawBounds(3,L) <= qTD(3,i) <= CYawBounds(3,U), C(i) == 1]);
    C4 = implies(zC(4,i),[CYawBounds(4,L) <= qTD(3,i) <= CYawBounds(4,U), C(i) == -8/(3*pi)*qTD(3,i) + 4/3]);
    C5 = implies(zC(5,i),[CYawBounds(5,L) <= qTD(3,i) <= CYawBounds(5,U), C(i) == -1]);
    constr = [constr, ...
              C1, C2, C3, C4, C5, sum(zC(:,i)) == 1];
    % piecewise sin
    S1 = implies(zS(1,i),[SYawBounds(1,L) <= qTD(3,i) <= SYawBounds(1,U), S(i) == -8/(3*pi)*qTD(3,i) - 8/3]);
    S2 = implies(zS(2,i),[SYawBounds(2,L) <= qTD(3,i) <= SYawBounds(2,U), S(i) == -1]);
    S3 = implies(zS(3,i),[SYawBounds(3,L) <= qTD(3,i) <= SYawBounds(3,U), S(i) == 8/(3*pi)*qTD(3,i)]);
    S4 = implies(zS(4,i),[SYawBounds(4,L) <= qTD(3,i) <= SYawBounds(4,U), S(i) == 1]);
    S5 = implies(zS(5,i),[SYawBounds(5,L) <= qTD(3,i) <= SYawBounds(5,U), S(i) == -8/(3*pi)*qTD(3,i) + 8/3]);
    constr = [constr, ...
              S1, S2, S3, S4, S5, sum(zS(:,i)) == 1];
end

% dynamics
i = 1;
constr = [constr, ...
          qdotTO(:,i) == h * ... 
            ([rotms{start_region},zeros(3); ...
              zeros(3),rotms{start_region}]*inv(H)*F(:,i) - g)];
i = nHops+1;
constr = [constr, ...
          qdotTO(:,i) == qdotTO(:,i-1) - g*Tair(i-1) + h * ... 
            ([rotms{goal_region}*[C(i-1),-S(i-1),0;S(i-1),C(i-1),0;0,0,1],zeros(3); ...
              zeros(3),rotms{goal_region}*[C(i-1),-S(i-1),0;S(i-1),C(i-1),0;0,0,1]]*inv(H)*F(:,i) - g)];

for i = 2:nHops
    if i ~= 2
        step = sum(nContactOptionsEachHop(1:i-2));
    else
        step = 0;
    end
    for j = 1:nContactOptionsEachHop(i-1)
            plat = goodContactOptionsUnique{i-1}(j);     
            constr = [constr, implies(zTD(step+j), ...
                      qdotTO(:,i) == qdotTO(:,i-1) - g*Tair(i-1) + h * ... 
                      ([rotms{plat}*[C(i-1),-S(i-1),0;S(i-1),C(i-1),0;0,0,1],zeros(3); ...
                      zeros(3),rotms{plat}*[C(i-1),-S(i-1),0;S(i-1),C(i-1),0;0,0,1]]*inv(H)*F(:,i) - g))];
            if plat == goal_region
                constr = [constr,implies(zTD(step+j), ...
                          qdotTO(:,i) == zeros(6,1))];
            end
    end
    constr = [constr, ...
              sum(zTD(step+1:step+nContactOptionsEachHop(i-1))) == 1];
end

% FWP
for i = 1:nHops+1
    constr = [constr, ...
              F(:,i) <= fzMAX*ones(6,1),...
              F(:,i) >= -fzMAX*ones(6,1)]; % for implies
    constr = [constr, ...
              FWP.A * scale * F(:,i) <= FWP.b];
end
 
% obstacle constraints 
for i = 1:nObstacles
    for j = 1:nHops
        for k = 1:nPoints
        constr = [constr,...
                  implies(0==zCOM(1,(j-1)*nPoints+k,i),qCOM(1,k,j) <= xMinsObstacles(i) - radius),...
                  implies(0==zCOM(2,(j-1)*nPoints+k,i),qCOM(1,k,j) >= xMaxsObstacles(i) + radius),...
                  implies(0==zCOM(3,(j-1)*nPoints+k,i),qCOM(2,k,j) <= yMinsObstacles(i) - radius),...
                  implies(0==zCOM(4,(j-1)*nPoints+k,i),qCOM(2,k,j) >= yMaxsObstacles(i) + radius),...
                  implies(0==zCOM(5,(j-1)*nPoints+k,i),qCOM(3,k,j) <= zMinsObstacles(i) - radius),...
                  implies(0==zCOM(6,(j-1)*nPoints+k,i),qCOM(3,k,j) >= zMaxsObstacles(i) + radius),...
                  zCOM(1,(j-1)*nPoints+k,i) + zCOM(2,(j-1)*nPoints+k,i) + zCOM(3,(j-1)*nPoints+k,i) ...
                   + zCOM(4,(j-1)*nPoints+k,i) + zCOM(5,(j-1)*nPoints+k,i) + zCOM(6,(j-1)*nPoints+k,i) == 5];
        end
    end
end

for j = 2:nHops
    if i ~= 2
        step = sum(nContactOptionsEachHop(1:j-2));
    else
        step = 0;
    end
    for opt = 1:nContactOptionsEachHop(j-1)
        i = goodContactOptionsUnique{j-1}(opt);
        this_constr = [qTD(4,j) >= xMins(i),qTD(4,j) <= xMaxs(i), ...
                       qTD(5,j) >= yMins(i),qTD(5,j) <= yMaxs(i), ...
                       pCOMrel{i}(1)*(qTD(4,j)-xCenters(i)) + ...
                       pCOMrel{i}(2)*(qTD(5,j)-yCenters(i)) + ...
                       pCOMrel{i}(3)*(qTD(6,j)-zCenters(i)) == 0,...
                       qCOM(:,1,j) == qTD(4:6,j) + pCOMrel{i}, ...
                       qTD(1,j) == angles{i}(1), ...
                       qTD(2,j) == angles{i}(2)
                       ];
        constr = [constr, ...
                  implies(zTD(step+opt),this_constr)];
    end
end

% box constraint
limits = [0.1,0.1,0.1,0.45,0.3,0.35]';
i = 1;
constr = [constr, ...
          h/2 * qdotTO(:,i) <= [rotms{start_region},zeros(3); ...
                                zeros(3),rotms{start_region}]*limits, ...
         -h/2 * qdotTO(:,i) <= [rotms{start_region},zeros(3); ...
                                 zeros(3),rotms{start_region}]*limits];
i = nHops+1;
constr = [constr, ...
          h/2 * (qdotTO(:,i-1)-g*Tair(i-1) + qdotTO(:,i)) <= [rotms{goal_region}*[C(i-1),-S(i-1),0;S(i-1),C(i-1),0;0,0,1],zeros(3); ...
           zeros(3),rotms{goal_region}*[C(i-1),-S(i-1),0;S(i-1),C(i-1),0;0,0,1]]*limits, ...
          -h/2 * (qdotTO(:,i-1)-g*Tair(i-1) + qdotTO(:,i)) <= [rotms{goal_region}*[C(i-1),-S(i-1),0;S(i-1),C(i-1),0;0,0,1],zeros(3); ...
           zeros(3),rotms{goal_region}*[C(i-1),-S(i-1),0;S(i-1),C(i-1),0;0,0,1]]*limits];

for i = 2:nHops
    if i ~= 2
        step = sum(nContactOptionsEachHop(1:i-2));
    else
        step = 0;
    end
    for j = 1:nContactOptionsEachHop(i-1)
        plat = goodContactOptionsUnique{i-1}(j);
        if plat ~= goal_region
            constr = [constr, implies(zTD(step+j), ...
                      [h/2 * (qdotTO(:,i-1)-g*Tair(i-1) + qdotTO(:,i)) <= [rotms{j}*[C(i),-S(i),0;S(i),C(i),0;0,0,1],zeros(3); ...
                       zeros(3),rotms{j}*[C(i),-S(i),0;S(i),C(i),0;0,0,1]]*limits, ...
                      -h/2 * (qdotTO(:,i-1)-g*Tair(i-1) + qdotTO(:,i)) <= [rotms{j}*[C(i),-S(i),0;S(i),C(i),0;0,0,1],zeros(3); ...
                       zeros(3),rotms{j}*[C(i),-S(i),0;S(i),C(i),0;0,0,1]]*limits])];
        end
    end
end

%% cost function

objective = 0;
for i = 1:nHops+1
    objective = objective + F(:,i)' * diag([1e3,1e3,1e3,1e1,1e2,2e1]) * F(:,i);
end

%% solve MIP

options = sdpsettings('verbose',2,'solver','GUROBI','debug',1,'gurobi.presolve',2,'gurobi.ScaleFlag',3,'gurobi.TimeLimit',10);
sol = optimize(constr,objective,options);

%% plot

plot3(value(qTD(4,:)), value(qTD(5,:)), value(qTD(6,:)),'b*');
hold on

zCON = ones(3,nHops+1);
zCON(2:3,:) = zeros(2,nHops+1);

for i = 1:nHops
    plot3(value(qCOM(1,:,i)), value(qCOM(2,:,i)), value(qCOM(3,:,i)),'g*-');
    hold on
end

axis equal

%% Save Trajectory in Format for NLP

traj.qTD = qTD;
traj.qCOM = qCOM;
traj.qdotTO = qdotTO;
traj.F = F;
traj.Tair = Tair;

if size(zTD,2) < 2
    nCols = nHops;
    zTD_copy = value(zTD);

    zTD = zeros(nFootstepRegions,nHops);

    zTD(1,1) = 1;
    step = 0;
    for i = 2:nHops
        if i ~= 2
            step = sum(nContactOptionsEachHop(1:i-2));
        end
        for j = 1:nContactOptionsEachHop(i-1)
            if zTD_copy(step+j) == 1
                zTD(goodContactOptionsUnique{i-1}(j),i) = 1;
                break;
            end
        end
    end
end

traj.zTD = zTD;

saveMipSolnForNLP(traj,env,nHops,nPoints,h,pNominal,'Example');