%% BenchmarkFullStanceMIP.m
clc; clear; 

add_cost = false;

%% physics
g = [0;0;0; 0;0;9.8];

%% robot specifications
pNominal = [ 0.19, -0.1110, 0;
             0.19,  0.1110, 0;
            -0.19, -0.1110, 0;
            -0.19,  0.1110, 0];
nFeet = length(pNominal(:,1));
pCOM = [0 0 0.18]'; % COM position relative to "contact point"

fzMAX = 400; 
load('FWP/FWP.mat');
scale = 1e-5;
FWP = Polyhedron('V',scale*V_FWP); % need to scale to compute HRep
FWP.minHRep();

h = 2e-1;  
m = 9; % mass in kg 
M = diag([m m m]);
I = diag([11253,36203,42673])*1e-6;
H = [I,zeros(3);
     zeros(3),M];

radius = norm(pNominal(1,:) - pCOM');
radius_platform = norm(pNominal(1,:));

%% footstep regions
nFootstepRegions = 9; % perfect square
start_region = 1; % START
yaw_init = 0;
goal_region = 9; % GOAL
edgeLength = 2*radius_platform + 0.1;
gapLength = 0.1;
[xCenters,yCenters] = meshgrid(0:edgeLength+gapLength:(sqrt(nFootstepRegions)-1)*(edgeLength+gapLength)); 
xCenters = reshape(xCenters,[numel(xCenters),1]);
yCenters = reshape(yCenters,[numel(yCenters),1]);
zCenters = zeros(nFootstepRegions,1);
vertices = cell(nFootstepRegions,1);
angles = cell(nFootstepRegions,1);
rotms = cell(nFootstepRegions,1);
pCOMrel = cell(nFootstepRegions,1);
xMins = zeros(nFootstepRegions,1);
yMins = zeros(nFootstepRegions,1);
xMaxs = zeros(nFootstepRegions,1);
yMaxs = zeros(nFootstepRegions,1);

for i = 1:nFootstepRegions
    vertices{i} = zeros(4,3);
    zCenters(i) = rand/10;
    vertices{i}(1,:) = [xCenters(i)-edgeLength/2,yCenters(i)-edgeLength/2,zCenters(i)];
    vertices{i}(2,:) = [xCenters(i)+edgeLength/2,yCenters(i)-edgeLength/2,zCenters(i)];
    vertices{i}(3,:) = [xCenters(i)+edgeLength/2,yCenters(i)+edgeLength/2,zCenters(i)];
    vertices{i}(4,:) = [xCenters(i)-edgeLength/2,yCenters(i)+edgeLength/2,zCenters(i)];
    
    if i ~= 1 && i ~=nFootstepRegions
        angles{i} = [(rand-0.5)*2/5,(rand-0.5)*2/5,0];
    else
        angles{i} = zeros(1,3);
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
end


%% add obstacles
nObstacles = 2; % max number of obstacles
nEpisodes = 100;
solve_times = zeros(nEpisodes,1);
N_optimal = 0;
N_fails = 0;
for episode = 1:nEpisodes
    % random cubes(s)
    indObstacles = start_region*ones(nObstacles,1);
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
        while (indObstacles(i) == start_region) || (indObstacles(i) == goal_region)
            rv = rand;
            for j = 1:nFootstepRegions
                if rv >= (j-1)/nFootstepRegions && rv <= j/nFootstepRegions
                    indObstacles(i) = j;
                    posObstacles{i} = [xCenters(j),yCenters(j),zCenters(j)+0.1]';
                end
            end
        end
    
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
    end
    
    
    %% MIP parameters
    nPoints = 3;
    nHops = 4;
    nStance = nHops+1;
    
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
    
    % bezier curves
    M  = 5;
    Tst = 0.2;
    Phi1 = zeros(M+2);
    for i = 1:M+2
        for j = 1:M+2
            if i == j && i ~= M+2
                Phi1(i,j) = -1;
            end
            if i+1 == j
                Phi1(i,j) = 1;
            end
            if i == M+2 && j == 1
                Phi1(i,j) = Tst/(M+1);
            end
        end
    end
    Phi2 = zeros(M+3);
    for i = 1:M+3
        for j = 1:M+3
            if i == j && i ~= M+3
                Phi2(i,j) = -1;
            end
            if i+1 == j
                Phi2(i,j) = 1;
            end
            if i == M+3 && j == 1
                Phi2(i,j) = Tst/(M+2);
            end
        end
    end
    
    %% MIP variables
    qCOM = sdpvar(3,nPoints,nHops,'full');  % COM position
    zCOM = binvar(6,nPoints*nHops,nObstacles); % binaries to restrict COM position
    
    qTD = sdpvar(6,nHops+1,'full');    % Contact points
    zTD = binvar(nFootstepRegions,nHops,'full'); % binaries to restrict footstep position
    
    qdotTO = sdpvar(6,nHops+1,'full'); % [take off velocity]
    
    Tair = sdpvar(1,nHops,'full');   % time spent in air for each hop 
    T2 = sdpvar(1,nHops,'full');
    zT2 = binvar(2,nHops,'full'); % 2 linear pieces
    
    C = sdpvar(1,nHops+1,'full'); % piecewise cos
    zC = binvar(5,nHops+1,'full'); % binaries for piecewise cos
    S = sdpvar(1,nHops+1,'full'); % piecewise sin
    zS = binvar(5,nHops+1,'full'); % binaries piecewise sin
    
    alphaWrench = sdpvar(M+1,6,nStance,'full'); % stance Bezier Curves
    alphaTwist = sdpvar(M+2,6,nStance,'full');
    alphaPos = sdpvar(M+3,6,nStance,'full');
    
    %% constraints
    constr = [];
    
    % start and end constraints
    constr = [constr, ...
              qTD(:,1) == [angles{start_region}(1),angles{start_region}(2),yaw_init, ...
                           xCenters(start_region),yCenters(start_region),zCenters(start_region)]', ... % Start point
              qTD(1,end) == angles{goal_region}(1), ...
              qTD(2,end) == angles{goal_region}(2), ...
              qTD(4,end) >= xMins(goal_region), ... % Constrain last hop to goal platform
              qTD(4,end) <= xMaxs(goal_region), ...
              qTD(5,end) >= yMins(goal_region), ...
              qTD(5,end) <= yMaxs(goal_region), ...
              qdotTO(:,end) == [0;0;0;0;0;0]
              ];
    % doing this to avoid small matrix coefficients
    if pCOMrel{goal_region}(1) == 0 && pCOMrel{goal_region}(2) == 0
        constr = [constr, ...
                  pCOMrel{goal_region}(3)*(qTD(6,end)-zCenters(goal_region)) == 0];
    elseif pCOMrel{goal_region}(1) == 0 && pCOMrel{goal_region}(2) ~= 0
        constr = [constr, ...
                  pCOMrel{goal_region}(2)*(qTD(5,end)-yCenters(goal_region)) + ...
                  pCOMrel{goal_region}(3)*(qTD(6,end)-zCenters(goal_region)) == 0];
    elseif pCOMrel{goal_region}(1) ~= 0 && pCOMrel{goal_region}(2) == 0
        constr = [constr, ...
                  pCOMrel{goal_region}(1)*(qTD(4,end)-yCenters(goal_region)) + ...
                  pCOMrel{goal_region}(3)*(qTD(6,end)-zCenters(goal_region)) == 0];
    else
        constr = [constr, ...
                  pCOMrel{goal_region}(1)*(qTD(4,end)-yCenters(goal_region)) + ...
                  pCOMrel{goal_region}(2)*(qTD(5,end)-yCenters(goal_region)) + ...
                  pCOMrel{goal_region}(3)*(qTD(6,end)-zCenters(goal_region)) == 0];
    end
    
    % piecewise linear approximation of T^2
    constr = [constr, ...
              0 <= Tair(:) <= 1, ...
              0 <= T2(:) <= 1];
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
                      qCOM(:,j,i) <= [max(xMaxs); max(yMaxs); max(zMaxs)+1]
                      ];
        end
    end
     
    % constrain qdotTO
    for i = 1:nHops+1
        constr = [constr, ...
                  qdotTO(:,i) >= [-5; -5; -5; -5; -5; 0], ...
                  qdotTO(:,i) <= [5; 5; 5; 5; 5; 5]];
    end
    
    % constrain bezier points
    for i = 1:nStance
        for j = 1:M+3
            constr = [constr, ...
                      alphaPos(j,:,i) <= [pi/4,pi/4,pi,max(xMaxs),max(yMaxs),max(zMaxs)+1],...
                      alphaPos(j,:,i) >= [-pi/4,-pi/4,-pi,min(xMins),min(yMins),min(zMins)]];
        end
        for j = 1:M+2
            constr = [constr, ...
                      alphaTwist(j,:,i) <= [50,50,50,10,10,10],...
                      alphaTwist(j,:,i) >= -[50,50,50,10,10,10]
                      ];
        end
        for j = 1:M+1
            constr = [constr, ...
                      alphaWrench(j,:,i) <= fzMAX*ones(1,6),...
                      alphaWrench(j,:,i) >= -fzMAX*ones(1,6)];
        end
    end
     
    % kinematic constraints
    for i = 1:nHops
        constr = [constr, ...
                  qTD(1:3,i+1) == qTD(1:3,i) + qdotTO(1:3,i)*Tair(i)];
               
        for j = 1:nPoints-1
            constr = [constr, ...
                      qCOM(:,j+1,i) == qCOM(:,1,i) + qdotTO(4:6,i)*j*Tair(i)/(nPoints-1) - g(4:6)*(j/(nPoints-1))^2 * T2(i) / 2];
        end
        % connect phases
        constr = [constr, ...
                  alphaPos(end,4:6,i) == qCOM(:,1,i)', ...
                  alphaTwist(end,:,i) == qdotTO(:,i)'];
    end
     
    % piecewise sinusoids
    for i = 1:nHops+1
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
    % first piece decided by initial condition
    for i = 1:size(CYawBounds,1)
        if CYawBounds(i,L) <= yaw_init && yaw_init  <= CYawBounds(i,U)
            constr = [constr, zC(i,1) == 1];
            for j = 1:size(CYawBounds,1)
                if i ~= j
                    constr = [constr, ...
                              zC(j,1) == 0];
                end
            end
        end
        if SYawBounds(i,L) <= yaw_init && yaw_init  <= SYawBounds(i,U)
            constr = [constr, zS(i,1) == 1];
            for j = 1:size(CYawBounds,1)
                if i ~= j
                    constr = [constr, ...
                              zC(j,1) == 0];
                end
            end
        end
    end
    
    % start and end
    constr = [constr, ...
              alphaPos(1,:,1) == [qTD(1:3,1)',pCOMrel{start_region}'],...
              alphaTwist(1,:,1) == zeros(1,6),...
              alphaPos(end,:,end) == qTD(:,end)' + [zeros(3,1);pCOMrel{goal_region}]',...
              alphaTwist(end,:,end) == zeros(1,6)];
    
    gblock = g(6)*[zeros(5,M+1);ones(1,M+1)];
    for i = 1:nHops+1
        if i == 1
            constr = [constr, ...
                      alphaWrench(1,:,i) == m*g',...
                      alphaWrench(end,1:3,i) == zeros(1,3)
                      ];
        elseif i == nStance
            constr = [constr, ...
                      alphaWrench(1,1:3,i) == zeros(1,3),...
                      alphaWrench(end,:,i) == m*g'];
        else
            constr = [constr, ...
                      alphaWrench(1,1:3,i) == zeros(1,3),...
                      alphaWrench(end,1:3,i) == zeros(1,3)
                      ];
        end
    
        if i == 1
            qdot0 = alphaTwist(1,:,i);
            q0 = alphaPos(1,:,i);
        else
            qdot0 = (qdotTO(:,i-1) - g*Tair(i-1))';
            q0 = ([qTD(1:3,i);qCOM(:,end,i-1)])';
        end
        if i == 1
            R = rotms{start_region}*[C(i),-S(i),0;S(i),C(i),0;0,0,1];
            constr = [constr, ...
                      (M+1)/Tst*Phi1*alphaTwist(:,:,i) == [[R,zeros(3);zeros(3),R]* ...
                      inv(H)*alphaWrench(:,:,i)'-gblock,qdot0']'];
        elseif i ~= nHops+1
            for j = 1:nFootstepRegions
                R = rotms{j}*[C(i),-S(i),0;S(i),C(i),0;0,0,1];
                constr = [constr, ...
                          implies(zTD(j,i),...
                          (M+1)/Tst*Phi1*alphaTwist(:,:,i) == [[R,zeros(3);zeros(3),R]* ...
                          inv(H)*alphaWrench(:,:,i)'-gblock,qdot0']')];
            end
        else
            R = rotms{goal_region}*[C(i),-S(i),0;S(i),C(i),0;0,0,1];
            constr = [constr, ...
                          (M+1)/Tst*Phi1*alphaTwist(:,:,i) == [[R,zeros(3);zeros(3),R]* ...
                          inv(H)*alphaWrench(:,:,i)'-gblock,qdot0']'];
        end
        constr = [constr, ...
                  (M+2)/Tst*Phi2*alphaPos(:,:,i) == [alphaTwist(:,:,i)',q0']'];
        % FWP
        for j = 1:M+1
            constr = [constr, ...
                      FWP.A * scale * alphaWrench(j,:,i)' <= FWP.b];
        end
     
        % binaries
        if i ~= nHops+1
            constr = [constr, ...
                      sum(zTD(:,i))==1];
        end
    
        % constrain pose to bounding box
        if i ~= nStance
            for j = 1:M+3
                for k = 1:nFootstepRegions
                    R = rotms{k}*[C(i),-S(i),0;S(i),C(i),0;0,0,1];
                    constr = [constr, ...
                              implies(zTD(k,i), [...
                              R'*(alphaPos(j,1:3,i)' - [angles{k}(1:2)';qTD(3,i)]) <= [0.05,0.05,0.05]',...
                             -R'*(alphaPos(j,1:3,i)' - [angles{k}(1:2)';qTD(3,i)]) <= [0.05,0.05,0.05]',...
                              R'*(alphaPos(j,4:6,i)' - (qTD(4:6,i) + pCOMrel{k})) <= [0.125,0.075,0.22]',...
                             -R'*(alphaPos(j,4:6,i)' - (qTD(4:6,i) + pCOMrel{k})) <= [0.125,0.075,0.13]'])];
                end
            end
        end
    end
    % i = nStance;
    for j = 1:M+3
        k = goal_region;
        R = rotms{k}*[C(i),-S(i),0;S(i),C(i),0;0,0,1];
        constr = [constr, ...
                  (alphaPos(j,1:3,i)' - [angles{k}(1:2)';qTD(3,i)]) <= [0.05,0.05,0.05]',...
                  (alphaPos(j,1:3,i)' - [angles{k}(1:2)';qTD(3,i)]) <= [0.05,0.05,0.05]',...
                  R'*(alphaPos(j,4:6,i)' - (qTD(4:6,i) + pCOMrel{k})) <= [0.125,0.075,0.22]',...
                 -R'*(alphaPos(j,4:6,i)' - (qTD(4:6,i) + pCOMrel{k})) <= [0.125,0.075,0.13]'];
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
    
    % platform constraints
    for i = 1:nFootstepRegions
        for j = 1:nHops
            constr = [constr, ...
                      implies(zTD(i,j), ...
                      [qTD(4,j) >= xMins(i),qTD(4,j) <= xMaxs(i), ...
                       qTD(5,j) >= yMins(i),qTD(5,j) <= yMaxs(i)])];
            if pCOMrel{i}(1) == 0 && pCOMrel{i}(2) == 0
                constr = [constr, ...
                          implies(zTD(i,j), ...
                          pCOMrel{i}(3)*(qTD(6,j)-zCenters(i)) == 0)];
            elseif pCOMrel{i}(1) == 0 && pCOMrel{i}(2) ~= 0
                constr = [constr, ...
                          implies(zTD(i,j), ...
                          pCOMrel{i}(2)*(qTD(5,j)-yCenters(i)) + ...
                          pCOMrel{i}(3)*(qTD(6,j)-zCenters(i)) == 0)];
            elseif pCOMrel{i}(1) ~= 0 && pCOMrel{i}(2) == 0
                constr = [constr, ...
                          implies(zTD(i,j), ...
                          pCOMrel{i}(1)*(qTD(4,j)-xCenters(i)) + ...
                          pCOMrel{i}(3)*(qTD(6,j)-zCenters(i)) == 0)];
            else
                constr = [constr, ...
                          implies(zTD(i,j), ...
                          pCOMrel{i}(1)*(qTD(4,j)-xCenters(i)) + ...
                          pCOMrel{i}(2)*(qTD(5,j)-yCenters(i)) + ...
                          pCOMrel{i}(3)*(qTD(6,j)-zCenters(i)) == 0)];
            end
        end
    
        if j~=1 && i~=1
                constr = [constr, ...
                          implies(zTD(i,j), ...
                          [qTD(1,j) == angles{i}(1), ...
                           qTD(2,j) == angles{i}(2)])];
        end
    end
    
    % if at goal, qdotTO zero
    for i = 2:nHops
        constr = [constr, ...
                  implies(zTD(goal_region,i),qdotTO(:,i)==zeros(6,1))];
    end
    
    % first stance on start platform
    constr = [constr, ...
              zTD(1,start_region)==1];
    
    %% cost function
    A = 1:nHops+1;
    objective = 0;
    if add_cost
        for i = 1:nHops + 1
            objective = objective + A(i)*sum(alphaWrench(:,6,i));
        end
    end
    
    %% solve MIP
    
    options = sdpsettings('verbose',0,'solver','GUROBI','debug',1,'gurobi.presolve',2,'gurobi.ScaleFlag',3,'gurobi.TimeLimit',10);
    sol = optimize(constr,objective,options);

    solve_times(episode) = sol.solvertime; 

    disp(['Finished episode ',num2str(episode)])
    obst_message = '   Obstacles at ';
    for i = 1:nObstacles
        obst_message = [obst_message,num2str(indObstacles(i))];
        if i ~= nObstacles
            obst_message = [obst_message,' and '];
        else
            obst_message = [obst_message,'.'];
        end
    end
    disp(obst_message)
    if isnan(value(qCOM(1,1)))||isequal(value(qCOM(:,1,1)),zeros(3,1))
        disp('   No solution found within time limit.')
        N_fails = N_fails + 1;
    else
        disp(['   Solve time: ',num2str(sol.solvertime),' s.'])
        N_optimal = N_optimal + 1;
    end
end
disp(' ')
disp(['Average Solve Time: ',num2str(mean(solve_times))])
disp(['Max Solve Time    : ',num2str(max(solve_times))])
disp(['The solver failed to find a solution on ',num2str(100 * N_fails / nEpisodes),' % of the trials.'])
disp(['The solver found the optimal solution on ',num2str(100 * N_optimal / nEpisodes),'% of the trials.'])