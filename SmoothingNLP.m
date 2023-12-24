clear
clc

long_stance = true;

addpath(genpath([pwd() '/Visualization']));%,'-end');
addpath(genpath([pwd() '/Utils']));%,'-end')
addpath(genpath([pwd() '/spatial_v2_extended']));%,'-end');
addpath(genpath([pwd() '/Models']));%,'-end')

%% robot specs

h = 10e-3; % 10 ms
m = 9; % mass in kg 
I = diag([11253,36203,42673])*1e-6;
M = diag([m m m]);
H = [I,zeros(3);
     zeros(3),M];
g = [0;0;0;0;0;9.8];

nCtacts = 4;

% for configuration constraint
robot = getMiniCheetahParams();

%% problem setup

% import FWPs
load('FWP/FWP.mat')
scale_FWP = 1e-5;
FWP = Polyhedron('V',scale_FWP*V_FWP);
FWP.minHRep();

% import trajectory
trajname = 'Example';
load(['Trajectories/',trajname,'TrajectoryForNLP.mat'])
use_grfs = true;
add_trot = false;
hold = true;
assert(~(add_trot == true && hold == true));
if ~use_grfs
    trajname = [trajname,'NoGRFs'];
end

% bezier curves
M  = 5;
if long_stance
    Tst = 0.25;
else
    Tst = 0.2;
end
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
Phi3 = cell(1,nStance-1);
Phi4 = cell(1,nStance-1);
for indFlight = 1:nStance-1
    Phi3{indFlight} = zeros(2);
    for i = 1:2
        for j = 1:2
            if i == j && i ~= 2
                Phi3{indFlight}(i,j) = -1;
            end
            if i+1 == j
                Phi3{indFlight}(i,j) = 1;
            end
            if i == 2 && j == 1
                Phi3{indFlight}(i,j) = Tair(indFlight);
            end
        end
    end
    Phi4{indFlight} = zeros(3);
    for i = 1:3
        for j = 1:3
            if i == j && i ~= 3
                Phi4{indFlight}(i,j) = -1;
            end
            if i+1 == j
                Phi4{indFlight}(i,j) = 1;
            end
            if i == 3 && j == 1
                Phi4{indFlight}(i,j) = Tair(indFlight)/2;
            end
        end
    end
end

% MIP solution is "desired trajectory"
mid = ceil(length(qCOM(1,:,1))/2);
alphaWrenchDes = zeros(6,nStance);
alphaTwistDes = zeros(6,nStance);
alphaPosDes = zeros(6,nStance);
alphaTwistFlightDes = zeros(6,nStance-1);
alphaPosFlightDes = zeros(6,nStance-1);
for i = 1:nStance
    alphaWrenchDes(:,i) = F(:,i);
    alphaTwistDes(:,i) = qdotTO(:,i);
    if i ~= nStance
        alphaPosDes(:,i) = [qTD(1:3,i);qCOM(:,1,i)];
        alphaTwistFlightDes(:,i) = qdotTO(:,i) - g*Tair(i)/2;
        alphaPosFlightDes(:,i) = [(qTD(1:3,i)+qTD(1:3,i+1))/2;qCOM(:,mid,i)];
    else
        alphaPosDes(:,i) = [qTD(1:3,i);qCOM(:,end,i-1)];
    end
end

% rotation matrices
R = cell(nStance);
for j = 1:nStance
    R{j} = rotms{regionTraj(j)}*[cos(qTD(3,j)),sin(qTD(3,j)),0;-sin(qTD(3,j)),cos(qTD(3,j)),0;0,0,1];
end

%% optimization variables
alphaWrench = sdpvar(M+1,6,nStance,'full'); % stance Bezier Curves ("terrain" frame)
alphaTwist = sdpvar(M+2,6,nStance,'full'); % world frame
alphaPos = sdpvar(M+3,6,nStance,'full'); % world frame
alphaTwistFlight = sdpvar(2,6,nStance-1,'full'); % flight Bezier Curves
alphaPosFlight = sdpvar(3,6,nStance-1,'full'); 

%% objective
% objective function
A = diag([50 50 20 1 1 1]);
B = diag([50 50 20 1 1 1]);
C = diag([200 50 30 10 10 10]);

% difference in alpha from MIP solution (should be same cost)
objective = 0;
for i = 1:nStance
%     for j = 1:M+1
%         objective = objective + ...
%             (alphaWrench(j,:,i)'-alphaWrenchDes(:,i))'*C*(alphaWrench(j,:,i)'-alphaWrenchDes(:,i));
%     end
%     for j = 1:M+2
%         objective = objective + ...
%             (alphaTwist(j,:,i)'-alphaTwistDes(:,i))'*B*(alphaTwist(j,:,i)'-alphaTwistDes(:,i));
%     end
    for j = 1:M+3
        objective = objective + ...
            (alphaPos(j,:,i)'-alphaPosDes(:,i))'*((j-1)-(M+3)/2)^2*A*(alphaPos(j,:,i)'-alphaPosDes(:,i));
    end
    if i ~= nStance
        objective = objective + ...
            (alphaPosFlight(2,:,i)'-alphaPosFlightDes(:,i))'*0.1*A*(alphaPosFlight(2,:,i)'-alphaPosFlightDes(:,i));
    end
end


%% constraints
constr = [];

% start and end
constr = [constr, ...
          alphaPos(1,:,1) == alphaPosDes(:,1)',...
          alphaTwist(1,:,1) == zeros(1,6),...
          alphaPos(end,:,end) == alphaPosDes(:,end)',...
          alphaTwist(end,:,end) == alphaTwistDes(:,end)'];

% dynamics
gblock = g(6)*[zeros(5,M+1);ones(1,M+1)];
for i = 1:nStance
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
        qdot0 = alphaTwistFlight(end,:,i-1);
        q0 = alphaPosFlight(end,:,i-1);
    end
    constr = [constr, ...
              (M+1)/Tst*Phi1*alphaTwist(:,:,i) == [[R{i},zeros(3);zeros(3),R{i}]* ...
                                    inv(H)*alphaWrench(:,:,i)'-gblock,qdot0']'];
    constr = [constr, ...
              (M+2)/Tst*Phi2*alphaPos(:,:,i) == [alphaTwist(:,:,i)',q0']'];
    if i ~= nStance
        qdot0flight = alphaTwist(end,:,i);
        q0flight = alphaPos(end,:,i);
        constr = [constr, ...
                  1/Tair(i)*Phi3{i}*alphaTwistFlight(:,:,i) == [-g,qdot0flight']';
                  2/Tair(i)*Phi4{i}*alphaPosFlight(:,:,i) == [alphaTwistFlight(:,:,i)',q0flight']'];
    end
    % FWP
    for j = 1:M+1
        constr = [constr, ...
                  FWP.A * scale_FWP * alphaWrench(j,:,i)' <= FWP.b];
    end
end 

% constrain pose (during stance!!!!!!!!!!!)
limits = [0.1,0.1,0.1,0.25,0.15,0.35]'; % adjust as needed to ensure kinematic feasibility
for i = 1:nStance
    for j = 1:M+3
        constr = [constr, ...
                   R{i}'*(alphaPos(j,1:3,i)' - alphaPosDes(1:3,i)) <= limits(1:3)/2,...
                  -R{i}'*(alphaPos(j,1:3,i)' - alphaPosDes(1:3,i)) <= limits(1:3)/2,...
                   R{i}'*(alphaPos(j,4:6,i)' - alphaPosDes(4:6,i)) <= limits(4:6)/2,...
                  -R{i}'*(alphaPos(j,4:6,i)' - alphaPosDes(4:6,i)) <= limits(4:6)/2];
    end
end

%% solve
% supply initial solution
for i = 1:nStance
    for j = 1:M+1
        assign(alphaWrench(j,:,i),alphaWrenchDes(:,i)');
    end
    for j = 1:M+2
        assign(alphaTwist(j,:,i),alphaTwistDes(:,i)');
    end
    for j = 1:M+3
        assign(alphaPos(j,:,i),alphaPosDes(:,i)');
    end
    if i ~= nStance
        assign(alphaTwistFlight(1,:,i),alphaTwistDes(:,i)');
        assign(alphaTwistFlight(2,:,i),alphaTwistDes(:,i+1)');
        assign(alphaPosFlight(1,:,i),alphaPosDes(:,i)');
        assign(alphaPosFlight(2,:,i),alphaPosFlightDes(:,i)');
        assign(alphaPosFlight(3,:,i),alphaPosDes(:,i+1)');
    end
end

options = sdpsettings('verbose',2,'solver','gurobi','debug',1,'showprogress',10,'usex0',1);
sol = optimize(constr,objective,options);

%% Sample bezier curves

n = 1;
Bwrench = cell(nStance,1);
Btwist = cell(nStance,1);
Bpos = cell(nStance,1);
BtwistFlight = cell(nStance-1,1);
BposFlight = cell(nStance-1,1);
phase = 1;
while n <= length(ctacts(1,:))
    ctact_phase = ctacts(:,n);
    stanceLength = 1;
    while n + 1 <= length(ctacts(1,:))
        if isequal(ctact_phase,ctacts(:,n+1))
            stanceLength = stanceLength + 1;
            n = n + 1;
        else
            break;
        end
    end
    n = n + 1;
    Bwrench{phase} = zeros(M+1,stanceLength);
    for i = 1:stanceLength
        for k = 1:M+1
            Bwrench{phase}(k,i) = nchoosek(M,(k-1))*(1-(i-1)/(stanceLength-1))^(M-(k-1))*((i-1)/(stanceLength-1))^(k-1);
        end
    end
    Btwist{phase} = zeros(M+2,stanceLength);
    for i = 1:stanceLength
        for k = 1:M+2
            Btwist{phase}(k,i) = nchoosek(M+1,(k-1))*(1-(i-1)/(stanceLength-1))^(M+1-(k-1))*((i-1)/(stanceLength-1))^(k-1);
        end
    end
    Bpos{phase} = zeros(M+3,stanceLength);
    for i = 1:stanceLength
        for k = 1:M+3
            Bpos{phase}(k,i) = nchoosek(M+2,(k-1))*(1-(i-1)/(stanceLength-1))^(M+2-(k-1))*((i-1)/(stanceLength-1))^(k-1);
        end
    end
    if n > length(ctacts(1,:))
        break;
    end

    flightLength = 1;
    ctact_phase = ctacts(:,n);
    while isequal(ctact_phase,ctacts(:,n+1))
        flightLength = flightLength + 1;
        n = n + 1;
    end
    n = n + 1;
    BtwistFlight{phase} = zeros(2,flightLength);
    BposFlight{phase} = zeros(3,flightLength);
    for i = 1:flightLength
        for k = 1:2
            BtwistFlight{phase}(k,i) = nchoosek(1,(k-1))*(1-(i-1)/(flightLength-1))^(1-(k-1))*((i-1)/(flightLength-1))^(k-1);
        end
        for k = 1:3
            BposFlight{phase}(k,i) = nchoosek(2,(k-1))*(1-(i-1)/(flightLength-1))^(2-(k-1))*((i-1)/(flightLength-1))^(k-1);
        end
    end
    phase = phase + 1;
end


F = cell(1,nStance);
twist = cell(1,nStance);
twistFlight = cell(1,nStance-1);
pos = cell(1,nStance);
posFlight = cell(1,nStance-1);

alphaWrench = value(alphaWrench);
alphaTwist = value(alphaTwist);
alphaPos = value(alphaPos);
alphaTwistFlight = value(alphaTwistFlight);
alphaPosFlight = value(alphaPosFlight);
for i = 1:nStance
    F{i} = Bwrench{i}'*alphaWrench(:,:,i);
    twist{i} = Btwist{i}'*alphaTwist(:,:,i);
    pos{i} = Bpos{i}'*alphaPos(:,:,i);
    if i ~= nStance
        twistFlight{i} = BtwistFlight{i}'*alphaTwistFlight(:,:,i);
        posFlight{i} = BposFlight{i}'*alphaPosFlight(:,:,i);
    end
end

%% build trajs
posTraj = [];
velTraj = [];
FTraj = [];
for i = 1:nStance
    if i ~= nStance
        % stance
        posTraj = [posTraj;pos{i}];
        velTraj = [velTraj;twist{i}];
        FTraj = [FTraj;F{i}]; 
        % flight
        posTraj = [posTraj;posFlight{i}];
        velTraj = [velTraj;twistFlight{i}];
        FTraj = [FTraj;zeros(size(twistFlight{i}))];
    else
        posTraj = [posTraj;pos{i}];
        velTraj = [velTraj;twist{i}];
        FTraj = [FTraj;F{i}]; % to world frame
    end
end
lengthTraj = length(posTraj);
t = 0:h:h*(lengthTraj-1);

posTrajMIP = [];
velTrajMIP = [];
FTrajMIP = [];
for i = 1:nStance
    if i ~= nStance
        % stance
        posTrajMIP = [posTrajMIP;repmat(alphaPosDes(:,i)',[size(pos{i},1)],1)];
        velTrajMIP = [velTrajMIP;repmat(alphaTwistDes(:,i)',[size(twist{i},1)],1)];
        FTrajMIP = [FTrajMIP;repmat(alphaWrenchDes(:,i)',[size(F{i},1)],1)];
        % flight
        posTrajMIP = [posTrajMIP;repmat(alphaPosFlightDes(:,i)',[size(posFlight{i},1)],1)];
        velTrajMIP = [velTrajMIP;repmat(alphaTwistFlightDes(:,i)',[size(twistFlight{i},1)],1)];
        FTrajMIP = [FTrajMIP;zeros(size(twistFlight{i}))];
    else
        posTrajMIP = [posTrajMIP;repmat(alphaPosDes(:,i)',[size(pos{i},1)],1)];
        velTrajMIP = [velTrajMIP;repmat(alphaTwistDes(:,i)',[size(twist{i},1)],1)];
        FTrajMIP = [FTrajMIP;repmat(alphaWrenchDes(:,i)',[size(F{i},1)],1)];
    end
end

%% plot with references
figure(2);
clf
subplot(2,1,1)
plot(t,posTraj(:,1),'r','LineWidth',2)
hold on
plot(t,posTraj(:,2),'g','LineWidth',2)
hold on
plot(t,posTraj(:,3),'b','LineWidth',2)
hold on
plot(t,posTrajMIP(:,1),'r--','LineWidth',2)
hold on
plot(t,posTrajMIP(:,2),'g--','LineWidth',2)
hold on
plot(t,posTrajMIP(:,3),'b--','LineWidth',2)
title('Orientation','FontSize',14)
ylabel('Angle (rad)','FontSize',14)
xlabel('Time (sec)','FontSize',14)
legend('\theta_{NLP}','\phi_{NLP}','\psi_{NLP}','\theta_{MIP}','\phi_{MIP}','\psi_{MIP}');%,'location','eastoutside')
set(gca,'fontname','times')  % Set it to times
set(gca,'fontsize',14) 

subplot(2,1,2)
plot(t,posTraj(:,4),'r','LineWidth',2)
hold on
plot(t,posTraj(:,5),'g','LineWidth',2)
hold on
plot(t,posTraj(:,6),'b')
hold on
plot(t,posTrajMIP(:,4),'r--','LineWidth',2)
hold on
plot(t,posTrajMIP(:,5),'g--','LineWidth',2)
hold on
plot(t,posTrajMIP(:,6),'b--','LineWidth',2)
title('COM Position','FontSize',14)
ylabel('Position (m)','FontSize',14)
xlabel('Time (sec)','FontSize',14)
legend('x_{NLP}','y_{NLP}','z_{NLP}','x_{MIP}','y_{MIP}','z_{MIP}');%,'location','eastoutside')
set(gca,'fontname','times')  % Set it to times
set(gca,'fontsize',14) 

figure(3);
subplot(2,1,1)
plot(t,FTraj(:,1),'r','LineWidth',2)
hold on
plot(t,FTraj(:,2),'g','LineWidth',2)
hold on
plot(t,FTraj(:,3),'b','LineWidth',2)
hold on
plot(t,FTrajMIP(:,1),'r--','LineWidth',2)
hold on
plot(t,FTrajMIP(:,2),'g--','LineWidth',2)
hold on
plot(t,FTrajMIP(:,3),'b--','LineWidth',2)
title('Net Moment','FontSize',14)
ylabel('Moment (N·m)','FontSize',14)
xlabel('Time (sec)','FontSize',14)
legend('n_{x,NLP}','n_{y,NLP}','n_{z,NLP}','n_{x,MIP}','n_{y,MIP}','n_{z,MIP}');%,'location','eastoutside')
set(gca,'fontname','times')  % Set it to times
set(gca,'fontsize',14) 

subplot(2,1,2)
plot(t,FTraj(:,4),'r','LineWidth',2)
hold on
plot(t,FTraj(:,5),'g','LineWidth',2)
hold on
plot(t,FTraj(:,6),'b','LineWidth',2)
hold on
plot(t,FTrajMIP(:,4),'r--','LineWidth',2)
hold on
plot(t,FTrajMIP(:,5),'g--','LineWidth',2)
hold on
plot(t,FTrajMIP(:,6),'b--','LineWidth',2)
title('Net Force','FontSize',14)
ylabel('Force (N)','FontSize',14)
xlabel('Time (sec)','FontSize',14)
legend('F_{x,NLP}','F_{y,NLP}','F_{z,NLP}','F_{x,MIP}','F_{y,MIP}','F_{z,MIP}');%,'location','eastoutside')
set(gca,'fontname','times')  % Set it to times
set(gca,'fontsize',14) 

%% visualization
      
% trajectory
eul = posTraj(:,1:3)';
pos = posTraj(:,4:6)';
qdummy = pfd;

% fill data
graphic_option.show_footloc = true;
graphic_option.show_floor = true;
graphic_option.show_platforms = false;
graphic_option.show_obstacles = true;
graphic_option.show_angled_platforms = true;
graphic_option.provided_vertices = true;
graphic_option.provided_faces = true;
graphic_option.setCameraPos = true;
graphic_option.show_GRF = false;
graphic_option.hide_leg = false;

graphics_data.angled_platforms = vertices;
% graphics_data.obstacles = facesObstacles;

Ni = 1;
graphics_data.eul = eul;
graphics_data.pos = pos;
graphics_data.F = FTraj';
graphics_data.qdummy = qdummy;
graphics_data.ctacts = ctacts;
graphics_data.time = 1:Ni:size(FTraj', 2);
graphics_data.CameraPos = [0,-8,2];
graphics_data.obstacles = facesObstacles;

% call visualize function
out = visualizeMCTrajectory(graphics_data, graphic_option);
