function out = saveMipSolnForNLP(traj, env, nHops, nPoints, h, pNominal, filename)

    qTD = value(traj.qTD);
    qCOM = value(traj.qCOM);
    qdotTO = value(traj.qdotTO);
    F = value(traj.F);
    Tair = value(traj.Tair);
    zTD = value(traj.zTD);
    
    nFootstepRegions = env.nFootstepRegions;
    start_region = env.start_region;
    goal_region = env.goal_region;
    pCOMrel = env.pCOMrel;
    vertices = env.vertices;
    rotms = env.rotms;
    facesObstacles = env.facesObstacles;
    g = env.g;
    
    used_less_hops = false;
    for i = 1:nHops
        if norm(value(qdotTO(:,i))) <= 1e-3 
            used_less_hops = true;
            break;
        end
    end
    if used_less_hops
        nHops = i-1;
    end
    
    total_time = h*(nHops+1) + sum(value(Tair(:,1:nHops))); % stance + flight time
    time = 0:0.01:total_time;
    if (time(end) < total_time)
        time(end+1) = total_time;
    end
    
    regionTraj = zeros(1,nHops+1);
    regionTraj(1) = start_region;
    for i = 2:nHops
        for j = 1:nFootstepRegions
            if value(zTD(j,i)) == 1
                regionTraj(i) = j;
            end
        end
    end
    regionTraj(end) = goal_region;
    
    t_list = [];
    t_list = [t_list, 0];
    x_list = [];
    x_list = [x_list, value([qTD(:,1)+[zeros(3,1);pCOMrel{regionTraj(1)}];zeros(6,1)])];
    F_list = [];
    F_list = [F_list,value(F(:,1))];
    
    for j = 1:nHops+1
        t_list(end+1) = t_list(end) + h; % stance time
        pos= value([qTD(4,j),qTD(5,j),qTD(6,j)]') + pCOMrel{regionTraj(j)};
        rpy= value([qTD(1,j),qTD(2,j),qTD(3,j)]');
        vel= value(qdotTO(4:6,j)); 
        rot_vel= value(qdotTO(1:3,j));
        x_list(:,end+1) = [rpy;pos;rot_vel;vel];
        F_list(:,end+1) = value(F(:,j));
        
        if(j <= nHops)
            if(value(Tair(j)) ~= 0)
                for k = 2:nPoints
                    t_list(end+1) = t_list(end) + value(Tair(j)) * 1/(nPoints-1);
                    pos = value(qCOM(:,k,j));
                    rpy = x_list(1:3,end) + 1/(nPoints-1)* ...
                                            (value([qTD(1,j+1),qTD(2,j+1),qTD(3,j+1)]') ...
                                            - value([qTD(1,j),qTD(2,j),qTD(3,j)]'));
                    vel = value(qdotTO(4:6,j)) - g(4:6)*value(Tair(j)) * (k-1)/(nPoints); 
                    rot_vel= value(qdotTO(4:6,j));
                    x_list(:,end+1) = [rpy;pos;rot_vel;vel];
                    F_list(:,end+1) = zeros(6,1);
                end
            end
        end
    end
    xd = interp1(t_list,x_list',time,'linear','extrap')'; % added extrap to avoid nans at end of traj
    Fd = interp1(t_list,F_list',time,'linear','extrap')';
    
    %% u, ctacts, pfd
    
    pfd = [];
    ctacts = [];
    for i = 1:length(xd(1,:))
        for j = 1:nHops+1
            if norm(xd(4:6,i)-(value(qTD(4:6,j)+pCOMrel{regionTraj(j)})))^2 < 1e-6
                ctacts = [ctacts,ones(4,1)];
                pfd = [pfd, value(reshape(rotms{regionTraj(j)}*[cos(qTD(3,j)),-sin(qTD(3,j)),0;sin(qTD(3,j)),cos(qTD(3,j)),0;0,0,1]*pNominal',[12 1]))+repmat(value(qTD(4:6,j)),4,1)];
                plot3(pfd(1,end),pfd(2,end),pfd(3,end),'*r')
                hold on
                plot3(pfd(4,end),pfd(5,end),pfd(6,end),'*r')
                hold on
                plot3(pfd(7,end),pfd(8,end),pfd(9,end),'*r')
                hold on
                plot3(pfd(10,end),pfd(11,end),pfd(12,end),'*r')
                hold on
            end
        end
        if length(pfd(1,:)) < i
            ctacts = [ctacts,zeros(4,1)];
            pfd = [pfd, zeros(12,1)];
        end
    end
    
    %% save trajectory
    nStance = nHops+1;
    filename = ['Trajectories/',filename,'TrajectoryForNLP.mat'];
    save(filename,'time','xd','Fd','pfd','ctacts','F','qdotTO','qTD','qCOM','Tair','nStance','rotms','regionTraj',"vertices","facesObstacles");
    
    out = 1;

end