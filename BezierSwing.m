function pf = BezierSwing(pf,pos,eul)  
% 1. transform foot positions to body frame
% 2. calculate swing trajectories using bezier curves
% 3. output: first 3 rows body frame (for controller), last 3 rows, world frame (for visual)?

pf_body = zeros(size(pf));
for i = 1:length(pf)
    Rbody = eul2Rot(eul(:,i));
    for l = 1:4
        pf_body(3*(l-1)+1:3*l,i) = Rbody'*(pf(3*(l-1)+1:3*l,i) - pos(:, i));
    end
end

% parameters
Dim = 3; % xyz
n = 2; % order of polynomial

for i = 1:length(pf)-1
    for l = 1:4
        % find N
        if isequal(pf(3*(l-1)+1:3*l,i),zeros(3,1))
            N = 1;
            while i+N < length(pf) && isequal(pf(3*(l-1)+1:3*l,i+N),zeros(3,1))
                N = N + 1;
            end
            N = N + 1;
            pf_swing_body = zeros(Dim,N+1);
            alpha = zeros(Dim,n+1);
            alpha(:,1) = pf_body(3*(l-1)+1:3*l,i-1);
            pf_avg = 1/2*(pf_body(3*(l-1)+1:3*l,i-1) + pf_body(3*(l-1)+1:3*l,i-1+N));
            alpha(:,2) =  [pf_avg(1),pf_avg(2),min(-0.05,pf_avg(3)+0.25)]';
            alpha(:,end) = pf_body(3*(l-1)+1:3*l,i-1+N);
            B = zeros(n+1,N+1);
            for ind = 1:N+1
                for j = 1:n+1
                    B(j,ind) = nchoosek(n,(j-1))*(1-(ind-1)/N)^(n-(j-1))*((ind-1)/N)^(j-1);
                end
            end
            pf_swing_body = alpha*B;
            pf_body(3*(l-1)+1:3*l,i-1:i-1+N) = pf_swing_body;
            pf(3*(l-1)+1:3*l,i-1:i-1+N) = ones(size(3*(l-1)+1:3*l,i-1:i-1+N)); % avoid accidental overwrite
        end
    end
end

for i = 1:length(pf)
    Rbody = eul2Rot(eul(:,i));
    for l = 1:4
        pf(3*(l-1)+1:3*l,i) = Rbody*pf_body(3*(l-1)+1:3*l,i) + pos(:,i);
    end
end

pf = [pf;pf_body];

end