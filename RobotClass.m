%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Class for the mobile robot
% ver 1.0, Kangjie Zhou, 2023/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef RobotClass
    properties
        % motion specs
        traj;
        planned_traj;
        state; % [x;y;theta;v]
        a_lb;
        a_ub;
        w_lb;
        w_ub;
        v_lb;
        v_ub;
        Qr; % noise of motion

        % sensor specs
        sensor_type;
        theta0; % sensor range in angle
        rmin; % sensor range minimum
        rmax; % sensor range maximum

        ranges;
        angles;

        inFOV_hist; 
        is_tracking;
        first;
        pred;

        % for general nonlinear sensors
        h; % y=h(x)
        del_h; % gradient of h
        R; % variance(1-d)/covariance(high-d) for sensor model
        mdim; % dimension of sensor measurement

        % observation
        y; % observation measurement

        % target
        target;
        A;
        sdim; % dimension of state

        % filtering
        % xKF
        P; % estimation covariance
        P_hist;
        % PF
        particles; % array of particle positions [x;y]
        w; % particles' weight
        N; % particles' number
        % GM-PHD
        gmm_num;
        gmm_w;
        gmm_mu;
        gmm_P;
        % cell-PHD
        cell;
        cell_size;

        first_particles;
        first_w;
        hgrid;
        h_particles;
        vir_tar;
        loc_par;
        loc_w;

        tree;
        allstate;

        a_hist;
        
        % map
        map;

        est_pos; % estimated target position
        est_pos_hist; % estimation history

        % path planning
        mpc_hor;
        dt;
        optu;

        % configuration of optimization
        snum;

        % action space
        a_S; % search phase
        a_T; % tracking phase
        int_pts_S;
        int_pts_T;

        value_max;

        % sampling para
        G;
        Pmax

        % performance metrics
        ml_pos;
        ent_pos;
    end

    methods
        function this = RobotClass(inPara)
            this.state = inPara.state;
            this.traj = inPara.traj;
            this.planned_traj = inPara.planned_traj;
            this.a_lb = inPara.a_lb;
            this.a_ub = inPara.a_ub;
            this.w_lb = inPara.w_lb;
            this.w_ub = inPara.w_ub;
            this.v_lb = inPara.v_lb;
            this.v_ub = inPara.v_ub;
            this.Qr = inPara.Qr;

            % sensor specs
            this.R = inPara.R;
            this.h = inPara.h;
            this.del_h = inPara.del_h;
            this.theta0 = inPara.theta0;
            this.rmin = inPara.minrange;
            this.rmax = inPara.maxrange;
            this.mdim = inPara.mdim;

            this.ranges = inPara.ranges;
            this.angles = inPara.angles;

            this.inFOV_hist = inPara.inFOV_hist; 
            this.is_tracking = inPara.is_tracking;
            this.first = inPara.first;
            this.pred = inPara.pred;

            % target
            this.target = inPara.target;
            this.A = [];
            this.sdim = inPara.sdim;

            % filtering
            this.sensor_type = inPara.sensor_type;
            % xKF
            this.est_pos = inPara.est_pos;
            this.P = inPara.P;
            this.est_pos_hist = [];
            this.P_hist = [];
            this.particles = inPara.particles;
            this.w = inPara.w;
            this.N = inPara.N;
            % GM-PHD
            this.gmm_num = inPara.gmm_num;
            this.gmm_w = inPara.gmm_w;
            this.gmm_mu = inPara.gmm_mu;
            this.gmm_P = inPara.gmm_P;

            this.cell = inPara.cell;
            this.cell_size = inPara.cell_size;

            this.first_particles = inPara.first_particles;
            this.first_w = inPara.first_w;
            this.hgrid = inPara.hgrid;
            this.vir_tar = inPara.vir_tar;

            this.tree = inPara.tree;
            this.allstate = inPara.allstate;

            this.a_hist = inPara.a_hist;

            this.map = inPara.map;

            this.a_S = inPara.a_S;
            this.a_T = inPara.a_T;
            this.int_pts_S = inPara.int_pts_S;
            this.int_pts_T = inPara.int_pts_T;

            this.mpc_hor = inPara.mpc_hor;
            this.dt = inPara.dt;
            this.optu = [];
        end

        %% sensor modeling
        % approximate straightline edge of sensor FOV based on current
        % robot state
        function [a,b] = FOV(this,st)
            theta = st(3);
            x0 = st(1);
            y0 = st(2);
            alp1 = theta - this.theta0;
            alp2 = theta + this.theta0;
            a = [sin(alp1),-cos(alp1);-sin(alp2),cos(alp2)]; % [a1;a2]
            b = [x0*sin(alp1)-y0*cos(alp1);-x0*sin(alp2)+y0*cos(alp2)];%[b1;b2];
        end

        % determine if the target is in sensor FOV
%         function flag = inFOV(this,tar_pos)
%             [a,b] = this.FOV(this.state);
%             flag = (a(1,:)*tar_pos-b(1) <= 0) && (a(2,:)*tar_pos-b(2) <= 0)...
%                 && (norm(tar_pos-this.state(1:2)) <= this.r);
%         end

        function flag = inFOV(this,map,z,tar_pos,id)
            tmp = tar_pos(1:2,:) - z(1:2);
            ran = vecnorm(tmp);
            flag1 = ran < this.rmax;
            flag2 = ran > this.rmin;
            flag3 = (tmp(1,:)*cos(z(3))+tmp(2,:)*sin(z(3)))./ran > cos(this.theta0/2);
            flag = flag1.*flag2.*flag3;

            if id
                %
                angle = atan2(tar_pos(2,:)-z(2),tar_pos(1,:)-z(1))-z(3);
                ran_tmp = ran.*flag;
                angle_tmp = angle.*flag;

                Indices = find(flag == 1);
                if ~isempty(Indices)

                    ran_tmp = ran_tmp(Indices);
                    angle_tmp = angle_tmp(Indices);

                    intsectionPts = rayIntersection(map,z(1:3)',angle_tmp,this.rmax,0.55);
                    obs_ran = vecnorm(intsectionPts'-z(1:2));
                    flag4_tmp = ran_tmp > obs_ran;
                    flag4_tmp = flag4_tmp.*~isnan(obs_ran);
                    flag4_tmp = ~flag4_tmp;

                    flag4 = zeros(1,size(flag,2));
                    flag4(Indices) = flag4_tmp;

                    flag = flag1.*flag2.*flag3.*flag4;

                end
                %}

                %{
                angle = atan2(tar_pos(2,:)-z(2),tar_pos(1,:)-z(1))-z(3);
                for ii = 1:size(tar_pos,2)
                    if flag(ii) == 0
                        continue
                    else
                        intsectionPts = rayIntersection(map,z(1:3)',angle(ii),this.rmax,0.55);
                        obs_ran = vecnorm(intsectionPts'-z(1:2));
                        flag4 = ran(ii) > obs_ran;
                        flag4 = flag4.*~isnan(obs_ran);
                        flag4 = ~flag4;
                        flag(ii) = flag(ii)*flag4;
                    end
                end
                %}
            end
            %}
        end

        function flag = inFOV_red(this,map,z,tar_pos,id)
            tmp = tar_pos(1:2,:) - z(1:2);
            ran = vecnorm(tmp);
            flag1 = ran < this.rmax-1;
            flag2 = ran > this.rmin+0;
            flag3 = (tmp(1,:)*cos(z(3))+tmp(2,:)*sin(z(3)))./ran > cos((this.theta0-20/180*pi)/2);
            flag = flag1.*flag2.*flag3;

            if id
                %
                angle = atan2(tar_pos(2,:)-z(2),tar_pos(1,:)-z(1))-z(3);
                ran_tmp = ran.*flag;
                angle_tmp = angle.*flag;

                Indices = find(flag == 1);
                if ~isempty(Indices)

                    ran_tmp = ran_tmp(Indices);
                    angle_tmp = angle_tmp(Indices);

                    intsectionPts = rayIntersection(map,z(1:3)',angle_tmp,this.rmax,0.55);
                    obs_ran = vecnorm(intsectionPts'-z(1:2));
                    flag4_tmp = ran_tmp > obs_ran;
                    flag4_tmp = flag4_tmp.*~isnan(obs_ran);
                    flag4_tmp = ~flag4_tmp;

                    flag4 = zeros(1,size(flag,2));
                    flag4(Indices) = flag4_tmp;

                    flag = flag1.*flag2.*flag3.*flag4;
                end
                %}

                %{
                angle = atan2(tar_pos(2,:)-z(2),tar_pos(1,:)-z(1))-z(3);
                for ii = 1:size(tar_pos,2)
                    if flag(ii) == 0
                        continue
                    else
                        intsectionPts = rayIntersection(map,z(1:3)',angle(ii),this.rmax,0.55);
                        obs_ran = vecnorm(intsectionPts'-z(1:2));
                        flag4 = ran(ii) > obs_ran;
                        flag4 = flag4.*~isnan(obs_ran);
                        flag4 = ~flag4;
                        flag(ii) = flag(ii)*flag4;
                    end
                end
                %}
            end
        end

        %% measurement generation
        % generate a random measurement
        function y = sensorGen(this,fld)
            tar_pos = fld.target.pos;

            % draw FOV and target position. for debug purpose
            %this.drawFOV(this.state,fld,'cur',[0.9290 0.6940 0.1250])
            %hold on
            %plot(fld.target.pos(1),fld.target.pos(2),'b','markers',5,'Marker','*');

            if strcmp(this.sensor_type,'rb')
                % range-bearing sensor
                if this.inFOV(fld.map.occ_map,this.state,tar_pos,1)%&&fld.map.V(ceil(this.state(1)),ceil(this.state(2)),ceil(tar_pos(1)),ceil(tar_pos(2)))
                    y = this.h(tar_pos,this.state)+(mvnrnd([0;0],this.R))';
                else
                    y = [-100;-100];
                end
            elseif strcmp(this.sensor_type,'ran')
                if this.inFOV(fld.map.occ_map,this.state,tar_pos,1)%&&fld.map.V(ceil(z(1)),ceil(z(2)),ceil(tar_pos(1)),ceil(tar_pos(2)))
                    y = this.h(tar_pos,this.state)+normrnd(0,this.R);
                else
                    y = -100;
                end
            elseif strcmp(this.sensor_type,'lin')
                if this.inFOV(fld.map.occ_map,this.state,tar_pos,1)%&&fld.map.V(ceil(z(1)),ceil(z(2)),ceil(tar_pos(1)),ceil(tar_pos(2)))
                    y = this.h(tar_pos,this.state)+(mvnrnd([0;0],this.R))';
                else
                    y = [-100;-100];
                end
            end
        end
        %% filtering
        % Kalman filter
        function this = KF(this,fld)
            % measurement
            y = this.y;

            % target
            tar = fld.target;
            A = tar.A;
            B = tar.B;

            Q = tar.Q;

            % current estimation
            x = this.est_pos;
            P = this.P;

            % sensor
            h = this.h;
            del_h = this.del_h;
            R = this.R;

            % prediction
            x_pred = A*x+B;
            P_pred = A*P*A'+Q;

            % update
            C = del_h(x,this.state(1:2));
            if sum(y-[-100;-100]) ~= 0
                % if an observation is obtained
                K = P_pred*C'/(C*P_pred*C'+R);
                x_next = x_pred+K*(y-h(x_pred,this.state(1:2)));%C*x_pred-(x_pred-this.state(1:2))
                P_next = P_pred-K*C*P_pred;
            else
                x_next = x_pred;
                P_next = P_pred;
            end

            this.est_pos = x_next;
            this.P = P_next;
            this.est_pos_hist = [this.est_pos_hist,x_next];
            this.P_hist = [this.P_hist,P_next];
            % this makes KF compatible with cvxPlanner_scp
            this.gmm_num = 1;
            this.wt = 1;
        end

        % GM-PHD filter
        function [w,mu,P] = GM_PHD(this,fld,sim,tt,ii,state,y)

            R = this.R;
            h = this.h;
            if this.is_tracking
                Q = fld.target.Q_tracking;
            else
                Q = fld.target.Q_search;
            end
            
            % sensor
            del_h = this.del_h;
            
            % used for updating gmm component weights
            alp = ones(1,this.gmm_num);

            w = this.gmm_w;
            mu = this.gmm_mu;
            P = this.gmm_P;
            
            for ii = 1:this.gmm_num
                % current estimation
                P_tmp = this.gmm_P{ii};
                x = this.gmm_mu(:,ii);
                A = eye(2);
                % prediction
                x_pred = x;
                %
                if this.is_tracking %&& ii == 1 %一个/所有组分均值根据观测改变
                    x_pred = [state(1)+y(1)*cos(y(2)+state(3));state(2)+y(1)*sin(y(2)+state(3))];
                end
                %}
                %{
                x_pred_tmp = x_pred;
                while any([1;1] >= x_pred(1:2))||any([49;49] <= x_pred(1:2))||this.map.region_exp(ceil(x_pred(1)*5),ceil(x_pred(2)*5)) < 0.45
                    x_pred = mvnrnd(x_pred_tmp',eye(2))'; 
                end
                %}
                P_pred = A*P_tmp*A'+Q;
                
                % update
                % sensor model linearization
                C = del_h(x_pred,state);
                
                if sum(y-[-100;-100]) ~= 0
                    % if an observation is obtained
                    K = P_pred*C'/(C*P_pred*C'+R);
                    obs_var = y-h(x_pred,state);
                    if abs(mod(obs_var(2),pi*2))>=abs(mod(obs_var(2),-pi*2))
                        obs_var(2) = mod(obs_var(2),-pi*2);
                    else
                        obs_var(2) = mod(obs_var(2),pi*2);
                    end
                    x_next = x_pred+K*obs_var;
                    if any([1;1] >= x_next(1:2))||any([49;49] <= x_next(1:2))
                        x_next = [state(1)+y(1)*cos(y(2)+state(3));state(2)+y(1)*sin(y(2)+state(3))];
                    end
                    P_next = P_pred-K*C*P_pred;

                    M = C*P_pred*C'+R;
                    M = (M + M') / 2;
                    alp(ii) = mvnpdf(obs_var,[0;0],M);
                else
                    x_next = x_pred;
                    P_next = P_pred;

                    M = C*P_pred*C'+R;
                    M = (M + M') / 2;
                    %alp(ii) = 1-mvnpdf(y,h(x_pred,state),M)/mvnpdf(y,y,M);
                    alp(ii) = 1;
                    if this.inFOV(this.map.occ_map,state,x_next,1)
                        alp(ii) = 0.01;
                    end
                end

                x_next_tmp = x_next;
                while any([1;1] >= x_next(1:2))||any([49;49] <= x_next(1:2))||this.map.region_exp(ceil(x_next(1)*5),ceil(x_next(2)*5)) < 0.45
                    x_next = mvnrnd(x_next_tmp',eye(2))'; 
                end
                
                mu(:,ii) = x_next;
                P{ii} = P_next;
            end
            
            % update gmm component weight
            w = w.*alp;
            w = w/sum(w);
        end

        % cell-PHD filter
        function cell = cell_PHD(this,fld,sim,tt,ii,state,y)
            R = this.R;
            h = this.h;
            if this.is_tracking
                Q = fld.target.Q_tracking;
            else
                Q = fld.target.Q_search;
            end
            
            % sensor
            del_h = this.del_h;
            
            % current estimation
            cell = zeros(size(this.cell,1),size(this.cell,2));
            mass = 0;
            
            for ii = 1:size(cell,1)
                for jj = 1:size(cell,2)
                    % prediction
                    %
                    for mm = -1:1
                        for nn = -1:1
                            xx = this.cell_size*(ii + mm)-this.cell_size/2;
                            yy = this.cell_size*(jj + nn)-this.cell_size/2;
                            if xx>=0 && xx<=50 && yy>=0 && yy<=50
                                p = 0.05;
                                cell(ii,jj) = cell(ii,jj) + this.cell(ii+mm,jj+nn)*p;
                            elseif mm == 0 && nn == 0
                                p = 0.6;
                                cell(ii,jj) = cell(ii,jj) + this.cell(ii,jj)*p;
                            end
                        end
                    end
                    %}

                    %cell(ii,jj) = this.cell(ii,jj);

                    % update
                    if sum(y-[-100;-100]) ~= 0
                        % if an observation is obtained
                        obs_var = y-h([ii*this.cell_size-this.cell_size/2;jj*this.cell_size-this.cell_size/2],state);
                        if abs(mod(obs_var(2),pi*2))>=abs(mod(obs_var(2),-pi*2))
                            obs_var(2) = mod(obs_var(2),-pi*2);
                        else
                            obs_var(2) = mod(obs_var(2),pi*2);
                        end

                        cell(ii,jj) = mvnpdf(obs_var,[0;0],R);

                    else
                        if this.inFOV(this.map.occ_map,state,[ii*this.cell_size-this.cell_size/2;jj*this.cell_size-this.cell_size/2],1)
                            cell(ii,jj) = 0;
                        end
                    end

                    if this.map.region_exp(ceil((ii*this.cell_size-this.cell_size/2)*5),ceil((jj*this.cell_size-this.cell_size/2)*5)) < 0.45
                        cell(ii,jj) = 0;
                    end

                    mass = mass + cell(ii,jj)*this.cell_size^2;
                end
            end

            cell = cell/mass;
        end

        % particle filter
        function [particles,w] = PF(this,fld,sim,tt,ii,state,particles,w,y,flag)

            %particles = this.particles;
            R = this.R;
            h = this.h;
            N = size(particles,2);
            if this.is_tracking
                Q = fld.target.Q_tracking;
            else
                Q = fld.target.Q_search;
            end

            % particles = f(particles);
            % particles = (mvnrnd(particles',Q))';

            if flag == 1
                %{
            control = [fld.target.control(tt,ii,1);fld.target.control(tt,ii,2)];

            t = 1;
            particles(1:2,:) = particles(1:2,:) + [control(1).*cos(particles(3,:))*t;control(1).*sin(particles(3,:))*t];
            particles(3,:) = particles(3,:) + control(2)*t;
            particles = (mvnrnd(particles',fld.target.Q))';
                %}
                %particles(1:2,:) = particles(1:2,:) -1 + 2*[rand;rand];
                particles = (mvnrnd(particles',Q))';
            end

            if y ~= -100
                particles(1:2,1) = [state(1)+y(1)*cos(y(2)+state(3));state(2)+y(1)*sin(y(2)+state(3))];
            end

            FOV = this.inFOV(this.map.occ_map,state,particles(1:2,:),1);
            if strcmp(sim.sensor_type,'rb')
                mu = h(particles(1:2,:),state);
                P = normpdf(y(1),mu(1,:),sqrt(R(1,1)));
                %
                P0 = normpdf(y(2),mu(2,:),sqrt(R(2,2)));
                P1 = normpdf(y(2),mu(2,:)+2*pi,sqrt(R(2,2)));
                P2 = normpdf(y(2),mu(2,:)-2*pi,sqrt(R(2,2)));
            else
                P = normpdf(y,h(particles(1:2,:),z),sqrt(R));
                %
                P1 = normpdf(y,h(particles(1:2,:),z)+2*pi,sqrt(R));
                P2 = normpdf(y,h(particles(1:2,:),z)-2*pi,sqrt(R));
                %
            end

            for jj = 1:N
                if any([0;0] > particles(1:2,jj))||any([50;50] < particles(1:2,jj))
                    w(jj) = 0;
                    continue
                end
                if this.map.region_exp(ceil(particles(1,jj)*5),ceil(particles(2,jj)*5)) < 0.45
                    w(jj) = 10^-20;
                    continue
                end
                if y == -100
                    % if the target is outside FOV.
                    %
                    if FOV(jj)%&&fld.map.V(ceil(state(1)),ceil(state(2)),ceil(particles(1,jj)),ceil(particles(2,jj)))
                        w(jj) = 10^-20;
                    else
                        w(jj) = 1;
                    end
                    %}
                else
                    if FOV(jj)%&&fld.map.V(ceil(state(1)),ceil(state(2)),ceil(particles(1,jj)),ceil(particles(2,jj)))
                        if strcmp(sim.sensor_type,'ran')
                            w(jj) = P(jj);
                        end
                        if strcmp(sim.sensor_type,'br')
                            if h(particles(1:2,jj),state) + z(3) <= 0
                                w(jj) = max([P(jj),P1(jj)]);
                            else
                                w(jj) = max([P(jj),P2(jj)]);
                            end
                        end
                        if strcmp(sim.sensor_type,'rb')
                            if mu(2,jj) + state(3) <= 0
                                w(jj) = max([P0(jj),P1(jj)]);
                            else
                                w(jj) = max([P0(jj),P2(jj)]);
                            end
                            w(jj) = w(jj)*P(jj);
                        end
                    else
                        w(jj) = 10^-20;
                    end
                end
            end
            w = w./sum(w);%归一化的粒子权重
            % resampling
            %
            N = this.N;
            ess = 1/sum(w.^2);
            flag = 1;
            if flag == 0||flag == 1%&&ess > 0.5*N)
                M = 1/N;
                U = rand(1)*M;
                new_particles = zeros(2,N);
                tmp_w = w(1);
                i = 1;
                jj = 1;
                while (jj <= N)
                    while (tmp_w < U+(jj-1)*M)
                        i = i+1;
                        tmp_w = tmp_w+w(i);
                    end
                    new_particles(:,jj) = particles(:,i);
                    jj = jj + 1;
                end
                particles = new_particles;
                w = repmat(1/N, 1, N);
            end
%             this.particles = particles;
%             this.w = w;
%             this.N = N;
            %}
        end

        %% GM-PHD-SAT
        function [this,optz] = Planner_GMPHD(this,fld,sim,plan_mode,ps,pt,tt,ii)
            is_tracking = this.is_tracking;
            first = this.first;

            if ~is_tracking
                a = this.a_S;
            else
                a = this.a_T;
            end
            
            id = this.a_hist;

            if ~is_tracking
                if id == 6
                    a(:,7) = [];
                elseif id == 7
                    a(:,6) = [];
                end
            end

            % no target motion model, no prediction

            map = copy(this.map.occ_map);
            inflate(map,0.5);
            map.FreeThreshold = 0.55;

            % planning
            if is_tracking
                v = 0.99;
            elseif first
                if ii < 5
                    v = 1.5;
                else
                    v = 3;
                end
                %v = 3;
            else
                v = 1.8;
            end
   
           
            % 最小距离
            %
            dist_min = 1000;
            for ii = 1:this.gmm_num
                if this.gmm_w(ii) > 0.3
                    dist = norm(this.state(1:2)-this.gmm_mu(:,ii));
                    if dist < dist_min
                        dist_min = dist;
                        id = ii;
                    end
                end
            end
            %}
            % 最大方差
            %{
            P_max = 0;
            for ii = 1:this.gmm_num
                if this.gmm_w(ii) > 0.3
                    P = det(this.gmm_P{ii});
                    if P > P_max
                        P_max = P;
                        id = ii;
                    end
                end
            end
            %}

            goal = [this.gmm_mu(:,id)' atan2(this.gmm_mu(2,id)-this.state(2),this.gmm_mu(1,id)-this.state(1))];

            ss = stateSpaceSE2;
            sv = validatorOccupancyMap(ss); 
            sv.Map = map;
            sv.ValidationDistance = 0.1;

            if ~this.first&&~this.is_tracking
                planner = plannerHybridAStar(sv, ...
                    MinTurningRadius=0.8, ...
                    MotionPrimitiveLength=1.0,ForwardCost=1,ReverseCost=1e6,InterpolationDistance=v/3);
            elseif this.is_tracking
                planner = plannerHybridAStar(sv, ...
                    MinTurningRadius=2, ...
                    MotionPrimitiveLength=3,ForwardCost=1,ReverseCost=1e6,InterpolationDistance=v/3);
            else
                planner = plannerHybridAStar(sv, ...
                    MinTurningRadius=1.3, ...
                    MotionPrimitiveLength=1.5,ForwardCost=1,ReverseCost=1e6,InterpolationDistance=v/3);
            end

            if isStateValid(sv,this.state(1:3)')&&isStateValid(sv,goal(1:3))
                [path,directions,solutionInfo] = plan(planner,this.state(1:3)',goal(1:3),SearchMode='greedy');
            else
                disp('start or target is invalid');
                directions = [];
            end

            if length(directions) >= 3 && sum(directions(1:3)) == 3 && solutionInfo.IsPathFound == 1
                if this.is_tracking
                    this.planned_traj = path.States(1:4,:)';
                else
                    this.planned_traj = path.States';
                end
                optz = this.planned_traj(:,4);
                this.traj = [this.traj this.planned_traj(:,1:4)];
            else
                this.planned_traj = this.state(1:3) + [0;0;pi/3];
                optz = this.planned_traj;
                this.traj = [this.traj this.planned_traj];
            end

            %tree(ii,1:length(tree_tmp)) = tree_tmp;
        end

        %% Cell-MB-SWT
        function [this,optz] = Planner_MB(this,fld,sim,plan_mode,ps,pt,tt,ii)
            is_tracking = this.is_tracking;

            if ~is_tracking
                a = this.a_S;
                interpolated_points = this.int_pts_S;
            else
                a = this.a_T;
                interpolated_points = this.int_pts_T;
            end
            
            id = this.a_hist;

            if ~is_tracking
                if id == 6
                    a(:,7) = [];
                elseif id == 7
                    a(:,6) = [];
                end
            end

            cell = zeros(size(this.cell,1),size(this.cell,2));
            mass = 0;
            
            for ii = 1:size(cell,1)
                for jj = 1:size(cell,2)
                    % prediction
                    %
                    for mm = -1:1
                        for nn = -1:1
                            xx = this.cell_size*(ii + mm)-this.cell_size/2;
                            yy = this.cell_size*(jj + nn)-this.cell_size/2;
                            if xx>=0 && xx<=50 && yy>=0 && yy<=50
                                p = 0.05;
                                cell(ii,jj) = cell(ii,jj) + this.cell(ii+mm,jj+nn)*p;
                            elseif mm == 0 && nn == 0
                                p = 0.6;
                                cell(ii,jj) = cell(ii,jj) + this.cell(ii,jj)*p;
                            end
                        end
                    end

                    if this.map.region_exp(ceil((ii*this.cell_size-this.cell_size/2)*5),ceil((jj*this.cell_size-this.cell_size/2)*5)) < 0.45
                        cell(ii,jj) = 0;
                    end

                    mass = mass + cell(ii,jj)*this.cell_size^2;
                end
            end

            cell = cell/mass;

            value = zeros(1,size(a,2));
            allState = zeros(4,size(a,2));
            t = 1;

            for jj = 1:size(a,2)
                action = a(:,jj);
 
                state = this.state;

                state(1) = this.state(1)+action(1)*sin(this.state(3))*t+action(2)*cos(this.state(3))*t;
                state(2) = this.state(2)-action(1)*cos(this.state(3))*t+action(2)*sin(this.state(3))*t;
                state(3) = this.state(3)+action(3)*t;
                state(4) = action(4);
                allState(:,jj) = state;

                id = action(5);
                if this.is_tracking
                    p = pt{id};
                else
                    p = ps{id};
                end

                z = this.state;
                tmp = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'];

                wrong = 0;
                for kk = 1:21
                    if any([1;1] >= tmp(1:2,kk))||any([49;49] <= tmp(1:2,kk))||this.map.region_exp(ceil(tmp(1,kk)*5),ceil(tmp(2,kk)*5)) < 0.45
                        wrong = 1;
                        break
                    end
                end

                if wrong
                    value(jj) = -1000;
                    continue
                end

                % cell-MB approximation
                % cell in FOV
                id_cell = [];
                for mm = 1:size(cell,1)
                    for nn = 1:size(cell,2)
                        if this.inFOV(this.map.occ_map,state,[mm*this.cell_size-this.cell_size/2;nn*this.cell_size-this.cell_size/2],1)
                            id_cell = [id_cell;[mm nn]];
                        end
                    end
                end


                if this.is_tracking
                    value(jj) = 0;
                    for ll = 1:size(id_cell,1)
                        xx = id_cell(ll,1);
                        yy = id_cell(ll,2);
                        value(jj) = value(jj) + cell(xx,yy);
                    end
                else
                    value(jj) = 0;
                    for ll = 1:size(id_cell,1)
                        xx = id_cell(ll,1);
                        yy = id_cell(ll,2);
                        value(jj) = value(jj) + cell(xx,yy);
                    end
                end
            end

            max_val = max(value);
            max_val_indices = find(value == max_val);
            idxmax = max_val_indices(randperm(length(max_val_indices),1));

            optz = allState(:,idxmax);

            this.planned_traj = [];

            id = idxmax;
            z = this.state;
            if is_tracking
                p = pt{id};
            else
                p = ps{id};
            end
            this.traj = [this.traj [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'-pi/2]];
            this.planned_traj = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'-pi/2];
            this.a_hist = id;

            this.value_max = max_val;

            %tree(ii,1:length(tree_tmp)) = tree_tmp;
        end

        %% Next-Best-View(NBV)
        function [this,optz] = Planner_NBV(this,fld,sim,plan_mode,ps,pt,tt,ii)
            is_tracking = this.is_tracking;

            if ~is_tracking
                a = this.a_S;
                interpolated_points = this.int_pts_S;
            else
                a = this.a_T;
                interpolated_points = this.int_pts_T;
            end
            
            id = this.a_hist;

            if ~is_tracking
                if id == 6
                    a(:,7) = [];
                elseif id == 7
                    a(:,6) = [];
                end
            end

            B = this.particles;
            w = this.w;

            if this.is_tracking
                B = (mvnrnd(B',fld.target.Q_tracking))';
            else
                B = (mvnrnd(B',fld.target.Q_search))';
            end

            for jj = 1:size(B,2)
                if any([1;1] > B(1:2,jj))||any([49;49] < B(1:2,jj))||this.map.region_exp(ceil(B(1,jj)*5),ceil(B(2,jj)*5)) < 0.45
                    w(jj) = 0;
                end
            end
            w = w./sum(w);
            N = size(B,2);

            M = 1/N;
            U = rand(1)*M;
            new_particles = zeros(2,N);
            tmp_w = w(1);
            i = 1;
            jj = 1;
            while (jj <= N)
                while (tmp_w < U+(jj-1)*M)
                    i = i+1;
                    tmp_w = tmp_w+w(i);
                end
                new_particles(:,jj) = B(:,i);
                jj = jj + 1;
            end
            B = new_particles;
            w = repmat(1/N, 1, N);

            value = zeros(1,size(a,2));
            allState = zeros(4,size(a,2));
            t = 1;

            for jj = 1:size(a,2)
                action = a(:,jj);
 
                state = this.state;

                state(1) = this.state(1)+action(1)*sin(this.state(3))*t+action(2)*cos(this.state(3))*t;
                state(2) = this.state(2)-action(1)*cos(this.state(3))*t+action(2)*sin(this.state(3))*t;
                state(3) = this.state(3)+action(3)*t;
                state(4) = action(4);
                allState(:,jj) = state;

                id = action(5);
                if this.is_tracking
                    p = pt{id};
                else
                    p = ps{id};
                end

                z = this.state;
                tmp = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'];

                wrong = 0;
                for kk = 1:21
                    if any([1;1] >= tmp(1:2,kk))||any([49;49] <= tmp(1:2,kk))||this.map.region_exp(ceil(tmp(1,kk)*5),ceil(tmp(2,kk)*5)) < 0.45
                        wrong = 1;
                        break
                    end
                end

                if wrong
                    value(jj) = -1000;
                    continue
                end

                value(jj) = this.MI(fld,sim,state,B,w);
            end

            max_val = max(value);
            max_val_indices = find(value == max_val);
            idxmax = max_val_indices(randperm(length(max_val_indices),1));

            optz = allState(:,idxmax);

            this.planned_traj = [];

            id = idxmax;
            z = this.state;
            if is_tracking
                p = pt{id};
            else
                p = ps{id};
            end
            this.traj = [this.traj [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'-pi/2]];
            this.planned_traj = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'-pi/2];
            this.a_hist = id;

            this.value_max = max_val;

            %tree(ii,1:length(tree_tmp)) = tree_tmp;
        end

        %% Next-Best-View(NBV)
        function [this,optz,G,Pmax] = Planner_sampling(this,fld,sim,plan_mode,ps,pt,tt,ii)
            is_tracking = this.is_tracking;
            first = this.first;

            if ~is_tracking
                a = this.a_S;
                interpolated_points = this.int_pts_S;
            else
                a = this.a_T;
                interpolated_points = this.int_pts_T;
            end
            
            id = this.a_hist;

            if ~is_tracking
                if id == 6
                    a(:,7) = [];
                elseif id == 7
                    a(:,6) = [];
                end
            end

            % belief prediction
            B = this.particles;
            w = this.w;

            if this.is_tracking
                B = (mvnrnd(B',fld.target.Q_tracking))';
            else
                B = (mvnrnd(B',fld.target.Q_search))';
            end

            for jj = 1:size(B,2)
                if any([1;1] > B(1:2,jj))||any([49;49] < B(1:2,jj))||this.map.region_exp(ceil(B(1,jj)*5),ceil(B(2,jj)*5)) < 0.45
                    w(jj) = 0;
                end
            end
            w = w./sum(w);
            N = size(B,2);

            M = 1/N;
            U = rand(1)*M;
            new_particles = zeros(2,N);
            tmp_w = w(1);
            i = 1;
            jj = 1;
            while (jj <= N)
                while (tmp_w < U+(jj-1)*M)
                    i = i+1;
                    tmp_w = tmp_w+w(i);
                end
                new_particles(:,jj) = B(:,i);
                jj = jj + 1;
            end
            B = new_particles;
            w = repmat(1/N, 1, N);

            value = zeros(1,size(a,2));
            allState = zeros(4,size(a,2));
            t = 1;

            map = copy(this.map.occ_map);
            inflate(map,0.1);
            ss = stateSpaceDubins;
            ss.MinTurningRadius = 0.2;
            ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];

            propagator = mobileRobotPropagator(Environment=map,DistanceEstimator="dubins",ControlPolicy="linearpursuit");
            propagator.StateSpace.StateBounds(1:2,:) = [map.XWorldLimits; map.YWorldLimits];
            propagator.SystemParameters.KinematicModel.SpeedLimit = [0,10];
            propagator.SystemParameters.KinematicModel.SteerLimit = [-pi/4,pi/4];
            propagator.SystemParameters.ControlPolicy.LookaheadDist = 1;

            % planning
            if is_tracking
                v = 0.99;
            elseif first
                v = 3;
                %v = 3;
            else
                v = 1.8;
            end

            if is_tracking
                propagator.ControlLimits = [0 10;-atan2(pi/8,v) atan2(pi/8,v)];
                propagator.ControlLimits = [0 10;-atan2(pi/12,v) atan2(pi/12,v)];
            elseif first
                %propagator.ControlLimits = [0 10;-atan2(pi/4,v) atan2(pi/4,v)];
                propagator.ControlLimits = [0 10;-atan2(pi/3,v) atan2(pi/3,v)];
            else
                propagator.ControlLimits = [0 10;-pi/6 pi/6];
                propagator.ControlLimits = [0 10;-atan2(pi,v) atan2(pi,v)];
            end

            setup(propagator);

            state = this.state;
            z = this.state;

            G = [];
            ns = 0;
            delta  = 0.5;
            Bud = 1e5; % Budget
            Ve = z(1:3)';
            G.Ve = KDTreeSearcher(Ve);
            G.E = [];
            G.traj = {z(1:3)'};
            G.C = 0;
            C = 0;

            I = 1e-15;

            green = [0.2980 .6 0];
            orange = [1 .5 0];
            darkblue = [0 .2 .4];
            Darkgrey = [.25 .25 .25];
            darkgrey = [.35 .35 .35];
            lightgrey = [.7 .7 .7];
            Lightgrey = [.9 .9 .9];
            fsize = 20;

            G.I = I;
            G.closed = 0;
            G.Ecost = 0;
            G.Einfo = 0;
            closed = 0;
            I_RIC = 0;
            RIC = [];

            conv_window = 30;
            cont = 1;
            iter = 0;
            step = 1;

            sp = 10;

            %while cont > 5e-2

            if first
                num = 100;
            else
                num = 100;
            end
            for jj = 1:num
                iter = iter + 1;
                % Sample configuration space of vehicle and find nearest node
                x_samp = zeros(1,3);
                while 1
                    %
                    if is_tracking
                        x_samp(1) = z(1)-10+20*rand;
                        x_samp(2) = z(2)-10+20*rand;
                        x_samp(3) = z(3)-pi/2+pi*rand;%z(3)-pi/4+pi/2*rand;
                    elseif first
                        x_samp(1) = 50*rand;
                        x_samp(2) = 50*rand;
                        x_samp(3) = 2*pi*rand-pi;
                    else
                        x_samp(1) = z(1)-10+20*rand;
                        x_samp(2) = z(2)-10+20*rand;
                        x_samp(3) = z(3)-pi/2+pi*rand;
                    end
                    %}
                    %{
                    x_samp(1) = 50*rand;
                    x_samp(2) = 50*rand;
                    x_samp(3) = 2*pi*rand-pi;
                    %}
                    if ~any([1 1] >= x_samp(1:2))&&~any([49 49] <= x_samp(1:2))%&&this.map.region_exp(ceil(x_samp(1)*5),ceil(x_samp(2)*5)) > 0.45
                        break
                    end
                end
                ns = ns + 1;
                [id_nearest, ~] = knnsearch(G.Ve, x_samp);
                if G.closed(id_nearest)
                    continue
                else
                    x_nearest = G.Ve.X(id_nearest,:);
                end
                x_feasible = zeros(1,3);

                %
                %r1 = norm(x_nearest-x_samp);
                r1 = distance(propagator,x_nearest,x_samp);
                %

                if is_tracking
                    r_thresh = 10;
                else
                    r_thresh = 10;
                end

                if r1 < r_thresh
                    x_feasible = x_samp;
                    q = [x_nearest;x_feasible];
                else
                    [q,u,steps] = propagateWhileValid(propagator,x_nearest,[v 0],x_samp,sp);
                    q = [x_nearest;q];
                    x_feasible = q(end,:);
                end
                if ~any(x_feasible)
                    error('1')
                end
 
                if any([1 1] >= x_feasible(1:2))||any([49 49] <= x_feasible(1:2))||this.map.region_exp(ceil(x_feasible(1)*5),ceil(x_feasible(2)*5)) < 0.45%||~this.inFOV(this.map.occ_map,x_nearest',x_feasible',1)%||~V(ceil(x_feasible(1)),ceil(x_feasible(2)),ceil(x_nearest(1)),ceil(x_nearest(2)))
                    continue
                end
                %}

                %{
                for ll = 1:size(a,2)
                    action = a(:,ll);

                    state = zeros(4,1);

                    state(1) = x_nearest(1)+action(1)*sin(x_nearest(3))*t+action(2)*cos(x_nearest(3))*t;
                    state(2) = x_nearest(2)-action(1)*cos(x_nearest(3))*t+action(2)*sin(x_nearest(3))*t;
                    state(3) = x_nearest(3)+action(3)*t;
                    state(4) = action(4);
                    allState(:,ll) = state;

                    id = action(5);
                    if this.is_tracking
                        p = pt{id};
                    else
                        p = ps{id};
                    end

                    z = x_nearest;
                    tmp = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'];

                    wrong = 0;
                    for kk = 1:21
                        if any([1;1] >= tmp(1:2,kk))||any([49;49] <= tmp(1:2,kk))||this.map.region_exp(ceil(tmp(1,kk)*5),ceil(tmp(2,kk)*5)) < 0.45
                            wrong = 1;
                            break
                        end
                    end

                    if wrong
                        value(ll) = 1000;
                        continue
                    end

                    value(ll) = norm(state(1:2)-x_samp(1:2)');
                end

                [~,idxmin] = min(value);

                x_feasible = allState(1:3,idxmin)';
                %}

                % Find near points to be extented
                [Idx, ~] = rangesearch(G.Ve, x_feasible, 15); % 30 for wifi
                Idx = Idx{1};
                if ~isempty(Idx)
                    for j = 1:length(Idx)
                        if G.closed(Idx(j))
                            continue
                        end
                        % Extend towards new point
                        %
                        x_new = zeros(1,3);
                        %r1 = norm(x_feasible - G.Ve.X(Idx(j),:));
                        r1 = distance(propagator,G.Ve.X(Idx(j),:),x_feasible);
                        if is_tracking
                            r_thresh = 1;
                        else
                            r_thresh = 1;
                        end
                        if r1 < r_thresh
                            x_new = x_feasible;
                            q = [G.Ve.X(Idx(j),:);x_new];
                        else
                            [q,u,steps] = propagateWhileValid(propagator,G.Ve.X(Idx(j),:),[v 0],x_feasible,sp);
                            q = [G.Ve.X(Idx(j),:);q];
                            x_new = q(end,:);
                        end

                        if any([0 0] >= x_new(1:2))||any([50 50] <= x_new(1:2))||this.map.region_exp(ceil(x_new(1)*5),ceil(x_new(2)*5)) < 0.45%||~this.inFOV(this.map.occ_map,G.Ve.X(Idx(j),:)',x_new',1)%||~V(ceil(x_new(1)),ceil(x_new(2)),ceil(G.Ve.X(Idx(j),1)),ceil(G.Ve.X(Idx(j),2)))
                            continue
                        end
                        %}

                        %{
                        for ll = 1:size(a,2)
                            action = a(:,ll);

                            state = zeros(4,1);

                            state(1) = G.Ve.X(Idx(j),1)+action(1)*sin(G.Ve.X(Idx(j),3))*t+action(2)*cos(G.Ve.X(Idx(j),3))*t;
                            state(2) = G.Ve.X(Idx(j),2)-action(1)*cos(G.Ve.X(Idx(j),3))*t+action(2)*sin(G.Ve.X(Idx(j),3))*t;
                            state(3) = G.Ve.X(Idx(j),3)+action(3)*t;
                            state(4) = action(4);
                            allState(:,ll) = state;

                            id = action(5);
                            if this.is_tracking
                                p = pt{id};
                            else
                                p = ps{id};
                            end

                            z = G.Ve.X(Idx(j),:);
                            tmp = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'];

                            wrong = 0;
                            for kk = 1:21
                                if any([1;1] >= tmp(1:2,kk))||any([49;49] <= tmp(1:2,kk))||this.map.region_exp(ceil(tmp(1,kk)*5),ceil(tmp(2,kk)*5)) < 0.45
                                    wrong = 1;
                                    break
                                end
                            end

                            if wrong
                                value(ll) = 1000;
                                continue
                            end

                            value(ll) = norm(tmp(1:2,end)-x_feasible(1:2)');
                        end

                        [~,idxmin] = min(value);

                        x_new = allState(1:3,idxmin)';

                        id = a(5,idxmin);
                        if this.is_tracking
                            p = pt{id};
                        else
                            p = ps{id};
                        end

                        z = G.Ve.X(Idx(j),:);
                        q = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'];
                        %}

                        % Calculate new information and cost
                        Ce = distance(ss,G.Ve.X(Idx(j),:),x_new);
                        Cnew = G.C(Idx(j)) + Ce;
                        Ie = this.MI(fld,sim,x_new',B,w);
                        %p = parentNode(G.E, Idx(j));
                        Inew = Ie + G.I(Idx(j));
                        if (Inew/G.I(Idx(j)) < 1) && (Cnew > G.C(Idx(j)))
                            continue
                        else
                            RIC = [RIC; (Inew/G.I(Idx(j)) - 1) / ns];
                            I_RIC(end+1) = I_RIC(end) + RIC(end);
                            step = step + 1;

                            ns = 0;
                            Ve = [Ve; x_new];
                            G.Ve = KDTreeSearcher(Ve);
                            G.E = [G.E; Idx(j) size(G.Ve.X,1)];
                            G.traj = [G.traj; q];
                            C = [C; Cnew];
                            G.C = C;
                            I = [I; Inew];
                            G.I = I;
                            G.Einfo = [G.Einfo; Ie];
                            G.Ecost = [G.Ecost; Ce];

                            %{
                    hold on
                    x_near = G.Ve.X(Idx(j),:);
                    line([x_near(1) x_new(1)], [x_near(2) x_new(2)], 'Color', lightgrey, 'linewidth', 2)
                    plot(x_new(1), x_new(2), '.', 'MarkerSize', 10, 'Color', darkgrey)
                            %}

                            if Cnew > Bud
                                closed = [closed; 1];
                                G.closed = closed;
                            else
                                closed = [closed; 0];
                                G.closed = closed;
                            end
                            if length(RIC) > conv_window
                                cont = mean(RIC(end-conv_window:end));
                            end
                        end
                        break
                    end
                end
            end

            if isempty(G.E)
                if this.a_hist == 1
                    z_opt = [z(1);z(2);z(3)-pi/3;1];
                elseif this.a_hist == 2
                    z_opt = [z(1);z(2);z(3)+pi/3;1];
                else
                    z1 = [z(1);z(2);z(3)+pi/2;1];
                    z2 = [z(1);z(2);z(3)-pi/2;1];
                    z1 = z1+[cos(z1(3))*2;sin(z1(3))*2;0;0];
                    z2 = z2+[cos(z2(3))*2;sin(z2(3))*2;0;0];
                    if any([0;0] >= z1(1:2))||any([50;50] <= z1(1:2))||this.map.region_exp(ceil(z1(1)*5),ceil(z1(2)*5)) < 0.45
                        z_opt = [z(1);z(2);z(3)-pi/3;1];
                        this.a_hist = 1;
                    elseif any([0;0] >= z2(1:2))||any([50;50] <= z2(1:2))||this.map.region_exp(ceil(z2(1)*5),ceil(z2(2)*5)) < 0.45
                        z_opt = [z(1);z(2);z(3)+pi/3;1];
                        this.a_hist = 2;
                    else
                        if norm(this.est_pos(1:2) - z1(1:2))<norm(this.est_pos(1:2) - z2(1:2))
                            z_opt = [z(1);z(2);z(3)+pi/3;1];
                            this.a_hist = 2;
                        else
                            z_opt = [z(1);z(2);z(3)-pi/3;1];
                            this.a_hist = 1;
                        end
                    end
                end

                optz = z_opt;
                traj_tmp = optz(1:3)';
                Pmax = [];
            else
                this.a_hist = 0;

                [path, leaves] = dfsPreorder(G);
                G.dfs = path;
                G.leaves = leaves;
                P = getMainPaths(G);
                Pmax = getMaxInfoPath(G);

                G.path = P;
                G.maxInfoPath = Pmax;
                G.step = step;
                G.iter = iter;
                G.I_RIC = I_RIC;

                optz = [G.Ve.X(Pmax(2),1);G.Ve.X(Pmax(2),2);G.Ve.X(Pmax(2),3);1];

                hold on
                %         for i = 1:length(P)
                %             plotPath(G.Ve.X, P{i})
                %         end
                %         plotPath(G.Ve.X, Pmax, Darkgrey, 2)

                for i = 1:length(P)
                    traj_tmp = [];
                    for j = 1:length(P{i})
                        traj_tmp = [traj_tmp;G.traj{P{i}(j),1}(end,1),G.traj{P{i}(j),1}(end,2)];
                        %plot(G.traj{P{i}(j),1}(1:end,1),G.traj{P{i}(j),1}(1:end,2),'color',[161 80 8]/255,MarkerSize=4,LineWidth=1);
                    end
                    %plot(traj_tmp(1:end,1),traj_tmp(1:end,2),'color',[161 80 8]/255,LineStyle='none',MarkerSize=8,Marker='.');
                end

                traj_tmp = G.traj{Pmax(2),1};
                %{
            for i = 2:length(Pmax)
                plot(G.traj{Pmax(i),1}(1,1:end),G.traj{Pmax(i),1}(2,1:end),'color',Darkgrey,MarkerSize=6,LineWidth=2);
       
                %text(G.traj{Pmax(i),1}(end,1),G.traj{Pmax(i),1}(end,2)+2,num2str(i));
            end
                %}

                %{
            this.planned_traj = traj_tmp;
            this.traj = [this.traj traj_tmp];
            this.G = G;
            this.Pmax = Pmax;
                %}
            end
            %
            this.planned_traj = traj_tmp';
            this.traj = [this.traj traj_tmp'];

            this.G = G;
            this.Pmax = Pmax;
            %}

            %{
            id = idxmax;
            z = this.state;
            if is_tracking
                p = pt{id};
            else
                p = ps{id};
            end
            this.traj = [this.traj [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'-pi/2]];
            this.planned_traj = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'-pi/2];
            this.a_hist = id;
            %}

            %this.value_max = max_val;

            %tree(ii,1:length(tree_tmp)) = tree_tmp;
        end

        %% planning
        function [this,optz] = Planner_HPFT(this,fld,sim,plan_mode,ps,pt,tt,ii)

            is_tracking = this.is_tracking;
            first = this.first;

            if ~is_tracking
                a = this.a_S;
                interpolated_points = this.int_pts_S;
            else
                a = this.a_T;
                interpolated_points = this.int_pts_T;
            end
           
            tree = [];
            root = Node_IMPFT;
            %Initialization
            root.num = 1;
            root.state = this.state;%匀速运动v=1
            root.hist = [];
            root.a = a;

            if ~is_tracking
                id = this.a_hist;
                if id == 6
                    root.a(:,7) = [];
                elseif id == 7
                    root.a(:,6) = [];
                end

                %{
                if ~first
                    root.a(:,1:5) = [];
                end
                %}
            end

            root.N = 0;
            root.Q = 0;
            root.parent = 0;
            root.children = [];
            root.children_maxnum = 18;
            root.is_terminal = 0;
            root.delete = 0;
            tree = [tree,root];
            this.allstate = [];

            if is_tracking
                max_depth = 4;
            else
                max_depth = 10;
            end
            %discount factor
            if is_tracking
                eta = 0.95;
            else
                eta = 0.95;
            end
            num = 1;% addtion point index

            if is_tracking
                planlen = 30;
            else
                planlen = 50;
            end

            %
            B = this.particles;
            w = this.w;
            %}
            %
            if ~this.is_tracking
                B = this.loc_par;
                w = this.loc_w;
            end
            %}

            ran = vecnorm(this.vir_tar(1:2)-this.state(1:2));
            angle = atan2(this.vir_tar(2)-this.state(2),this.vir_tar(1)-this.state(1))-this.state(3);

            intsectionPts = rayIntersection(this.map.occ_map,this.state(1:3)',angle,this.rmax,0.55);
            obs_ran = vecnorm(intsectionPts'-this.state(1:2));
            flag = ran > obs_ran;
            flag = flag.*~isnan(obs_ran);
            flag = ~flag;

            %{
            for jj = 1:planlen
                [this,tree,Reward,num] = this.simulate(fld,sim,1,num,tree,max_depth,eta,ii+1,tt,interpolated_points,a,B,w,pt,ps);
            end
            %}
            %
            while num < 100
                [this,tree,Reward,num] = this.simulate(fld,sim,1,num,tree,max_depth,eta,ii+1,tt,interpolated_points,a,B,w,pt,ps,flag);
                %num
            end
            %}

            this.tree = tree;
            this.planned_traj = [];

            max_value = -10000;
            if isempty(tree(1).children)
                optz = this.state;
            else
                %% planning horizon 1
                val = zeros(length(tree(1).children),1);
                for jj = 1:length(tree(1).children)
                    val(jj) = tree(tree(1).children(jj)).Q;
                end
                [value_max,maxid] = max(val);
                opt = tree(1).children(maxid);
                optz = tree(opt).state;
                id = tree(opt).a_num;
                z = this.state;
                if is_tracking
                    p = pt{id};
                else
                    p = ps{id};
                end
                this.traj = [this.traj [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'-pi/2]];
                this.planned_traj = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'-pi/2];
                this.a_hist = id;

                %% planning horizon
                for hor = 1:2
                    if ~isempty(tree(opt).children)
                        opt = tree(opt).children(1);
                        if ~isempty(tree(opt).children)
                            val = zeros(length(tree(opt).children),1);
                            for jj = 1:length(tree(opt).children)
                                val(jj) = tree(tree(opt).children(jj)).Q;
                            end
                            z = tree(opt).state;
                            [~,maxid] = max(val);
                            opt = tree(opt).children(maxid);
                            id = tree(opt).a_num;
                            if is_tracking
                                p = pt{id};
                            else
                                p = ps{id};
                            end
                            this.planned_traj = [this.planned_traj [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'-pi/2]];
                        end
                    end
                end
            end
            this.value_max = value_max;

            %tree(ii,1:length(tree_tmp)) = tree_tmp;
        end

        %% policy tree construction
        function [this,tree_tmp,Reward,num] = simulate(this,fld,sim,begin,num,tree_tmp,depth,eta,simIndex,tt,interpolated_points,a,B,w,pt,ps,flg) 
            if this.is_tracking
                K = 0.5;
            else
                K = 0.5;
            end
            alpha = 0.1;
            %control = [fld.target.control(tt,simIndex,1);fld.target.control(tt,simIndex,2)];

            if depth == 0
                %num = num + 1;
                Reward = 0;
                return
            else
                z = tree_tmp(begin).state(1:3);

                tree_tmp(begin).N = tree_tmp(begin).N+1;
                %if length(tree_tmp(begin).children) == tree_tmp(begin).children_maxnum
                if isempty(tree_tmp(begin).a)
                    if ~isempty(tree_tmp(begin).children)
                        [begin,tree_tmp] = this.best_child(begin,0.732,tree_tmp);
                    else
                        begin_tmp = begin;
                        begin = tree_tmp(begin).parent;
                        if begin ~= 0
                            tree_tmp(begin).children(find(tree_tmp(begin).children==begin_tmp))=[];
                        end
                        Reward = -100;
                        return
                    end
                else
                    num = num + 1;
                    [this,tree_tmp,begin,flag2] = this.expand(sim,begin,num,tree_tmp,0,1,interpolated_points,a,pt,ps);
                    if flag2 == 0
                        num = num - 1;
                        if ~isempty(tree_tmp(begin).children)
                            [begin,tree_tmp] = this.best_child(begin,0.732,tree_tmp);
                        else
                            begin_tmp = begin;
                            begin = tree_tmp(begin).parent;
                            if begin ~= 0
                                tree_tmp(begin).children(find(tree_tmp(begin).children==begin_tmp))=[];
                            end
                            Reward = -100;
                            return
                        end
                    end
                    
                    num_a = begin;
    
                    %
                    id = tree_tmp(num_a).a_num;
                    if this.is_tracking
                        p = pt{id};
                    else
                        p = ps{id};
                    end
    
                    tmp = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'];
    
                    wrong = 0;
                    for jj = 1:21
                        if any([1;1] >= tmp(1:2,jj))||any([49;49] <= tmp(1:2,jj))||this.map.region_exp(ceil(tmp(1,jj)*5),ceil(tmp(2,jj)*5)) < 0.45
                            wrong = 1;
                            break
                        end
                    end
    
                    %%%%%%%%%%%% 这里不应该分情况
                    %{
                    if ~this.is_tracking
                        if wrong
                            tree_tmp(num_a).N = tree_tmp(num_a).N+1;
                            %%%% need to be modified
                            tree_tmp(num_a).Q = -5;
                            Reward = -5;
                            return
                        end
                    end
                    %}
                    if wrong
                        tree_tmp(num_a).N = tree_tmp(num_a).N+1;
                        %%%% need to be modified
                        tree_tmp(num_a).Q = -5;
                        Reward = -5;
                        return
                    end
                    %}
                end
                
                num_a = begin;
                tree_tmp(num_a).N = tree_tmp(num_a).N+1;

                %{
                t = 1;
                B(1:2,:) = B(1:2,:) + [control(1).*cos(B(3,:))*t;control(1).*sin(B(3,:))*t];
                B(3,:) = B(3,:) + control(2)*t;
                B = (mvnrnd(B',fld.target.Q))';
                %}
                %B(1:2,:) = B(1:2,:) -1 + 2*[rand;rand];
                if this.is_tracking
                    B = (mvnrnd(B',fld.target.Q_tracking))';
                else
                    B = (mvnrnd(B',fld.target.Q_search))';
                end

                % feasible particles
                %{
                jj = 1;
                for ii = 1:size(B,2)
                    if any([1;1] > B(1:2,jj))||any([49;49] < B(1:2,jj))||this.map.region_exp(ceil(B(1,jj)),ceil(B(2,jj))) < 0.3
                        B(:,jj) = [];
                        w(jj) = [];
                        continue
                    end
                    jj = jj+1;
                end
                w = w./sum(w);
                N = size(B,2);
                %}

                %
                for jj = 1:size(B,2)
                    if any([1;1] > B(1:2,jj))||any([49;49] < B(1:2,jj))||this.map.region_exp(ceil(B(1,jj)*5),ceil(B(2,jj)*5)) < 0.45
                        w(jj) = 0;
                    end
                end
                w = w./sum(w);
                N = size(B,2);

                M = 1/N;
                U = rand(1)*M;
                new_particles = zeros(2,N);
                tmp_w = w(1);
                i = 1;
                jj = 1;
                while (jj <= N)
                    while (tmp_w < U+(jj-1)*M)
                        i = i+1;
                        tmp_w = tmp_w+w(i);
                    end
                    new_particles(:,jj) = B(:,i);
                    jj = jj + 1;
                end
                B = new_particles;
                w = repmat(1/N, 1, N);
                %}

                state_estimation = B*w';

                state = tree_tmp(num_a).state;
                if this.is_tracking
                    reward = this.MI(fld,sim,tree_tmp(num_a).state,B,w);% + this.MI(fld,sim,tree_tmp(num_a).inter_state,B,w);
                else
                    reward = this.MI(fld,sim,tree_tmp(num_a).state,B,w);
                end

                %{
                reward = 2*reward + 10*exp(-norm(state(1:2)-this.vir_tar));
                %reward = 0.5*reward + 5/norm(state(1:2)-this.vir_tar);
                %reward = 5/norm(state(1:2)-this.vir_tar);
                %reward = reward + 0.2*(mindist-norm(state(1:2)-this.first_particles(1:2,id)));
                %}

                % immediate reward (to be modified)
                tree_tmp(num_a).R = reward;


                if length(tree_tmp(begin).children) <= K*(tree_tmp(begin).N^alpha)
                    %{
                    B_tmp = B;
                    jj = 1;
                    ii = 1;
                    while(ii<=N)
                        if this.inFOV(this.map.occ_map,state(1:3),B(:,jj),1) == 0%||fld.map.V(ceil(state(1)),ceil(state(2)),ceil(B(1,jj)),ceil(B(2,jj))) == 0
                            B_tmp(:,jj) = [];
                        else
                            jj = jj + 1;
                        end
                        ii = ii + 1;
                    end
                    if strcmp(sim.sensor_type,'rb')
                        if size(B_tmp,2) == 0
                            o = [-100;-100];
                        else
                            mu = zeros(2,size(B_tmp,2));
                            for ii = 1:size(B_tmp,2)
                                mu(:,ii) = this.h(B_tmp(1:2,ii),state);
                            end
                            gm = gmdistribution(mu',this.R);
                            o = random(gm);
                        end
                    else
                        if size(B_tmp,2) == 0
                            o = -100;
                        else
                            mu = zeros(size(B_tmp,2),1);
                            for ii = 1:size(B_tmp,2)
                                mu(ii) = this.h(B_tmp(1:2,ii),state);
                            end
                            gm = gmdistribution(mu,this.R);
                            o = random(gm);
                        end
                    end
                    %}
                    %
                    if N == 0
                        o = [-100;-100];
                    else
                        randnum = randsample(N,1,true,w);
                        if this.inFOV(this.map.occ_map,state(1:3),B(:,randnum),1) == 0%||fld.map.V(ceil(state(1)),ceil(state(2)),ceil(B(1,jj)),ceil(B(2,jj))) == 0
                            o = [-100;-100];
                        else
                            o = this.h(B(1:2,randnum),state)+(mvnrnd([0;0],this.R))';
                        end
                    end
                    %}
                    num = num + 1;
                    [this,tree_tmp,begin] = this.expand(sim,begin,num,tree_tmp,o,2,interpolated_points,a,pt,ps);
                    flag = 1;
                else
                    begin = tree_tmp(begin).children(randperm(length(tree_tmp(begin).children),1));
                    o = tree_tmp(begin).hist(6:end,end);
                    flag = 0;
                end
                %o=-100;
                num_o = begin;

                B_pre = B;
                if o(1)~=-100%这里如果不走PF可能会出现infeasible的粒子
                    [B,w] = this.PF(fld,sim,tt,simIndex,tree_tmp(num_a).state(:,1),B,w,o,0);
                end
                if flag == 1
                    node = tree_tmp(begin);
                    if this.is_tracking %%% tracking case
                        %{
                        simIndex = simIndex + 1;
                        rollout = this.rollOut(fld,sim,node,eta,depth-1,B,w,simIndex,tt,pt,ps,flg);
                        %}
                        %
                        if isempty(this.allstate)
                            simIndex = simIndex + 1;
                            rollout = this.rollOut(fld,sim,node,eta,depth-1,B,w,simIndex,tt,pt,ps,flg);
                            this.allstate = [this.allstate [node.state(1:3);begin]];
                            tree_tmp(begin).r = rollout;
                        else
                            [Idx,D] = knnsearch(this.allstate(1:2,:)',node.state(1:2)');
       
                            if D < 3 && norm(this.allstate(3,Idx)-node.state(3)) < pi %&& begin ~= this.allstate(4,Idx)
                                rollout = tree_tmp(this.allstate(4,Idx)).r;
                                tree_tmp(begin).r = rollout;
                            else
                                simIndex = simIndex + 1;
                                rollout = this.rollOut(fld,sim,node,eta,depth-1,B,w,simIndex,tt,pt,ps,flg);
                                this.allstate = [this.allstate [node.state(1:3);begin]];
                                tree_tmp(begin).r = rollout;

                                % rollout reuse
                                %
                                % tic
                                T = size(tree_tmp(begin).hist,2);
                                for ii = 2:length(tree_tmp)
                                    if tree_tmp(ii).a_num == 0 && size(tree_tmp(ii).hist,2)+1 == T
                                        kk = 1;
                                        for jj = 1:size(tree_tmp(ii).a,2)
                                            action = tree_tmp(ii).a(:,kk);
                                            id = action(5);
                                            if this.is_tracking
                                                p = pt{id};
                                            else
                                                p = ps{id};
                                            end

                                            z = tree_tmp(ii).state(1:3);
                                            tmp = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'];

                                            wrong = 0;
                                            for ll = 1:21
                                                if mod(ll-1,5)==0
                                                    if any([1;1] >= tmp(1:2,ll))||any([49;49] <= tmp(1:2,ll))||this.map.region_exp(ceil(tmp(1,ll)*5),ceil(tmp(2,ll)*5)) < 0.45
                                                        wrong = 1;
                                                        break
                                                    end
                                                end
                                            end

                                            if wrong
                                                %{
                                            tree_tmp(ii).a(:,kk) = [];
                                            kk = kk - 1;
                                            tree_tmp = backup(this,tree_tmp,ii,-5,eta);
                                                %}
                                            else
                                                if norm(this.allstate(1:2,end)-tmp(1:2,end)) < 0.5 && norm(this.allstate(3,end)-tmp(3,end)) < pi
                                                    % expand with rollout reward
                                                    state = tmp(:,end);
                                                    state = [state;action(4)];
                                                    num = num + 1;
                                                    [this,tree_tmp,num] = this.expand_spec(sim,ii,num,tree_tmp,state,action,a);

                                                    % backup
                                                    r = eta*rollout + this.MI(fld,sim,state,B_pre,w); % + dist
                                                    %r = eta*rollout + 3*exp(-norm(state(1:2)-this.vir_tar));
                                                    tree_tmp = backup(this,tree_tmp,num,r,eta);

                                                    tree_tmp(ii).a(:,kk) = [];
                                                    kk = kk - 1;
                                                end
                                            end

                                            kk = kk + 1;
                                        end
                                    end
                                end
                                %
                                % toc
                            end
                        end
                        %}
                    else %%% search case
                        %{
                        simIndex = simIndex + 1;
                        rollout = this.rollOut(fld,sim,node,eta,depth-1,B,w,simIndex,tt,pt,ps,flg);
                        %}
                        %
                        if isempty(this.allstate)
                            simIndex = simIndex + 1;
                            rollout = this.rollOut(fld,sim,node,eta,depth-1,B,w,simIndex,tt,pt,ps,flg);
                            this.allstate = [this.allstate [node.state(1:3);begin]];
                            tree_tmp(begin).r = rollout;
                        else
                            [Idx,D] = knnsearch(this.allstate(1:2,:)',node.state(1:2)');
                            if D < 1 && norm(this.allstate(3,Idx)-node.state(3)) < pi %&& begin ~= this.allstate(4,Idx)
                                rollout = tree_tmp(this.allstate(4,Idx)).r;
                                tree_tmp(begin).r = rollout;
                            else
                                simIndex = simIndex + 1;
                                rollout = this.rollOut(fld,sim,node,eta,depth-1,B,w,simIndex,tt,pt,ps,flg);
                                this.allstate = [this.allstate [node.state(1:3);begin]];
                                tree_tmp(begin).r = rollout;

                                % rollout reuse
                                %
                                % tic
                                T = size(tree_tmp(begin).hist,2);
                                for ii = 2:length(tree_tmp)
                                    if tree_tmp(ii).a_num == 0 && size(tree_tmp(ii).hist,2)+1 == T
                                        kk = 1;
                                        for jj = 1:size(tree_tmp(ii).a,2)
                                            action = tree_tmp(ii).a(:,kk);
                                            id = action(5);
                                            if this.is_tracking
                                                p = pt{id};
                                            else
                                                p = ps{id};
                                            end

                                            z = tree_tmp(ii).state(1:3);
                                            tmp = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'];

                                            wrong = 0;
                                            for ll = 1:21
                                                if mod(ll-1,5)==0
                                                    if any([1;1] >= tmp(1:2,ll))||any([49;49] <= tmp(1:2,ll))||this.map.region_exp(ceil(tmp(1,ll)*5),ceil(tmp(2,ll)*5)) < 0.45
                                                        wrong = 1;
                                                        break
                                                    end
                                                end
                                            end

                                            if wrong
                                                %{
                                            tree_tmp(ii).a(:,kk) = [];
                                            kk = kk - 1;
                                            tree_tmp = backup(this,tree_tmp,ii,-5,eta);
                                                %}
                                            else
                                                if norm(this.allstate(1:2,end)-tmp(1:2,end)) < 0.6 && norm(this.allstate(3,end)-tmp(3,end)) < pi
                                                    % expand with rollout reward
                                                    state = tmp(:,end);
                                                    state = [state;action(4)];
                                                    num = num + 1;
                                                    [this,tree_tmp,num] = this.expand_spec(sim,ii,num,tree_tmp,state,action,a);

                                                    % backup
                                                    r = eta*rollout + this.MI(fld,sim,state,B_pre,w); % + dist
                                                    %r = eta*rollout + 3*exp(-norm(state(1:2)-this.vir_tar));
                                                    tree_tmp = backup(this,tree_tmp,num,r,eta);

                                                    tree_tmp(ii).a(:,kk) = [];
                                                    kk = kk - 1;
                                                end
                                            end

                                            kk = kk + 1;
                                        end
                                    end
                                end
                                %
                                % toc
                            end
                        end
                        %}
                    end
                    %}

                    %rollout = 0;
                    Reward = reward + eta*rollout;
                else
                    simIndex = simIndex + 1;
                    [this,tree_tmp,Reward,num] = this.simulate(fld,sim,begin,num,tree_tmp,depth-1,eta,simIndex,tt,interpolated_points,a,B,w,pt,ps,flg);
                    Reward = reward + eta*Reward;
                end
                tree_tmp(num_o).N = tree_tmp(num_o).N + 1;
                tree_tmp(num_a).Q = tree_tmp(num_a).Q + (Reward-tree_tmp(num_a).Q)/tree_tmp(num_a).N;
            end
        end

        function [v,tree_tmp] = best_child(this,begin,c,tree_tmp)
            max = -10000;
            for jj = 1:length(tree_tmp(begin).children)
                node = tree_tmp(begin);
                tmp = node.children(jj);
                val = tree_tmp(tmp).Q+2*c*(log(node.N)/tree_tmp(tmp).N)^0.5;
                if val>max
                    max = val;
                    v = tmp;
                end
            end
        end

        function tree_tmp = backup(this,tree_tmp,begin,r,eta)
            idx = begin;
            while(idx~=1)
                if tree_tmp(idx).a_num == 0
                    tree_tmp(idx).N = tree_tmp(idx).N+1;
                    idx = tree_tmp(idx).parent;
                else
                    tree_tmp(idx).N = tree_tmp(idx).N+1;
                    tree_tmp(idx).Q = tree_tmp(idx).Q + (r-tree_tmp(idx).Q)/tree_tmp(idx).N;
                    idx = tree_tmp(idx).parent;
                    r = tree_tmp(idx).R + eta*r;
                end
            end
        end

        function [this,tree_tmp,num] = expand_spec(this,sim,begin,num,tree_tmp,state,action,a)
            node = tree_tmp(begin);

            % action node
            new = Node_IMPFT;
            new.num = num;
            new.a_num = action(5);
            new.state = state;

            if strcmp(sim.sensor_type,'rb')
                hist = [action;0;0];
            else
                hist = [action;0];
            end
            new.hist = [node.hist,hist];

            new.a = a;
            new.N = 1;
            new.R = 0;
            new.Q = 0;
            new.r = 0;
            new.parent = begin;
            new.children = [];
            new.children_maxnum = 18;
            tree_tmp(begin).children = [tree_tmp(begin).children,new.num];
            new.is_terminal = 0;
            new.delete = 0;
            tree_tmp(num) = new;
            begin = num;

            num = num + 1;

            % observation node
            %new = node;
            new.num = num;
            new.a_num = 0;
            %new.hist(6:end,end) = o;

%             new.a = a;
%             new.N = 0;
%             new.R = 0;
%             new.Q = 0;
%             new.r = 0;
            new.parent = begin;
            new.children = [];
            new.children_maxnum = 18;
            tree_tmp(begin).children = [tree_tmp(begin).children,new.num];
            new.is_terminal = 0;
            new.delete = 0;
            tree_tmp(num) = new;
            begin = num;
        end

        function [this,tree_tmp,begin,flag2] = expand(this,sim,begin,num,tree_tmp,o,tmp,interpolated_points,a,pt,ps)
            t = 1;
            flag2 = 1;
            node = tree_tmp(begin);
            state = zeros(4,1);
            if tmp == 1 %action
                %
                ii = randperm(size(tree_tmp(begin).a,2),1);
                action = tree_tmp(begin).a(:,ii);
                tree_tmp(begin).a(:,ii) = [];

                state(1) = node.state(1)+action(1)*sin(node.state(3))*t+action(2)*cos(node.state(3))*t;
                state(2) = node.state(2)-action(1)*cos(node.state(3))*t+action(2)*sin(node.state(3))*t;
                state(3) = node.state(3)+action(3)*t;
                state(4) = action(4);

%                 inter_state = [interpolated_points(action(5),1,1)*sin(node.state(3))+interpolated_points(action(5),1,2)*cos(node.state(3))+node.state(1);
%                     -interpolated_points(action(5),1,1)*cos(node.state(3))+interpolated_points(action(5),1,2)*sin(node.state(3))+node.state(2);
%                     node.state(3)+interpolated_points(action(5),1,3)];

                tree_tmp(begin).children_maxnum = tree_tmp(begin).children_maxnum-1;
                %}
                %{
                while(1)
                    flag = 0;
                    inregion = 1;
                    if isempty(tree_tmp(begin).a)%&&isempty(tree_tmp(begin).children)
                        flag2 = 0;
                        return
                    end
                    ii = randperm(size(tree_tmp(begin).a,2),1);
                    action = tree_tmp(begin).a(:,ii);
                    tree_tmp(begin).a(:,ii) = [];

                    state(1) = node.state(1)+action(1)*sin(node.state(3))*t+action(2)*cos(node.state(3))*t;
                    state(2) = node.state(2)-action(1)*cos(node.state(3))*t+action(2)*sin(node.state(3))*t;
                    state(3) = node.state(3)+action(3)*t;
                    state(4) = action(4);

                    inter_state = [interpolated_points(action(5),1,1)*sin(node.state(3))+interpolated_points(action(5),1,2)*cos(node.state(3))+node.state(1);
                        -interpolated_points(action(5),1,1)*cos(node.state(3))+interpolated_points(action(5),1,2)*sin(node.state(3))+node.state(2);
                        node.state(3)+interpolated_points(action(5),1,3)];

                    %                     if norm(state(1:2)-state_estimation(1:2)) < 1%&&is_tracking
                    %                         continue
                    %                     end

                    id = action(5);
                    if this.is_tracking
                        p = pt{id};
                    else
                        p = ps{id};
                    end

                    z = state(1:3);
                    tmp = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'];

                    wrong = 0;
                    for jj = 1:21
                        if any([1;1] >= tmp(1:2,jj))||any([49;49] <= tmp(1:2,jj))||this.map.region(ceil(tmp(1,jj)),ceil(tmp(2,jj))) < 0.3
                            wrong = 1;
                            break
                        end
                    end

                    if wrong
                        break
                    end

                    %{
                    if any([0;0] >= state(1:2))||any([50;50] <= state(1:2))||any([0;0] >= inter_state(1:2))||any([50;50] <= inter_state(1:2))
                        flag = 1;
                        inregion = 0;
                        %tree_tmp(begin).Q = tree_tmp(begin).Q - 1000;
                    end
                    if inregion == 1
                        if fld.map.region_exp(ceil(state(1)),ceil(state(2))) == 0||fld.map.V(ceil(node.state(1)),ceil(node.state(2)),ceil(state(1)),ceil(state(2))) == 0%||region(ceil(inter_state(1)),ceil(inter_state(2))) == 0
                            flag = 1;
                            %tree_tmp(begin).Q = tree_tmp(begin).Q - 1000;
                        end
                    end

                    if state(4)>=0&&state(4)<=50&&flag == 0&&this.is_tracking==0
                        break
                    end
                    if state(4)>=0&&state(4)<=30&&flag==0&&this.is_tracking==1%&&abs(action(3))<0.4
                        break
                    end
                    %}
                    tree_tmp(begin).children_maxnum = tree_tmp(begin).children_maxnum-1;
                end
                %}
                new = Node_IMPFT;
                new.num = num;
                new.a_num = action(5);
                new.state = state;
                inter_state = [interpolated_points(action(5),1,1)*sin(node.state(3))+interpolated_points(action(5),1,2)*cos(node.state(3))+node.state(1);
                    -interpolated_points(action(5),1,1)*cos(node.state(3))+interpolated_points(action(5),1,2)*sin(node.state(3))+node.state(2);
                    node.state(3)+interpolated_points(action(5),1,3)];
                new.inter_state = inter_state;
                if strcmp(sim.sensor_type,'rb')
                    hist = [action;0;0];
                else
                    hist = [action;0];
                end
                new.hist = [node.hist,hist];
                new.a = a;
            else % observation
                new = node;
                new.a = a;

                %{
                id = node.a_num;
                if id == 6
                    new.a(:,7) = [];
                elseif id == 7
                    new.a(:,6) = [];
                end
                %}

                new.num = num;
                new.a_num = 0;
                new.hist(6:end,end) = o;
            end
            new.N = 0;
            new.R = 0;
            new.Q = 0;
            new.r = 0;
            new.parent = begin;
            new.children = [];
            new.children_maxnum = 18;
            tree_tmp(begin).children = [tree_tmp(begin).children,new.num];
            new.is_terminal = 0;
            new.delete = 0;
            tree_tmp(num) = new;
            begin = num;
        end

        function reward = rollOut(this,fld,sim,node,eta,depth,B,w,simIndex,tt,pt,ps,flg)
            if depth == 0
                reward = 0;
                return
            else
                %{
                if simIndex <= 210
                    control = [fld.target.control(tt,simIndex,1);fld.target.control(tt,simIndex,2)];
                else
                    control = [0;0];
                end

                Q = fld.target.Q;
                t = 1;
                B(1:2,:) = B(1:2,:) + [control(1).*cos(B(3,:))*t;control(1).*sin(B(3,:))*t];
                B(3,:) = B(3,:) + control(2)*t;
                B = (mvnrnd(B',Q))';
                %}
                %B(1:2,:) = B(1:2,:) -1 + 2*[rand;rand];

                %{
                if this.is_tracking
                    B = (mvnrnd(B',fld.target.Q_tracking))';
                else
                    B = (mvnrnd(B',fld.target.Q_search))';
                end
                %}

                B_tmp1 = B;
                N = size(B,2);

                flag1 = ~any(zeros(2,N)>B_tmp1(1:2,:));
                flag2 = ~any(50*ones(2,N)<B_tmp1(1:2,:));
                flag = flag1.*flag2;
                B_tmp1(1,:) = B_tmp1(1,:).*flag;
                B_tmp1(2,:) = B_tmp1(2,:).*flag;
                %B_tmp1(3,:) = B_tmp1(3,:).*flag;
                B_tmp1(:,any(B_tmp1,1)==0)=[];

                if isempty(B_tmp1)
                    reward = 0;
                    return
                end

                B_tmp2 = zeros(2,N);
                B_tmp2(:,1:size(B_tmp1,2)) = B_tmp1;
                for jj = size(B_tmp1,2)+1:N
                    B_tmp2(:,jj) = B_tmp1(:,randperm(size(B_tmp1,2),1));
                end
                B = B_tmp2;
                w = repmat(1/N, 1, N);
                

                %
                if this.is_tracking
                    threshold = 0.9;
                else
                    threshold = 0.4;
                end

                threshold = 0;

                if rand < threshold
                    if isempty(node.a)
                        reward = 0;
                    else
                        id = randperm(size(node.a,2),1);
                        action = node.a(:,id);
                        state = node.state;
                        state(1) = node.state(1)+action(1)*sin(node.state(3))+action(2)*cos(node.state(3));
                        state(2) = node.state(2)-action(1)*cos(node.state(3))+action(2)*sin(node.state(3));
                        state(3) = node.state(3)+action(3);
                        state(4) = action(4);
                        if any([0;0] >= state(1:2))||any([50;50] <= state(1:2))||this.map.region_exp(ceil(state(1)*5),ceil(state(2)*5)) < 0.45||fld.map.V(ceil(node.state(1)),ceil(node.state(2)),ceil(state(1)),ceil(state(2))) == 0||state(4)<0%||state(4)>5
                            reward = 0;
                            node.a(:,id) = [];
                        else
                            reward = MI(this,fld,sim,state,B,w);
                            node.state = state;
                        end
                    end
                else
                    action_opt = [];
                    target = [B(1,:)*w';B(2,:)*w'];

                    %
                    if ~this.is_tracking
                        target = this.vir_tar;
                    end
                    %}
                    mindis = 100000;
                    max_rew = -100000;
                    for jj = 1:size(node.a,2)
                        action = node.a(:,jj);
                        state = node.state;
                        state(1) = node.state(1)+action(1)*sin(node.state(3))+action(2)*cos(node.state(3));
                        state(2) = node.state(2)-action(1)*cos(node.state(3))+action(2)*sin(node.state(3));
                        state(3) = node.state(3)+action(3);
                        state(4) = action(4);

                        % 判断motion primitives中的多个点
                        %
                        id = action(5);

                        %{
                        if id == 6 || id == 7
                            continue
                        end
                        %}

                        if this.is_tracking
                            p = pt{id};
                        else
                            p = ps{id};
                        end

                        z = node.state(1:3);
                        tmp = [p(:,1)'*sin(z(3))+p(:,2)'*cos(z(3))+z(1);-p(:,1)'*cos(z(3))+p(:,2)'*sin(z(3))+z(2);z(3)+p(:,3)'];

                        wrong = 0;
                        for kk = 1:21
                            if mod(kk-1,5)==0
                                if any([1;1] >= tmp(1:2,kk))||any([49;49] <= tmp(1:2,kk))||this.map.region_exp(ceil(tmp(1,kk)*5),ceil(tmp(2,kk)*5)) < 0.45
                                    wrong = 1;
                                    break
                                end
                            end
                        end

                        if wrong
                            continue
                        end
                        %}

                        % 只判断motion primitives的终点
                        %{
                        if any([0;0] >= state(1:2))||any([50;50] <= state(1:2))||this.map.region_exp(ceil(state(1)),ceil(state(2))) < 0.3||fld.map.V(ceil(node.state(1)),ceil(node.state(2)),ceil(state(1)),ceil(state(2))) == 0||state(4)<0%||state(4)>5
                            continue
                        end
                        %}
                        %

                        %{
                        reward = MI(this,fld,sim,state,B,w);
                        if reward > max_rew
                            max_rew = reward;
                            action_opt = action;
                        end
                        %}
                        %
                        if norm(state(1:2)-target) < mindis
                            mindis = norm(state(1:2)-target);
                            action_opt = action;
                        end
                        %}
                    end
                    if isempty(action_opt)
                        reward = 0; %-10
                        return
                    else
                        node.state(1) = node.state(1)+action_opt(1)*sin(node.state(3))+action_opt(2)*cos(node.state(3));
                        node.state(2) = node.state(2)-action_opt(1)*cos(node.state(3))+action_opt(2)*sin(node.state(3));
                        node.state(3) = node.state(3)+action_opt(3);
                        node.state(4) = action_opt(4);
                        
                        %{
                        if flg == 0||norm(this.state(1:2)-target)<5
                            reward = MI(this,fld,sim,node.state,B,w);
                            if mindis < 1
                                reward = reward + 3;
                            end
                        else
                            reward = exp(-norm(node.state(1:2)-target));
                        end
                        %}

                        %
                        reward = MI(this,fld,sim,node.state,B,w);

                        reward = 3*exp(-norm(node.state(1:2)-target));
                        %{
                        if ~this.first
                            reward = 3*exp(-norm(node.state(1:2)-target));
                        end
                        %}
                        %}
                        %reward = MI(this,fld,sim,node.state,B,w) + exp(-norm(node.state(1:2)-target));

                        %{
                        if this.inFOV(this.map.occ_map,node.state,target(1:2),1)
                            reward = reward + 3*(depth-1);
                            return
                        end
                        %}
                        %{
                        if mindis < 2
                            reward = reward + 3;
                        end
                        %}
                    end
                end

                %{
                if ~this.is_tracking
                    sim.plotSim(this,fld,state,B,tt,simIndex);
                    %pause(0.01);
                    clf
                end
                %}
 
                reward = reward + eta*this.rollOut(fld,sim,node,eta,depth-1,B,w,simIndex+1,tt,pt,ps,flg);
            end
        end

        %% objective function
        
        function reward = MI(this,fld,sim,state,particles,w)
            %
            if any([0;0] >= state(1:2))||any([50;50] <= state(1:2))||this.map.region_exp(ceil(state(1)*5),ceil(state(2)*5)) < 0.45
                reward = -1000;
                return
            end
            %}
            R = this.R;
            H_cond = 0;
            H0 = 0.5*size(R,1)*(log(2*pi)+1)+0.5*log(det(R));
            %H0 = 0.5*(log(2*pi)+1)+0.5*log(det(R));
            N = size(particles,2);

            if this.is_tracking
                FOV = this.inFOV_red(this.map.occ_map,state(1:3),particles,0);
            else
                FOV = this.inFOV(this.map.occ_map,state(1:3),particles,0);
            end

            if ~any(FOV)
                reward = 0;
                return;
            end

            ratio = 1;

            % particle simplification
            Cidx = zeros(size(particles,2),2);
            flag = zeros(200,200);
            N = 0;
            if this.is_tracking
                grid_size = 1;
            else
                grid_size = 2.5;
            end
            for mm = 1:size(particles,2)
                id1 = ceil(particles(1,mm)/grid_size)+5;
                Cidx(mm,1) = id1;
                id2 = ceil(particles(2,mm)/grid_size)+5;
                Cidx(mm,2) = id2;
                if flag(id1,id2) == 0
                    N = N + 1;
                    flag(id1,id2) = N;
                end
            end
            %N
            particles_tmp = particles;
            w_tmp = w;
            particles = zeros(2,N);
            w = zeros(1,N);
            for mm = 1:size(particles_tmp,2)
                w(flag(Cidx(mm,1),Cidx(mm,2))) = w(flag(Cidx(mm,1),Cidx(mm,2))) + w_tmp(mm);
            end
            for mm = 1:size(particles_tmp,2)
                particles(:,flag(Cidx(mm,1),Cidx(mm,2))) = particles(:,flag(Cidx(mm,1),Cidx(mm,2))) + particles_tmp(:,mm).*w_tmp(mm)./w(flag(Cidx(mm,1),Cidx(mm,2)));
            end
            %

            if this.is_tracking
                FOV = this.inFOV_red(this.map.occ_map,state(1:3),particles,1);
            else
                FOV = this.inFOV(this.map.occ_map,state(1:3),particles,1);
            end

            for jj = 1:N
                H_temp = w(jj)*FOV(jj);
                H_cond = H_cond+H_temp;
            end
            H_cond = H0*H_cond;

            mu = zeros(N,1);%观测的均值矩阵，第i行表示第i个粒子在未来T个时间步的观测均值
            if strcmp(sim.sensor_type,'rb')
                mu = zeros(N,2);
            end
            for jj = 1:N
                if strcmp(sim.sensor_type,'ran')
                    mu(jj) = sqrt(sum((state(1:2)-particles(1:2,jj)).^2)+0.1);
                end
                if strcmp(sim.sensor_type,'br')
                    mu(jj) = atan2(particles(2,jj)-state(2),particles(1,jj)-state(1));%-state(3);%state(3)~=0在目前解决atan2突变的方法有问题
                end
                if strcmp(sim.sensor_type,'rb')
                    mu(jj,:) = [sqrt(sum((state(1:2)-particles(1:2,jj)).^2)+0.1),atan2(particles(2,jj)-state(2),particles(1,jj)-state(1))];%-state(3);%state(3)~=0在目前解决atan2突变的方法有问题
                end
            end
            %解决atan2突变问题
            if strcmp(sim.sensor_type,'br')
                if range(mod(mu,2*pi))>range(rem(mu,2*pi))
                    mu = rem(mu,2*pi);
                else
                    mu = mod(mu,2*pi);
                end
            end
            if strcmp(sim.sensor_type,'rb')
                if range(mod(mu(:,2),2*pi))>range(rem(mu(:,2),2*pi))
                    mu(:,2) = rem(mu(:,2),2*pi);
                else
                    mu(:,2) = mod(mu(:,2),2*pi);
                end
            end
            %
            %构造sigma点的参数
            nx = 1;
            if strcmp(sim.sensor_type,'rb')
                nx = 2;
            end
            
            %b = repmat({R},nx,1);
           
            b = repmat({R},1,1);
            V = blkdiag(b{:});
            X = sqrtm(V);
            lambda = 2;
            ws = zeros(1,2*nx+1);
            ws(1) = lambda/(lambda+nx);
            ws(2:end) = repmat(1/2/(lambda+nx),1,2*nx);
            tmp1 = 0;
            for jj = 1:N
                sigma = zeros(2*nx+1,nx);
                sigma(1,:) = mu(jj,:);
                for ss= 1:nx
                    sigma(2*ss,:) = mu(jj,:) + sqrt(lambda+nx)*X(:,ss)';
                    sigma(2*ss+1,:) = mu(jj,:) - sqrt(lambda+nx)*X(:,ss)';
                end
                tmp2=0;
                for ll = 1:(2*nx+1)
                    %{
                    tmp3 = 0;
                    for ss=1:N
                        tmp4 = 1;
                        if FOV(jj)~=FOV(ss)
                            tmp4 = 0;
                        end
                        if FOV(jj)==1&&FOV(ss)==1
                            tmp4 = normpdf(sigma(ll),mu(ss),sqrt(R));
                        end
                        tmp3=tmp3+w(ss)*tmp4;
                    end
                    %}
                    %
                    if FOV(jj)==1
                        if strcmp(sim.sensor_type,'rb')
                            tmp4 = (FOV==1)'.*normpdf(sigma(ll,1),mu(:,1),sqrt(R(1,1))).*normpdf(sigma(ll,2),mu(:,2),sqrt(R(2,2)));
%                             tmp4 = (FOV==1)'.*mvnpdf(sigma(ll,:),mu,R);
                        else
                            tmp4 = (FOV==1)'.*normpdf(sigma(ll),mu,sqrt(R));
                        end
                    else
                        tmp4 = (FOV==0)';
                    end
                    tmp3 = w*tmp4;
                    %
                    tmp2=tmp2+ws(ll)*log(tmp3);
                end
                tmp1 = tmp1 + w(jj) * tmp2;
            end
            %}
            %Taylor series expansion (Charrow)
            %{
            tmp1 = 0;
            for jj = 1:N
                tmp2 = 0;
                for kk = 1:N
                    if FOV(jj)==1&&FOV(kk)==1
                        tmp2 = tmp2+w(kk)*normpdf(mu(jj),mu(kk),sqrt(R));
                    elseif  FOV(jj)==FOV(kk)
                        tmp2 = tmp2+w(kk);
                    end
                end
                tmp1 = tmp1 + w(jj)*log(tmp2);
            end
            %}
            reward = -tmp1-H_cond;
            reward = reward*ratio;
            %reward = -tmp1;
            if reward < -10^1
                error('1');
            elseif reward < 1e-10
                reward = 0;
            end
        end

        %% %%%%% visualization for debug purpose

        function drawFOV(this,z,fld,fov_mode,color)
            if strcmp(fov_mode,'plan')
                % draw the FOV for planned robot state
                N = this.mpc_hor;
                for ii = 1:N+1
                    a1 = z(3,ii)-this.theta0;  % A random direction
                    a2 = z(3,ii)+this.theta0;
                    t = linspace(a1,a2,50);
                    x0 = z(1,ii);
                    y0 = z(2,ii);
                    x1 = z(1,ii) + this.r*cos(t);
                    y1 = z(2,ii) + this.r*sin(t);
                    plot([x0,x1,x0],[y0,y1,y0],'r--','LineWidth',0.5)
                end
            elseif strcmp(fov_mode,'cur')
                % draw the FOV for current robot state
                rmin = this.rmin;
                rmax = this.rmax;
                theta0 = this.theta0;
                plot([z(1)+rmin*cos(z(3)+theta0/2),z(1)+rmax*cos(z(3)+theta0/2)],[z(2)+rmin*sin(z(3)+theta0/2),z(2)+rmax*sin(z(3)+theta0/2)],'Color',color,'LineStyle','--',LineWidth=2);
                plot([z(1)+rmin*cos(z(3)-theta0/2),z(1)+rmax*cos(z(3)-theta0/2)],[z(2)+rmin*sin(z(3)-theta0/2),z(2)+rmax*sin(z(3)-theta0/2)],'Color',color,'LineStyle','--',LineWidth=2);
                theta=linspace(z(3)-theta0/2,z(3)+theta0/2);
                plot(z(1)+rmin*cos(theta),z(2)+rmin*sin(theta),'Color',color,'LineStyle','--',LineWidth=1);
                plot(z(1)+rmax*cos(theta),z(2)+rmax*sin(theta),'Color',color,'LineStyle','--',LineWidth=1);
            end
%             xlim([fld.fld_cor(1),fld.fld_cor(2)]);
%             ylim([fld.fld_cor(3),fld.fld_cor(4)]);
%             box on
%             axis equal
%             drawnow
        end

        function drawFOV_red(this,z,fld,fov_mode,color)
            if strcmp(fov_mode,'plan')
                % draw the FOV for planned robot state
                N = this.mpc_hor;
                for ii = 1:N+1
                    a1 = z(3,ii)-this.theta0;  % A random direction
                    a2 = z(3,ii)+this.theta0;
                    t = linspace(a1,a2,50);
                    x0 = z(1,ii);
                    y0 = z(2,ii);
                    x1 = z(1,ii) + this.r*cos(t);
                    y1 = z(2,ii) + this.r*sin(t);
                    plot([x0,x1,x0],[y0,y1,y0],'r--','LineWidth',0.5)
                end
            elseif strcmp(fov_mode,'cur')
                % draw the FOV for current robot state
                rmin = this.rmin+0.2;
                rmax = this.rmax-0.5;
                theta0 = this.theta0-10/180*pi;
                plot([z(1)+rmin*cos(z(3)+theta0/2),z(1)+rmax*cos(z(3)+theta0/2)],[z(2)+rmin*sin(z(3)+theta0/2),z(2)+rmax*sin(z(3)+theta0/2)],'Color',color,'LineStyle','--',LineWidth=2);
                plot([z(1)+rmin*cos(z(3)-theta0/2),z(1)+rmax*cos(z(3)-theta0/2)],[z(2)+rmin*sin(z(3)-theta0/2),z(2)+rmax*sin(z(3)-theta0/2)],'Color',color,'LineStyle','--',LineWidth=2);
                theta=linspace(z(3)-theta0/2,z(3)+theta0/2);
                plot(z(1)+rmin*cos(theta),z(2)+rmin*sin(theta),'Color',color,'LineStyle','--',LineWidth=1);
                plot(z(1)+rmax*cos(theta),z(2)+rmax*sin(theta),'Color',color,'LineStyle','--',LineWidth=1);
            end

        end
    end
end