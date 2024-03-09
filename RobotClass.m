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
        %h; % y=h(x)
        %del_h; % gradient of h
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
        loc_par2;
        loc_w2;

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
            %this.h = inPara.h;
            %this.del_h = inPara.del_h;
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
            flag1 = ran < this.rmax-2;
            flag2 = ran > this.rmin+1;
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
            %h = this.h;
            %del_h = this.del_h;
            R = this.R;

            % prediction
            x_pred = A*x+B;
            P_pred = A*P*A'+Q;

            % update
            C = del_h(x,this.state(1:2));
            if sum(y-[-100;-100]) ~= 0
                % if an observation is obtained
                K = P_pred*C'/(C*P_pred*C'+R);
                x_next = x_pred+K*(y-this.h(x_pred,this.state(1:2)));%C*x_pred-(x_pred-this.state(1:2))
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

        % particle filter
        function [particles,w] = PF(this,fld,sim,tt,ii,state,particles,w,y,flag)

            %particles = this.particles;
            R = this.R;
            %h = this.h;
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
                mu = this.h(particles(1:2,:),state);
                P = normpdf(y(1),mu(1,:),sqrt(R(1,1)));
                %
                P0 = normpdf(y(2),mu(2,:),sqrt(R(2,2)));
                P1 = normpdf(y(2),mu(2,:)+2*pi,sqrt(R(2,2)));
                P2 = normpdf(y(2),mu(2,:)-2*pi,sqrt(R(2,2)));
            else
                P = normpdf(y,this.h(particles(1:2,:),z),sqrt(R));
                %
                P1 = normpdf(y,this.h(particles(1:2,:),z)+2*pi,sqrt(R));
                P2 = normpdf(y,this.h(particles(1:2,:),z)-2*pi,sqrt(R));
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
                            if this.h(particles(1:2,jj),state) + z(3) <= 0
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

            B = this.particles;
            w = this.w;

            ran = vecnorm(this.vir_tar(1:2)-this.state(1:2));
            angle = atan2(this.vir_tar(2)-this.state(2),this.vir_tar(1)-this.state(1))-this.state(3);

            intsectionPts = rayIntersection(this.map.occ_map,this.state(1:3)',angle,this.rmax,0.55);
            obs_ran = vecnorm(intsectionPts'-this.state(1:2));
            flag = ran > obs_ran;
            flag = flag.*~isnan(obs_ran);
            flag = ~flag;

            while num < 70
                [this,tree,Reward,num] = this.simulate(fld,sim,1,num,tree,max_depth,eta,ii+1,tt,interpolated_points,a,B,w,pt,ps,flag);
                %num
            end

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
    
                    if wrong
                        tree_tmp(num_a).N = tree_tmp(num_a).N+1;
                        tree_tmp(num_a).Q = -5;
                        Reward = -5;
                        return
                    end
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
                reward = MI(this,fld,sim,tree_tmp(num_a).state,B,w);

                % immediate reward (to be modified)
                tree_tmp(num_a).R = reward;

                if length(tree_tmp(begin).children) <= K*(tree_tmp(begin).N^alpha)
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
                    simIndex = simIndex + 1;
                    rollout = this.rollOut(fld,sim,node,eta,depth-1,B,w,simIndex,tt,pt,ps,flg);

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
               
                action_opt = [];
                target = [B(1,:)*w';B(2,:)*w'];

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
                    id = action(5);

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
%                     reward = MI(this,fld,sim,node.state,B,w);
                    reward = 3*exp(-norm(node.state(1:2)-target));
                end

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
            %
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
            %}

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
                    mu(jj) = atan2(particles(2,jj)-state(2),particles(1,jj)-state(1));
                end
                if strcmp(sim.sensor_type,'rb')
                    mu(jj,:) = [sqrt(sum((state(1:2)-particles(1:2,jj)).^2)+0.1),atan2(particles(2,jj)-state(2),particles(1,jj)-state(1))];
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
                    if FOV(jj)==1
                        if strcmp(sim.sensor_type,'rb')
                            tmp4 = (FOV==1)'.*normpdf(sigma(ll,1),mu(:,1),sqrt(R(1,1))).*normpdf(sigma(ll,2),mu(:,2),sqrt(R(2,2)));
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

        function obs = h(this,x,z)
            obs = [sqrt(sum((x(1:2,:)-z(1:2)).^2)+0.1);atan2(x(2,:)-z(2),x(1,:)-z(1))-z(3)];
        end

        function obs = del_h(this,x,z)
            obs = [(x(1:2)-z(1:2))'/sqrt(sum((x(1:2,:)-z(1:2)).^2)+0.1); [-(x(2)-z(2))/sum((x(1:2,:)-z(1:2)).^2) (x(1)-z(1))/sum((x(1:2,:)-z(1:2)).^2)]]; % z is the robot state.
        end
    end
end