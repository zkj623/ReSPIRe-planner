%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code for "ReSPIRe-Planner"
% Simulation for target search and tracking in unknown environments
% ver 1.0, Kangjie Zhou, 2024/3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clc 
clear % clear global variables
close all

% Initialize arrays to store results
t_search_all = [];
loss_rate_all = [];
est_err_all = [];
time_search_all = [];
time_tracking_all = [];
runtime_all = [];
error = cell(10,1);
for i = 1:10
    error{i} = zeros(200,10);
end

% Loop over multiple simulations
for zz = 1:5
t_search = zeros(50,1);
traj_length = zeros(50,1);
t_loss = zeros(50,1);
estimation_error = zeros(50,1);
time_search = zeros(50,1);
time_tracking = zeros(50,1);
runtime = zeros(50,1);

% Loop over different trials
for tt = 1:10
% for tt = 2

% Setup simulation parameters
simSetup;

% Debugging: stop if an error occurs
dbstop if error

%% %%%%%%%%%%%%%%% Simulation %%%%%%%%%%%%%%% 
% record the optimal solution of current time
optz = [];

% save figures to video
path = sprintf('../video/%s/',datetime('today'));
if ~exist(path,'dir')
    mkdir(path);
end

if save_video
    vidObj = VideoWriter(strcat(path,sprintf('unknown_%s_%s_%d_%d_%s.avi',plan_mode,sensor_type,zz,tt,datetime('today'))));
    vidObj.FrameRate = 5;
    vidObj.Quality = 100;
    open(vidObj);
end

list = repmat(Node_IMPFT,210,500);

% Main simulation loop
for ii = 1:sim_len

%     tic

    %% target moves
    fld = fld.targetMove(tt,ii);

    fld.target.traj = [fld.target.traj;fld.target.pos'];

    %% target position estimation

    % Sensor measurement generation
    rbt.y = rbt.sensorGen(fld);

    % Lidar simulation
    lidarnum = 60;
    ranges = zeros(lidarnum,1);
    angles = linspace(-pi/4,pi/4,lidarnum);
    maxrange = rbt.rmax;

    % Calculate ray intersections
    intsectionPts = rayIntersection(fld.map.occ_map,rbt.state(1:3)',angles,maxrange,0.8);

    for jj = 1:size(intsectionPts,1)
        if ~isnan(intsectionPts(jj,1))
            ranges(jj) = norm(intsectionPts(jj,:)-rbt.state(1:2)')+0.1+normrnd(0,rbt.R(1,1));
            while(ranges(jj)<=0)
                ranges(jj) = norm(intsectionPts(jj,:)-rbt.state(1:2)')+0.1+normrnd(0,rbt.R(1,1));
            end
            %ranges(jj) = 6;
        else
            ranges(jj) = maxrange+0.1;
        end
    end

    %ranges = ranges - 0.1;

    scan = lidarScan(ranges,angles);

    % Update occupancy map
    for jj = 1:5
        insertRay(rbt.map.occ_map,rbt.state(1:3)',scan,maxrange);
    end

    %show(rbt.map.occ_map)

    % Update robot's map
    rbt.map.region = occupancyMatrix(rbt.map.occ_map);
    region_tmp = rbt.map.region';
    region1 = zeros(250,250);

    for jj = 1:size(region_tmp,2)
        region1(:,jj) = region_tmp(:,size(region_tmp,2)-jj+1);
    end
    rbt.map.region = 1-region1;

    % map inflation
    map_tmp = copy(rbt.map.occ_map);
    inflate(map_tmp,0.5);

    rbt.map.region_exp = occupancyMatrix(map_tmp);

    region_tmp = rbt.map.region_exp';
    region1 = zeros(250,250);

    for jj = 1:size(region_tmp,2)
        region1(:,jj) = region_tmp(:,size(region_tmp,2)-jj+1);
    end
    rbt.map.region_exp = 1-region1;

    % Check if target is in field of view
    rbt.is_tracking = 0;
    if rbt.y(1) ~= -100
        rbt.is_tracking = 1;
        rbt.N = 50;
    end
    rbt.inFOV_hist = [rbt.inFOV_hist rbt.is_tracking];

    if rbt.first&&rbt.is_tracking
        rbt.first = 0;
    end

    %% Estimation
    if strcmp(plan_mode,'GM-PHD-SAT')
        [rbt.gmm_w,rbt.gmm_mu,rbt.gmm_P] = rbt.GM_PHD(fld,sim,tt,ii,rbt.state,rbt.y);
        rbt.est_pos = rbt.gmm_mu*rbt.gmm_w';
    elseif strcmp(plan_mode,'Cell-MB-SWT')
        [rbt.particles,rbt.w] = rbt.PF(fld,sim,tt,ii,rbt.state,rbt.particles,rbt.w,rbt.y,1);
        rbt.cell = rbt.cell_PHD(fld,sim,tt,ii,rbt.state,rbt.y);

        rbt.est_pos = [0;0];
        for jj = 1:size(rbt.cell,1)
            for kk = 1:size(rbt.cell,2)
                rbt.est_pos = rbt.est_pos + rbt.cell(jj,kk)*rbt.cell_size^2*[rbt.cell_size*jj-rbt.cell_size/2;rbt.cell_size*kk-rbt.cell_size/2];
            end
        end
    else
        % particle filtering
        [rbt.particles,rbt.w] = rbt.PF(fld,sim,tt,ii,rbt.state,rbt.particles,rbt.w,rbt.y,1);
        rbt.est_pos = rbt.particles*rbt.w';
    end

    rbt.est_pos_hist = [rbt.est_pos_hist rbt.est_pos];
    error{tt}(ii,zz) = norm(rbt.est_pos(1:2)-fld.target.pos(1:2));

    %tic
    % hierarchical particles
    if strcmp(plan_mode,'ASPIRe')
        particles = rbt.particles;
        w = rbt.w;
        Cidx = zeros(size(particles,2),2);
        flag = zeros(4,4);
        N = 0;
        grid_size = rbt.map.size/4;
        for mm = 1:size(particles,2)
            id1 = ceil(particles(1,mm)/grid_size);
            Cidx(mm,1) = id1;
            id2 = ceil(particles(2,mm)/grid_size);
            Cidx(mm,2) = id2;
            if flag(id1,id2) == 0
                N = N + 1;
                flag(id1,id2) = N;
            end
        end

        % rbt.h_particles = repmat(hpar,N,1);
        rbt.h_particles = cell(N,1);
        for jj = 1:N
            rbt.h_particles{jj} = hpar;
        end

        particles_tmp = particles;
        w_tmp = w;
        particles = zeros(2,N);
        w = zeros(1,N);
        for mm = 1:size(particles_tmp,2)
            id = flag(Cidx(mm,1),Cidx(mm,2));
            w(id) = w(id) + w_tmp(mm);
        end
        for mm = 1:size(particles_tmp,2)
            id = flag(Cidx(mm,1),Cidx(mm,2));
            particles(:,id) = particles(:,id) + particles_tmp(:,mm).*w_tmp(mm)./w(id);
            rbt.h_particles{id}.third_par = [rbt.h_particles{id}.third_par particles_tmp(:,mm)];
            rbt.h_particles{id}.third_w = [rbt.h_particles{id}.third_w w_tmp(mm)];
        end

        for jj = 1:size(rbt.h_particles,1)
            rbt.h_particles{jj}.num = jj;
            rbt.h_particles{jj}.first_w = w(jj);
            rbt.h_particles{jj}.first_par = particles(:,jj);
        end

        rbt.first_particles = particles;
        rbt.first_w = w;

        % Solve TSP problem to determine the target point

        %{
        dist_matrix = pdist2(rbt.first_particles', rbt.first_particles');
        [~, path] = tsp_solver(dist_matrix);
        target_index = path(1);
        target_point = rbt.first_particles(:, target_index);
        %}


        %{
        dist_all = vecnorm(rbt.state(1:2)-rbt.first_particles(1:2,:));
        %{
        planner = plannerAStarGrid(rbt.map.occ_map);
        for jj = 1:size(rbt.first_particles,2)
            [~,info] = plan(planner,rbt.state(1:2)',rbt.first_particles(1:2,jj)','world');
            dist_all(jj) = info.PathCost;
        end
        %}
        dist_all = dist_all.*exp(-w);
        [dist_sort,I] = sort(dist_all);
        w = w(I);
        for jj = 1:size(dist_sort,2)
            if w(jj)>0.05
                id = jj;
                break
            end
        end
        % [mindist,id] = min(dist_all);
        rbt.vir_tar = rbt.first_particles(1:2,I(id));
        rbt.loc_par = rbt.h_particles(I(id)).third_par;
        rbt.loc_w = rbt.h_particles(I(id)).third_w;
        %}

        %{
        kk = 1;
        for jj = 1:size(rbt.first_particles,2)
            if norm(rbt.first_particles(:,kk)) == 0 || rbt.first_w(kk) < 0.05 %0.10
                rbt.first_particles(:,kk) = [];
                rbt.first_w(kk) = [];
                kk = kk-1;
            end
            kk = kk+1;
        end
        %}

        % Determine critical particle
        flg = 0;
        idx = ceil(rbt.vir_tar(1)/grid_size);
        idy = ceil(rbt.vir_tar(2)/grid_size);
        for jj = 1:size(rbt.first_particles,2)
            idx_tmp = ceil(rbt.first_particles(1,jj)/grid_size);
            idy_tmp = ceil(rbt.first_particles(2,jj)/grid_size);
            if idx_tmp == idx && idy_tmp == idy && rbt.first_w(jj) > 0.20 % ablation:Q小,0.15; comparison:Q大,0.02  % unstructured 0.10
                flg = 1;
                tmp = jj;
                break
            end
        end

        if flg == 0
            dist_all = vecnorm(rbt.state(1:2)-rbt.first_particles(1:2,:));
            [dist_sort,I] = sort(dist_all);
            w = w(I);
            for jj = 1:size(dist_sort,2)
                if w(jj)>0.05
                    id = jj;
                    break
                end
            end
            % [mindist,id] = min(dist_all);
            rbt.vir_tar = rbt.first_particles(1:2,I(id));
            rbt.loc_par = rbt.h_particles{I(id)}.third_par;
            rbt.loc_w = rbt.h_particles{I(id)}.third_w;
        else
        %{
        dist_all = vecnorm(rbt.state(1:2)-rbt.first_particles(1:2,:));
        %     dist_all = dist_all.*exp(w);
        [dist_sort,I] = sort(dist_all);
        w = w(I);
        for jj = 1:size(dist_sort,2)
            if w(jj)>0.2&&dist_all(jj)<10
                tmp = jj;
                break
            end
        end
        %}

            rbt.vir_tar = rbt.first_particles(1:2,tmp);
            rbt.loc_par = rbt.h_particles{tmp}.third_par;
            rbt.loc_w = rbt.h_particles{tmp}.third_w;
        end
        %}
    end

    %
    Cidx = zeros(size(rbt.loc_par,2),2);
    flag = zeros(200,200);
    N = 0;
    if rbt.is_tracking
        grid_size = 1;
    else
        grid_size = 2.5;
    end
    for mm = 1:size(rbt.loc_par,2)
        id1 = ceil(rbt.loc_par(1,mm)/grid_size)+5;
        Cidx(mm,1) = id1;
        id2 = ceil(rbt.loc_par(2,mm)/grid_size)+5;
        Cidx(mm,2) = id2;
        if flag(id1,id2) == 0
            N = N + 1;
            flag(id1,id2) = N;
        end
    end
    %N
    particles_tmp = rbt.loc_par;
    w_tmp = rbt.loc_w;
    particles = zeros(2,N);
    w = zeros(1,N);
    for mm = 1:size(particles_tmp,2)
        w(flag(Cidx(mm,1),Cidx(mm,2))) = w(flag(Cidx(mm,1),Cidx(mm,2))) + w_tmp(mm);
    end
    for mm = 1:size(particles_tmp,2)
        particles(:,flag(Cidx(mm,1),Cidx(mm,2))) = particles(:,flag(Cidx(mm,1),Cidx(mm,2))) + particles_tmp(:,mm).*w_tmp(mm)./w(flag(Cidx(mm,1),Cidx(mm,2)));
    end

    rbt.loc_par2 = particles;
    rbt.loc_w2 = w;
    %}

%     t = toc

    %% robot motion planning
    %
    tic
    
    if strcmp(plan_mode,'NBV')
        [rbt,optz] = rbt.Planner_NBV(fld,sim,plan_mode,ps,pt,tt,ii);
    elseif strcmp(plan_mode,'sampling')
        [rbt,optz] = rbt.Planner_sampling(fld,sim,plan_mode,ps,pt,tt,ii);
    elseif strcmp(plan_mode,'PFT')
        [rbt,optz] = rbt.Planner_PFT(fld,sim,plan_mode,ps,pt,tt,ii);
    elseif strcmp(plan_mode,'ASPIRe')
        [rbt,optz] = rbt.Planner_HPFT(fld,sim,plan_mode,ps,pt,tt,ii);
    elseif strcmp(plan_mode,'GM-PHD-SAT')
        [rbt,optz] = rbt.Planner_GMPHD(fld,sim,plan_mode,ps,pt,tt,ii);
    elseif strcmp(plan_mode,'Cell-MB-SWT')
        [rbt,optz] = rbt.Planner_MB(fld,sim,plan_mode,ps,pt,tt,ii);
    end

    t = toc

    if strcmp(plan_mode,'ASPIRe')
        list(ii,1:length(rbt.tree)) = rbt.tree;
    end

    %}

    % Plot simulation results
    sim.plot_rbt_map(rbt,fld,tt,ii,plan_mode);
    %pause(0.2);

    % Update robot state
    rbt.state = optz;
    rbt.traj = [rbt.traj rbt.planned_traj];

    % Collision check
    if ii > 1
        wrong = 0;
        %
        if strcmp(plan_mode,'sampling')||strcmp(plan_mode,'Cell-MB-SWT')
            end_idx = size(rbt.planned_traj,2)-1;
        elseif strcmp(plan_mode,'GM-PHD-SAT')
            end_idx = 3;
        else 
            end_idx = 20;
        end
        %
        for jj = 0:end_idx
            if any([1;1] >= rbt.traj(1:2,end-jj))||any([49;49] <= rbt.traj(1:2,end-jj))||~fld.map.region(ceil(rbt.traj(1,end-jj)*5),ceil(rbt.traj(2,end-jj)*5))
                wrong = 1;
                break
            end
        end
        if wrong
            disp('Collision.');
            %pause
            pause(1);
            break
        end
    end
    %}

    % save the plot as a video
    frame = getframe(gcf);
    if save_video
        writeVideo(vidObj,frame);
    end   

    %%
    %{
    if ii == sim_len
        figure
        hold on
        axis([0,50,0,50]);

        %%%%%%
        map_tmp = copy(rbt.map.occ_map);
        inflate(map_tmp,0.5);
        show(map_tmp)

        %{
        plot(rbt.particles(1,:),rbt.particles(2,:),'.','Color',[0 1 0.5]);
        if strcmp(plan_mode,'ASPIRe')
            plot(rbt.loc_par(1,:),rbt.loc_par(2,:),'.','Color',[0.9290 0.6940 0.1250]);
        end
        plot(rbt.est_pos(1),rbt.est_pos(2),"^",'Color','r','MarkerFaceColor','r',MarkerSize=15);

        if ~rbt.is_tracking
            for jj = 1:size(rbt.first_particles,2)
                plot(rbt.first_particles(1,jj),rbt.first_particles(2,jj),'.m',MarkerSize=ceil(100*rbt.first_w(jj)));
            end
        end
        %}

        c = colorbar('AxisLocation','out','Ticks',[0 1],'TickLabels',{0,200});
        c.Label.String = 'Simulation step';
        c.FontSize = 15;
        cmap = colormap("jet"); % 创建颜色条

%         plot(fld.target.traj(1:ii+1,1),fld.target.traj(1:ii+1,2),'-','Color',[0.13333 0.5451 0.13333],LineWidth=3);
%         plot(rbt.traj(1,1:end-1),rbt.traj(2,1:end-1),'-','Color','r',MarkerSize=0.1,LineWidth=3);

        %
        for jj = 1:200-1
            color = interp1(linspace(1, 200, size(cmap, 1)), cmap, jj);
            plot(fld.target.traj(jj:jj+1,1),fld.target.traj(jj:jj+1,2), 'Color', color, 'LineWidth', 2,'LineStyle','--');
        end
        %plot(rbt.traj(1,1:end-1),rbt.traj(2,1:end-1),'-','Color','r',MarkerSize=0.1,LineWidth=3);

        % Plot the trajectory with gradient color
        if strcmp(plan_mode,'sampling')
            len = 200;
            step = floor(size(rbt.traj,2)/200)+1;
            for kk = 1:len
                color = interp1(linspace(1, 200, size(cmap, 1)), cmap, kk);
                if kk == len
                    for jj = (kk-1)*step+1:size(rbt.traj,2)
                        plot(rbt.traj(1,jj:jj+1),rbt.traj(2,jj:jj+1), 'Color', color, 'LineWidth', 3);
                    end
                else
                    if (kk*step > size(rbt.traj,2))
                        for jj = (kk-1)*step+1:size(rbt.traj,2)-1
                            plot(rbt.traj(1,jj:jj+1),rbt.traj(2,jj:jj+1), 'Color', color, 'LineWidth', 3);
                        end
                    else
                        for jj = (kk-1)*step+1:kk*step
                            plot(rbt.traj(1,jj:jj+1),rbt.traj(2,jj:jj+1), 'Color', color, 'LineWidth', 3);
                        end
                    end
                end
            end
        else
            len = size(rbt.traj,2)/21;
            step = 21;
            for kk = 1:len
                color = interp1(linspace(1, 200, size(cmap, 1)), cmap, kk);
                if kk == len
                    for jj = (kk-1)*step+1:kk*step-1
                        plot(rbt.traj(1,jj:jj+1),rbt.traj(2,jj:jj+1), 'Color', color, 'LineWidth', 3);
                    end
                else
                    for jj = (kk-1)*step+1:kk*step
                        plot(rbt.traj(1,jj:jj+1),rbt.traj(2,jj:jj+1), 'Color', color, 'LineWidth', 3);
                    end
                end
            end
        end
        %
        

        rbt.drawFOV(rbt.state,fld,'cur',[0.9290 0.6940 0.1250]);
%         rbt.drawFOV_red(rbt.state,fld,'cur',[1 0 1]);
    
        plot(fld.target.traj(ii+1,1),fld.target.traj(ii+1,2),"pentagram",'Color',[0.13333 0.5451 0.13333],'MarkerFaceColor',[0.13333 0.5451 0.13333],MarkerSize=15);
        plot(rbt.state(1),rbt.state(2),'ro',MarkerFaceColor='r',MarkerSize=15);

        % start plot
        %{
        plot(fld.target.traj(1,1),fld.target.traj(1,2),"pentagram",'Color','k','MarkerFaceColor','k',MarkerSize=15);
        plot(rbt.traj(1,1),rbt.traj(2,1),'ko',MarkerFaceColor='k',MarkerSize=15);
        plot(fld.target.traj(1,1),fld.target.traj(1,2),"pentagram",'Color',[0.13333 0.5451 0.13333],'MarkerFaceColor',[0.13333 0.5451 0.13333],MarkerSize=8);
        plot(rbt.traj(1,1),rbt.traj(2,1),'ro',MarkerFaceColor='r',MarkerSize=10);
        %}

        xticks(0:10:50);
        xticklabels({'0','10','20','30','40','50'});
        yticks(0:10:50);
        yticklabels({'0','10','20','30','40','50'});
        axis equal;
        axis([0,50,0,50]);

        drawnow limitrate

%         set(gcf,'position',[500,200,1300,800]);
        set(gcf,'position',[500,200,800,800]);
        set(gca,'FontSize',15,'FontName','Times New Roman');
        xticks([]);
        yticks([]);
        title('');
        xlabel('');
        ylabel('');
        f = getframe(gcf);
        imwrite(f.cdata, strcat('20240403_',plan_mode,num2str(tt),'.png'));
        pause(1)
    end
    %}

    %%
    clf

    % skip tracking
    %{
    if rbt.is_tracking
        pause(1);
        clf
        break
    end
    %}

    if rbt.is_tracking
    time_tracking(tt) = time_tracking(tt) + t;
    else
    time_search(tt) = time_search(tt) + t;
    end
    runtime(tt) = runtime(tt) + t;    

    %{
    particles_all{zz,tt,ii} = rbt.particles;
    est_all{zz,tt,ii} = rbt.est_pos;
    obs_all{zz,tt,ii} = rbt.y;
    %}
end

traj_rbt{zz,tt} = rbt.traj;
% 
if save_video
    close(vidObj);
end

inFOV_time = find(rbt.inFOV_hist==1);
if ~isempty(inFOV_time)
    t_search(tt) = inFOV_time(1);
    t_loss(tt) = 200 - t_search(tt) + 1 - length(inFOV_time);
    traj_length(tt) = 0;
%     for ii = 2:t_search_inter
%         traj_length(tt) = traj_length(tt) + norm(traj1(1:2,ii) - traj1(1:2,ii-1));
%     end
    estimation_error(tt) = mean(error{tt}(t_search(tt):end,zz));
else
    t_search(tt) = sim_len;
end
fprintf('ASPIRe: search time: %d, loss time/tracking time: %d/%d\n', t_search(tt), t_loss(tt), 200-t_search(tt));
time_search(tt) = time_search(tt)/(t_search(tt)-1);
time_tracking(tt) = time_tracking(tt)/(201-t_search(tt));
runtime(tt) = runtime(tt)/200;
%}
end
t_search_all = [t_search_all t_search];
loss_rate_all = [loss_rate_all t_loss./(200-t_search)];
est_err_all = [est_err_all estimation_error];
time_search_all = [time_search_all time_search];
time_tracking_all = [time_tracking_all time_tracking];
runtime_all = [runtime_all runtime];
end

%% save simulation result

save(sprintf("ASPIRe_%s_%s",plan_mode,date),"est_err_all","loss_rate_all","t_search_all","particles_all","est_all","obs_all","traj_rbt");
% run resultAnalysis.m to analyze the simulation results
