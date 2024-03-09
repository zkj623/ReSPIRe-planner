% unknown map version

% clc 
clear % clear global variables
close all

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

for zz = 1:10
t_search = zeros(50,1);
traj_length = zeros(50,1);
t_loss = zeros(50,1);
estimation_error = zeros(50,1);
time_search = zeros(50,1);
time_tracking = zeros(50,1);
runtime = zeros(50,1);

%rng(0)

%for tt = 1:10
for tt = 1

% set up parameters
simSetup;

dbstop if error
%% %%%%%%%%%%%%%%% Simulation %%%%%%%%%%%%%%% 
% record the optimal solution of current time for warm starting ngPlanner
optz = [];
optu = [];

% save figures to video
path = sprintf('video/%s/',datetime('today'));
if ~exist(path,'dir')
    mkdir(path);
end

if save_video
    vidObj = VideoWriter(strcat(path,sprintf('unknown_%s_%s_%d_%d_%s.avi',plan_mode,sensor_type,zz,tt,datetime('today'))));
    vidObj.FrameRate = 3;
    open(vidObj);
end

list = repmat(Node_IMPFT,210,500);

for ii = 1:sim_len
    %fprintf('[main loop] gameSim.m, line %d, iteration %d, Progress: %d\n',MFileLineNr(),ii,ii/sim_len)

%     tic

    %% target moves
    fld = fld.targetMove(tt,ii);

    fld.target.traj = [fld.target.traj;fld.target.pos'];

    %% target position estimation
    rbt.y = rbt.sensorGen(fld);

    lidarnum = 60;
    ranges = zeros(lidarnum,1);
    angles = linspace(-pi/4,pi/4,lidarnum);
    maxrange = rbt.rmax;

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

    for jj = 1:5
    insertRay(rbt.map.occ_map,rbt.state(1:3)',scan,maxrange);
    end

    %show(rbt.map.occ_map)

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

    rbt.is_tracking = 0;
    %rbt.inFOV(rbt.state,fld.target.pos)
    %if rbt.inFOV(rbt.state,fld.target.pos)&&fld.map.V(ceil(rbt.state(1)),ceil(rbt.state(2)),ceil(fld.target.pos(1)),ceil(fld.target.pos(2)))
    if rbt.y(1) ~= -100
        rbt.is_tracking = 1;
        rbt.N = 50;
    end
    rbt.inFOV_hist = [rbt.inFOV_hist rbt.is_tracking];

    if rbt.first&&rbt.is_tracking
        rbt.first = 0;
    end

    %% estimation
    % particle filtering

    [rbt.particles,rbt.w] = rbt.PF(fld,sim,tt,ii,rbt.state,rbt.particles,rbt.w,rbt.y,1);
    rbt.est_pos = rbt.particles*rbt.w';
    rbt.est_pos_hist = [rbt.est_pos_hist rbt.est_pos];
    error{tt}(ii,zz) = norm(rbt.est_pos(1:2)-fld.target.pos(1:2));

    %% robot motion planning
    %
    tic

    [rbt,optz] = rbt.Planner_HPFT(fld,sim,plan_mode,ps,pt,tt,ii);

    t = toc

    if strcmp(plan_mode,'ASPIRe')
        list(ii,1:length(rbt.tree)) = rbt.tree;
    end

    %}

    % draw plot
    sim.plot_rbt_map(rbt,fld,tt,ii,plan_mode);
    %pause(0.2);

    rbt.state = optz;

    %
    if ii > 1
        wrong = 0;
        end_idx = 20;
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
