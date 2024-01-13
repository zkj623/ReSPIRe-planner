%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Class for the simulation environment
% ver 1.0, Kangjie Zhou, 2023/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef FieldClass
    properties
        fld_cor; % length of x,y coordinate  
        target; % a struct for target  
        dt; % simulation time interval
        map; % map information
    end
    
    methods
        function this = FieldClass(inPara)
            this.fld_cor = inPara.fld_cor;
            this.target = inPara.target;
            this.dt = inPara.dt;
            this.map = inPara.map;
        end

        function this = targetMove(this,tt,ii)
            %{
            tar = this.target;
            f = tar.f;
            Q = tar.Q; % covariance matrix of process noise

            %                 
            tar.state = mvnrnd(f(tar.state),Q)';
            tar.pos = mvnrnd(f(tar.pos),Q)';
            tar.traj = [tar.traj,tar.pos];

            this.target = tar;
            %}
            tar = this.target;

            tar.pos = [tar.targetPose(tt,ii,1);tar.targetPose(tt,ii,2);tar.targetPose(tt,ii,3)];
            %tar.pos = [tar.targetPose(tt,1);tar.targetPose(tt,2);tar.targetPose(tt,3)];

            %tar.traj = [tar.traj,tar.pos];

            this.target = tar;
        end
    end   
end