%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation class, used for rendering target and robot states
% ver 1.0, Kangjie Zhou, 2023/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef SimClass
    properties
        dt; % discretization time interval
        sim_len;
        sensor_type;
        fig_cnt; % counter for figure
    end
    
    methods
        function this = SimClass(inPara)
            this.dt = inPara.dt;
            this.sim_len = inPara.sim_len;     
            this.sensor_type = inPara.sensor_type;
            this.fig_cnt = 1;
        end
        
        function plotFilter(this,rbt,fld,tt,ii)
            % Plotting for particles
            hold on
            set(gcf,'position',[600,200,1920,1080]);
            axis equal;
            axis([0,50,0,50]);
            set(gca,'FontSize',40);
            box on;
           
            plot(fld.map.poly,'FaceColor',[.5 .5 .5],'LineStyle','none');
            plot(rbt.particles(1,:),rbt.particles(2,:),'.','Color',[0 1 0.5]);
            plot(rbt.est_pos(1),rbt.est_pos(2),"^",'Color','r','MarkerFaceColor','r',MarkerSize=15);

            plot(fld.target.traj(tt,1:ii+1,1),fld.target.traj(tt,1:ii+1,2),'-','Color',[0.13333 0.5451 0.13333],LineWidth=3);
            plot(rbt.traj(1,1:end-1),rbt.traj(2,1:end-1),'-','Color','r',MarkerSize=0.1,LineWidth=3);

            plot(fld.target.traj(tt,ii+1,1),fld.target.traj(tt,ii+1,2),"pentagram",'Color',[0.13333 0.5451 0.13333],'MarkerFaceColor',[0.13333 0.5451 0.13333],MarkerSize=15);
            plot(rbt.state(1),rbt.state(2),'ro',MarkerFaceColor='r',MarkerSize=15);

            rbt.drawFOV(rbt.state,fld,'cur',[0.9290 0.6940 0.1250]);
            rbt.drawFOV_red(rbt.state,fld,'cur',[1 0 1]);

            xticks(0:10:50);
            yticks(0:10:50);
            text(1,49,strcat('t=',num2str(ii)),"FontSize",30,"FontName",'Times New Roman');

            drawnow limitrate
        end

        function plot_rbt_map(this,rbt,fld,tt,ii,plan_mode)
            % Plotting for particles
            hold on
            set(gcf,'position',[600,200,1920,1080]);

            subplot(1,2,1);
            hold on
            axis([0,50,0,50]);
            show(fld.map.occ_map)

            %plot(rbt.particles(1,:),rbt.particles(2,:),'.','Color',[0 1 0.5]);
            plot(rbt.est_pos(1),rbt.est_pos(2),"^",'Color','r','MarkerFaceColor','r',MarkerSize=15);

            plot(fld.target.traj(1:ii+1,1),fld.target.traj(1:ii+1,2),'-','Color',[0.13333 0.5451 0.13333],LineWidth=3);
            plot(rbt.traj(1,1:end-1),rbt.traj(2,1:end-1),'-','Color','r',MarkerSize=0.1,LineWidth=3);
            if ~isempty(rbt.planned_traj)
                plot(rbt.planned_traj(1,1:end),rbt.planned_traj(2,1:end),'-','Color','g',MarkerSize=0.1,LineWidth=3);
            end

            plot(fld.target.traj(ii+1,1),fld.target.traj(ii+1,2),"pentagram",'Color',[0.13333 0.5451 0.13333],'MarkerFaceColor',[0.13333 0.5451 0.13333],MarkerSize=15);
            plot(rbt.state(1),rbt.state(2),'ro',MarkerFaceColor='r',MarkerSize=15);

            rbt.drawFOV(rbt.state,fld,'cur',[0.9290 0.6940 0.1250]);
            rbt.drawFOV_red(rbt.state,fld,'cur',[1 0 1]);

            text(1,48,strcat('t=',num2str(ii)),"FontSize",20,"FontName",'Times New Roman');

            %%%%%%

            subplot(1,2,2);
            hold on
            axis([0,50,0,50]);

            %%%%%%
            map_tmp = copy(rbt.map.occ_map);
            inflate(map_tmp,0.5);
            show(map_tmp)

            if strcmp(plan_mode,'GM-PHD-SAT')
                plot(rbt.est_pos(1),rbt.est_pos(2),"^",'Color','r','MarkerFaceColor','r',MarkerSize=15);
                plot(rbt.gmm_mu(1,:),rbt.gmm_mu(2,:),'x','Color',[0 1 0.5],MarkerSize=10);

                %
                % 使用 linspace 创建一个网格
                x = linspace(0, 50, 50);
                y = linspace(0, 50, 50);

                % 使用 meshgrid 创建一个网格
                [X, Y] = meshgrid(x, y);

                Z = cell(rbt.gmm_num,1);

                for jj = 1:rbt.gmm_num
                    mu = rbt.gmm_mu(:,jj);
                    Sigma = rbt.gmm_P{jj};
                    Sigma = (Sigma +Sigma') / 2;

                    % 计算每个网格点的 PDF
                    Z{jj} = mvnpdf([X(:) Y(:)], mu', Sigma);
                end

                Z_all = zeros(size(X));
                for jj = 1:rbt.gmm_num
                    Z{jj} = reshape(Z{jj}, size(X));
                    Z_all = Z_all + rbt.gmm_w(jj)*Z{jj} ;
                end

                Z_all = Z_all*50;
                alpha_data = Z_all / max(Z_all(:));

                % 绘制 PDF

                % 创建一个颜色映射，使得小的值映射到白色
                %colormap(flipud(gray));
                colormap("jet");
                clim([0, 50]);

                % 绘制 3D 图
                h = surf(X, Y, Z_all);

                % 将颜色数据设置为 Z 值
                set(h, 'CData', Z_all, 'FaceColor', 'interp');
                set(h, 'AlphaData', alpha_data, 'FaceAlpha', 'interp');

                % 添加颜色条
                colorbar;
                h.EdgeColor = 'none';
                %surf(X, Y, Z_all);
                zlim([0, 0.05]);
                view(3);
                view(-30, 45);
                %camzoom(1.3);
                %}

            elseif strcmp(plan_mode,'Cell-MB-SWT')
                plot(rbt.est_pos(1),rbt.est_pos(2),"^",'Color','r','MarkerFaceColor','r',MarkerSize=15);

                %
                x = linspace(0, 50, 50/rbt.cell_size);
                y = linspace(0, 50, 50/rbt.cell_size);
                [X, Y] = meshgrid(x, y);

                Z = rbt.cell;
                Z = reshape(Z, size(X));
                %Z = Z';

                Z = Z*50;
                alpha_data = Z / max(Z(:));

                % 绘制 PDF

                % 创建一个颜色映射，使得小的值映射到白色
                %colormap(flipud(gray));
                colormap("jet");
                clim([0, 50]);

                % 绘制 3D 图
                h = surf(Y, X, Z);

                % 将颜色数据设置为 Z 值
                set(h, 'CData', Z, 'FaceColor', 'interp');
                set(h, 'AlphaData', alpha_data, 'FaceAlpha', 'interp');

                % 添加颜色条
                colorbar;
                h.EdgeColor = 'none';
                %surf(X, Y, Z_all);
                zlim([0, 0.05]);
                view(3);
                view(-30, 45);
                %camzoom(1.3);
                %}
            else
                plot(rbt.particles(1,:),rbt.particles(2,:),'.','Color',[0 1 0.5]);
                plot(rbt.est_pos(1),rbt.est_pos(2),"^",'Color','r','MarkerFaceColor','r',MarkerSize=15);
            end

            plot(rbt.traj(1,1:end-1),rbt.traj(2,1:end-1),'-','Color','r',MarkerSize=0.1,LineWidth=3);
            %
            %}
            if strcmp(plan_mode,'sampling')
                Pmax = rbt.Pmax;
                G = rbt.G;
                for i = 2:length(Pmax)
                    %plot(G.traj{Pmax(i),1}(1,1:end),G.traj{Pmax(i),1}(2,1:end),'color',[.25 .25 .25],MarkerSize=6,LineWidth=2);
                    plot(G.traj{Pmax(i),1}(1:end,1),G.traj{Pmax(i),1}(1:end,2),'color',[.25 .25 .25],MarkerSize=6,LineWidth=2);
                end
            end

            if ~isempty(rbt.planned_traj)
                plot(rbt.planned_traj(1,1:end),rbt.planned_traj(2,1:end),'-','Color','g',MarkerSize=0.1,LineWidth=3);
            end

            plot(fld.target.traj(ii+1,1),fld.target.traj(ii+1,2),"pentagram",'Color',[0.13333 0.5451 0.13333],'MarkerFaceColor',[0.13333 0.5451 0.13333],MarkerSize=15);
            plot(rbt.state(1),rbt.state(2),'ro',MarkerFaceColor='r',MarkerSize=15);

            rbt.drawFOV(rbt.state,fld,'cur',[0.9290 0.6940 0.1250]);
            rbt.drawFOV_red(rbt.state,fld,'cur',[1 0 1]);

            xticks(0:10:50);
            xticklabels({'0','10','20','30','40','50'});
            yticks(0:10:50);
            yticklabels({'0','10','20','30','40','50'});
            axis equal;
            axis([0,50,0,50]);

            if strcmp(plan_mode,'GM-PHD-SAT')||strcmp(plan_mode,'Cell-MB-SWT')
                zticks(0:10:50);
                zticklabels({'0','0.2','0.4','0.6','0.8','1'});
                axis([0,50,0,50,0,50]);
            end
            drawnow limitrate
        end

        function plotSim(this,rbt,fld,state,B,tt,ii)
            % Plotting for particles
            hold on
            set(gcf,'position',[600,200,1920,1080]);
            axis equal;
            axis([0,50,0,50]);
            set(gca,'FontSize',40);
            box on;
           
            plot(fld.map.poly,'FaceColor',[.5 .5 .5],'LineStyle','none');
            plot(B(1,:),B(2,:),'.','Color',[0 1 0.5]);
            %plot(rbt.est_pos(1),rbt.est_pos(2),"^",'Color','r','MarkerFaceColor','r',MarkerSize=15);

            plot(fld.target.traj(tt,1:ii+1,1),fld.target.traj(tt,1:ii+1,2),'-','Color',[0.13333 0.5451 0.13333],LineWidth=3);
            plot(rbt.traj(1,1:end-1),rbt.traj(2,1:end-1),'-','Color','r',MarkerSize=0.1,LineWidth=3);

            plot(fld.target.traj(tt,ii+1,1),fld.target.traj(tt,ii+1,2),"pentagram",'Color',[0.13333 0.5451 0.13333],'MarkerFaceColor',[0.13333 0.5451 0.13333],MarkerSize=15);
            plot(state(1),state(2),'ro',MarkerFaceColor='r',MarkerSize=15);

            rbt.drawFOV(state,fld,'cur',[0.9290 0.6940 0.1250]);
            rbt.drawFOV_red(state,fld,'cur',[1 0 1]);
            

            xticks(0:10:50);
            yticks(0:10:50);
            text(1,49,strcat('t=',num2str(ii)),"FontSize",30,"FontName",'Times New Roman');

            drawnow limitrate
        end
        
        
        function plotTraj(this,rbt,fld)
            % Plotting for trajectory (path planning part)
%             shading interp
%             contourf(prob_map','LineColor','none');
%             load('MyColorMap','mymap')
%             colormap(mymap);
%             colorbar
            
            % robot traj
            hdl1 = plot(rbt.traj(1,:),rbt.traj(2,:),'r','markers',5);
            set(hdl1,'MarkerFaceColor','r');
            set(hdl1,'MarkerEdgeColor','r');
            set(hdl1,'Color','r');
            set(hdl1,'LineStyle','-');
            set(hdl1,'Marker','o');
            
            % target actual traj
            hdl2 = plot(fld.target.traj(1,:),fld.target.traj(2,:),'b','markers',5);
            set(hdl2,'MarkerFaceColor','b');
            set(hdl2,'MarkerEdgeColor','b');%
            set(hdl2,'Color','b');
            %     set(hdl2,'LineStyle','-');
            set(hdl2,'Marker','*');
            
            
            drawFOV(rbt,rbt.state,fld,'cur');

            xlim([fld.fld_cor(1),fld.fld_cor(2)]);
            ylim([fld.fld_cor(3),fld.fld_cor(4)]);
            box on
            axis equal
            drawnow
        end                
        
    end
end