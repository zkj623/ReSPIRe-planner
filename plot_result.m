%% ablation study
close all

%importfile('result.mat');
importfile('result_ablation.mat');

%set(gcf,'position',[600,100,900,1200]);
Mean = zeros(6,4);
Var = zeros(6,4);

for ii = 1:6

figure
%subplot(3,2,1)
hold on
data_tmp1 = data_van(ii,:)';
data_tmp2 = data_reuse(ii,:)';
data_tmp3 = data_hier(ii,:)';
data_tmp4 = data_reuse_hier(ii,:)';
data_tmp1(find(data_tmp1==0))=NaN;
data_tmp2(find(data_tmp2==0))=NaN;
data_tmp3(find(data_tmp3==0))=NaN;
data_tmp4(find(data_tmp4==0))=NaN;
Mean(ii,1) = mean(data_tmp1,"omitnan");
Var(ii,1) = std(data_tmp1,"omitnan");
Mean(ii,2) = mean(data_tmp2,"omitnan");
Var(ii,2) = std(data_tmp2,"omitnan");
Mean(ii,3) = mean(data_tmp3,"omitnan");
Var(ii,3) = std(data_tmp3,"omitnan");
Mean(ii,4) = mean(data_tmp4,"omitnan");
Var(ii,4) = std(data_tmp4,"omitnan");

vec = [data_tmp1 data_tmp2 data_tmp3 data_tmp4];
boxplot(vec,  'Widths', 0.5);
xlim([0.5 4.5]);
%ylim([0, 100]);

set(gca,'xtick',1:4);
set(gca,'XTickLabel',{'vanilla','reuse','hierarchy','reuse+hierarchy'},'FontSize',20,'FontName','Times New Roman','FontWeight','bold'); %修改横坐标名称、字体
title(strcat('Scenario',num2str(ii)));

end

time_van(find(time_van==0))=NaN;
time_reuse(find(time_reuse==0))=NaN;
time_hier(find(time_hier==0))=NaN;
time_reuse_hier(find(time_reuse_hier==0))=NaN;
time_van = time_van(:);
time_reuse = time_reuse(:);
time_hier = time_hier(:);
time_reuse_hier = time_reuse_hier(:);
time_mean1 = mean(time_van,"omitnan");
time_std1 = std(time_van,"omitnan");
time_mean2 = mean(time_reuse,"omitnan");
time_std2 = std(time_reuse,"omitnan");
time_mean3 = mean(time_hier,"omitnan");
time_std3 = std(time_hier,"omitnan");
time_mean4 = mean(time_reuse_hier,"omitnan");
time_std4 = std(time_reuse_hier,"omitnan");

%% comparison
close all

%importfile('result.mat');
importfile('result_comparison.mat');

%set(gcf,'position',[600,100,900,1200]);
Mean = zeros(6,4);
Var = zeros(6,4);

figure
%subplot(3,2,1)
hold on
%{
data_tmp1 = data_van(ii,:)';
data_tmp2 = data_reuse(ii,:)';
data_tmp3 = data_hier(ii,:)';
data_tmp4 = data_reuse_hier(ii,:)';
data_tmp1(find(data_tmp1==0))=NaN;
data_tmp2(find(data_tmp2==0))=NaN;
data_tmp3(find(data_tmp3==0))=NaN;
data_tmp4(find(data_tmp4==0))=NaN;
Mean(ii,1) = mean(data_tmp1,"omitnan");
Var(ii,1) = std(data_tmp1,"omitnan");
Mean(ii,2) = mean(data_tmp2,"omitnan");
Var(ii,2) = std(data_tmp2,"omitnan");
Mean(ii,3) = mean(data_tmp3,"omitnan");
Var(ii,3) = std(data_tmp3,"omitnan");
Mean(ii,4) = mean(data_tmp4,"omitnan");
Var(ii,4) = std(data_tmp4,"omitnan");
%}

vec = [t_search_PFT500 t_search_PFT50 t_search_NBV t_search_sampling t_search_GMPHD t_search_MB]-t_search_ASPIRe ;
boxplot(vec,  'Widths', 0.5);
xlim([0.5 6.5]);
%ylim([0, 100]);

set(gca,'xtick',1:6);
set(gca,'XTickLabel',{'PFT-500','PFT-50','NBV','IIG','GMPHD','cell-MB'},'FontSize',20,'FontName','Times New Roman','FontWeight','bold'); %修改横坐标名称、字体
title('Search time comparison');

% loss rate
figure
hold on

vec = [loss_rate_ASPIRe loss_rate_PFT500 loss_rate_PFT50 loss_rate_NBV loss_rate_sampling loss_rate_GMPHD loss_rate_MB];
boxplot(vec,  'Widths', 0.5);
xlim([0.5 7.5]);
ylim([0, 0.7]);

set(gca,'xtick',1:7);
set(gca,'XTickLabel',{'ASPIRe','PFT-500','PFT-50','NBV','IIG','GMPHD','cell-MB'},'FontSize',20,'FontName','Times New Roman','FontWeight','bold'); %修改横坐标名称、字体
title('Loss rate');

% estimation error
figure
hold on

vec = [est_err_ASPIRe est_err_PFT500 est_err_PFT50 est_err_NBV est_err_sampling est_err_GMPHD est_err_MB];
boxplot(vec,  'Widths', 0.5);
xlim([0.5 7.5]);
%ylim([0, 100]);

set(gca,'xtick',1:7);
set(gca,'XTickLabel',{'ASPIRe','PFT-500','PFT-50','NBV','IIG','GMPHD','cell-MB'},'FontSize',20,'FontName','Times New Roman','FontWeight','bold'); %修改横坐标名称、字体
title('Estimation error');

% computation time
figure
hold on

vec = [com_time_ASPIRe com_time_PFT500 com_time_PFT50 com_time_NBV com_time_sampling com_time_GMPHD com_time_MB];
boxplot(vec,  'Widths', 0.5);
xlim([0.5 7.5]);
ylim([0, 0.5]);

set(gca,'xtick',1:7);
set(gca,'XTickLabel',{'ASPIRe','PFT-500','PFT-50','NBV','IIG','GMPHD','cell-MB'},'FontSize',20,'FontName','Times New Roman','FontWeight','bold'); %修改横坐标名称、字体
title('Computation time');

%{
time_van(find(time_van==0))=NaN;
time_reuse(find(time_reuse==0))=NaN;
time_hier(find(time_hier==0))=NaN;
time_reuse_hier(find(time_reuse_hier==0))=NaN;
time_van = time_van(:);
time_reuse = time_reuse(:);
time_hier = time_hier(:);
time_reuse_hier = time_reuse_hier(:);
time_mean1 = mean(time_van,"omitnan");
time_std1 = std(time_van,"omitnan");
time_mean2 = mean(time_reuse,"omitnan");
time_std2 = std(time_reuse,"omitnan");
time_mean3 = mean(time_hier,"omitnan");
time_std3 = std(time_hier,"omitnan");
time_mean4 = mean(time_reuse_hier,"omitnan");
time_std4 = std(time_reuse_hier,"omitnan");
%}

%%

close all

importfile('data.mat');

figure
hold on

cmap = colormap("jet");
legends = {'ASPIRe','PFT-500','PFT-50','NBV','IIG','GMPHD','cell-MB'};

for ii = 7:-1:1

data_tmp = data{ii}';
Mean = mean(data_tmp,"omitnan");
Std = std(data_tmp,"omitnan");

color = interp1(linspace(1, 7, size(cmap, 1)), cmap, ii);

plot(Mean, 'LineWidth', 1.5,'Color', color);
hold on;
%plot(Mean + Std, 'LineWidth', 0.5,'Color', color,'LineStyle','--');
%plot(Mean - Std, 'LineWidth', 0.5,'Color', color,'LineStyle','--');

% Fill the area between Mean + Std and Mean - Std with color
fill([1:length(Mean), fliplr(1:length(Mean))], [Mean + Std, fliplr(Mean - Std)], color, 'FaceAlpha', 0.2, 'EdgeColor','none');
end

for ii = 7:-1:1

data_tmp = data{ii}';
Mean = mean(data_tmp,"omitnan");
Std = std(data_tmp,"omitnan");

color = interp1(linspace(1, 7, size(cmap, 1)), cmap, ii);

plot(Mean, 'LineWidth', 1.5,'Color', color);
end

%legend('ASPIRe','','','PFT-500','','','PFT-50','','','NBV','','','IIG','','','GMPHD','','','cell-MB','','');
legend('cell-MB','','GMPHD','','IIG','','NBV','','PFT-50','','PFT-500','','ASPIRe','');


xlabel('simulation step');
ylabel('estimation error');
title('Estimation Error Variation over Time');

