%% ablation study
close all

% importfile('result.mat');
importfile('result_ablation.mat');

%set(gcf,'position',[600,100,900,1200]);
Mean = zeros(10,4);
Var = zeros(10,4);

for ii = 1:10

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

%{
figure
hold on
vec = [data_tmp1 data_tmp2 data_tmp3 data_tmp4];
boxplot(vec,  'Widths', 0.5);
xlim([0.5 4.5]);
%ylim([0, 100]);

set(gca,'xtick',1:4);
set(gca,'XTickLabel',{'vanilla','reuse','hierarchy','reuse+hierarchy'},'FontSize',20,'FontName','Times New Roman','FontWeight','bold'); %修改横坐标名称、字体
title(strcat('Scenario',num2str(ii)));
%}

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

% Plotting the bar chart
fig = figure;
hold on
methods = {'Van','Van+R','Van+H','SPIRe'};
mean_values = [mean(Mean(:,1)), time_mean1*100;mean(Mean(:,2)), time_mean2*100;mean(Mean(:,3)), time_mean3*100;mean(Mean(:,4)), time_mean4*100];
b = bar(mean_values);
b(1).FaceColor = [242 121 112]/255;%[254 129 125]/255;
b(1).FaceAlpha = 0.8;
b(1).EdgeAlpha = 0;
b(2).FaceColor = [5 185 226]/255;%[129 184 223]/255;
b(2).EdgeAlpha = 0;
b(2).FaceAlpha = 0.8;
xticks(1:4);
xticklabels(methods);
yyaxis left
ylabel('Search time (step)');
% set(gca,'ycolor',[242 121 112]/255);
yyaxis right
ylabel('Computation time (s)');
% set(gca,'ycolor',[5 185 226]/255);
set(gca,'ycolor',[0 0 0]);
yticks(0:1/6:1);
yticklabels({'0','0.1','0.2','0.3','0.4','0.5','0.6'});

set(gca,'FontSize',30,'FontName','Times New Roman');
set(gcf,'position',[100,400,1600,800]);


legend('Search time', 'Computation time');
%title('Ablation Study Comparison');

%% comparison
close all

% importfile('result_comparison.mat');
importfile('unstructured_result_comparison.mat');

%set(gcf,'position',[600,100,900,1200]);
Mean = zeros(6,4);
Var = zeros(6,4);

figure
hold on

%{
vec = [t_search_ASPIRe-t_search_ASPIRe;t_search_PFT500-t_search_ASPIRe;t_search_PFT50-t_search_ASPIRe;t_search_NBV-t_search_ASPIRe;t_search_sampling-t_search_ASPIRe;t_search_GMPHD-t_search_ASPIRe;t_search_MB-t_search_ASPIRe];
cgroupdata = cell(70,1);
name = {'Ours','PFT-500','PFT-50','NBV','IIG','GMPHD','cell-MB'};
for ii = 1:7
    for jj = 1:10
        id = 10*(ii-1) + jj;
        cgroupdata{id} = name{ii};
    end
end
cgroupdata = categorical(cgroupdata);
b = boxchart(vec,'Orientation','horizontal','GroupByColor',cgroupdata);

%b.BoxWidth = 0.8;
legend;
set(gcf,'position',[600,100,900,1200]);
%xlim([0.5 7.5]);
%ylim([0, 2]);
yticklabels({'Category 1', 'Category 2', 'Category 3', 'Category 4'});
%}

%
cmap = colormap("jet");
b = {};
data = [t_search_ASPIRe t_search_PFT500 t_search_PFT50 t_search_NBV t_search_sampling t_search_GMPHD t_search_MB]-t_search_ASPIRe;
% data1 = data;
color = {};
%{
for ii = 1:7
    color{ii} = interp1(linspace(1, 9, size(cmap, 1)), cmap, ii+1);
end
color{1} = 'm';
color{4} = 'g';
color{5} = interp1(linspace(1, 7, size(cmap, 1)), cmap, 5);
%}

% color{8} = [242 121 112]/255;
% color{2} = [255 165 16]/255;
% color{7} = [118 218 145]/255; %[84 210 69]/255
% color{4} = [50 184 151]/255;
% color{3} = 'c';
% color{6} = [5 185 226]/255;
% color{5} = [137 131 191]/255;
% color{1} = [199 109 162]/255;


color{1} = [242 121 112]/255; 
color{2} = [255 165 16]/255;%[187 151 39]/255;
color{3} = [118 218 145]/255;%[100 200 69]/255; %[84 210 69]/255
color{4} = [50 184 151]/255;%[80 171 140]/255;
color{5} = [5 185 226]/255;
color{6} = [137 131 191]/255;
color{7} = [199 109 162]/255;

tmp = 1;

for ii = 1:7
    vec = NaN(10,tmp+2*7);
    vec(:,ii) = data1(:,ii);
    b{ii} = boxchart(vec);
    b{ii}.BoxWidth = 0.7;
    b{ii}.BoxFaceColor = color{ii};
    b{ii}.MarkerColor = color{ii};
%     b{ii}.MarkerStyle = ".";
%     b{ii}.MarkerSize = 4;
    b{ii}.WhiskerLineColor = color{ii};
    b{ii}.BoxFaceAlpha = 0.6;
    b{ii}.LineWidth = 1.5;
end

% for ii = 8:tmp
%     vec = NaN(10,tmp-7);
%     b{ii} = boxchart(vec);
% end

for ii = 1:7
    vec = NaN(10,tmp+2*7);
    vec(:,ii+tmp+7) = data(:,ii);
    b{ii+tmp+7} = boxchart(vec);
    b{ii+tmp+7}.BoxWidth = 0.7;
    b{ii+tmp+7}.BoxFaceColor = color{ii};
    b{ii+tmp+7}.MarkerColor = color{ii};
%     b{ii}.MarkerStyle = ".";
%     b{ii}.MarkerSize = 4;
    b{ii+tmp+7}.WhiskerLineColor = color{ii};
    b{ii+tmp+7}.BoxFaceAlpha = 0.6;
    b{ii+tmp+7}.LineWidth = 1.5;
end

% xlim([0.5 17.5]);
ylim([-30, 170]);
%}
xticks([]);
% xticks([0 5 10]);
% xticklabels({'structured map','unstructured map'});
set(gca,'FontSize',20,'FontName','Times New Roman'); %修改横坐标名称、字体
set(gcf,'position',[100,400,1000,400]);
% title('Search time difference');
box on

% loss rate
figure
hold on

b = {};
data = [loss_rate_ASPIRe loss_rate_PFT500 loss_rate_PFT50 loss_rate_NBV loss_rate_sampling loss_rate_GMPHD loss_rate_MB];
% data2 = data;
tmp = 1;

for ii = 1:7
    vec = NaN(10,tmp+2*7);
    vec(:,ii) = data2(:,ii);
    b{ii} = boxchart(vec);
    b{ii}.BoxWidth = 0.7;
    b{ii}.BoxFaceColor = color{ii};
    b{ii}.MarkerColor = color{ii};
%     b{ii}.MarkerStyle = ".";
%     b{ii}.MarkerSize = 4;
    b{ii}.WhiskerLineColor = color{ii};
    b{ii}.BoxFaceAlpha = 0.6;
    b{ii}.LineWidth = 1.5;
end

% for ii = 8:tmp
%     vec = NaN(10,tmp-7);
%     b{ii} = boxchart(vec);
% end

for ii = 1:7
    vec = NaN(10,tmp+2*7);
    vec(:,ii+tmp+7) = data(:,ii);
    b{ii+tmp+7} = boxchart(vec);
    b{ii+tmp+7}.BoxWidth = 0.7;
    b{ii+tmp+7}.BoxFaceColor = color{ii};
    b{ii+tmp+7}.MarkerColor = color{ii};
%     b{ii}.MarkerStyle = ".";
%     b{ii}.MarkerSize = 4;
    b{ii+tmp+7}.WhiskerLineColor = color{ii};
    b{ii+tmp+7}.BoxFaceAlpha = 0.6;
    b{ii+tmp+7}.LineWidth = 1.5;
end

% xlim([0.5 17.5]);
%}
xticks([]);
% xticks([0 5 10]);
% xticklabels({'structured map','unstructured map'});
set(gca,'FontSize',20,'FontName','Times New Roman'); %修改横坐标名称、字体
set(gcf,'position',[100,400,1000,400]);
% title('Loss rate');
box on

% estimation error
figure
hold on

b = {};
data = [est_err_ASPIRe est_err_PFT500 est_err_PFT50 est_err_NBV est_err_sampling est_err_GMPHD est_err_MB];
% data3 = data;
tmp = 1;

for ii = 1:7
    vec = NaN(10,tmp+2*7);
    vec(:,ii) = data3(:,ii);
    b{ii} = boxchart(vec);
    b{ii}.BoxWidth = 0.7;
    b{ii}.BoxFaceColor = color{ii};
    b{ii}.MarkerColor = color{ii};
%     b{ii}.MarkerStyle = ".";
%     b{ii}.MarkerSize = 4;
    b{ii}.WhiskerLineColor = color{ii};
    b{ii}.BoxFaceAlpha = 0.6;
    b{ii}.LineWidth = 1.5;
end

% for ii = 8:tmp
%     vec = NaN(10,tmp-7);
%     b{ii} = boxchart(vec);
% end

for ii = 1:7
    vec = NaN(10,tmp+2*7);
    vec(:,ii+tmp+7) = data(:,ii);
    b{ii+tmp+7} = boxchart(vec);
    b{ii+tmp+7}.BoxWidth = 0.7;
    b{ii+tmp+7}.BoxFaceColor = color{ii};
    b{ii+tmp+7}.MarkerColor = color{ii};
%     b{ii}.MarkerStyle = ".";
%     b{ii}.MarkerSize = 4;
    b{ii+tmp+7}.WhiskerLineColor = color{ii};
    b{ii+tmp+7}.BoxFaceAlpha = 0.6;
    b{ii+tmp+7}.LineWidth = 1.5;
end

% xlim([0.5 17.5]);
%}
ylim([0, 18]);
xticks([]);
% xticks([0 5 10]);
% xticklabels({'structured map','unstructured map'});
set(gca,'FontSize',20,'FontName','Times New Roman'); %修改横坐标名称、字体
set(gcf,'position',[100,400,1000,400]);
% title('Estimation error');
box on

% computation time
%
figure
hold on

b = {};
data = [com_time_ASPIRe com_time_PFT500 com_time_PFT50 com_time_NBV com_time_sampling com_time_GMPHD com_time_MB];
% data = data';

for ii = 1:7
    vec = NaN(10,7);
    vec(:,ii) = data(:,ii);
    b{ii} = boxchart(vec);
    b{ii}.BoxWidth = 0.5;
    b{ii}.BoxFaceColor = color{ii};
    b{ii}.MarkerColor = color{ii};
%     b{ii}.MarkerStyle = ".";
%     b{ii}.MarkerSize = 4;
    b{ii}.WhiskerLineColor = color{ii};
    b{ii}.BoxFaceAlpha = 0.6;
    b{ii}.LineWidth = 1.5;
%     b{ii}.Orientation="horizontal";
end

xticks([]);
% xticks([0 5 10]);
% xticklabels({'structured map','unstructured map'});
set(gca,'FontSize',20,'FontName','Times New Roman'); %修改横坐标名称、字体
set(gca,'YTickLabelRotation',90);
set(gcf,'position',[100,400,620,300]);
% title('Computation time');
box on

%}
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
legend('Cell-MB','','GMPHD','','IIG','','NBV','','PFT-50','','PFT-500','','ASPIRe','','Location','northwest');
xticks(0:50:200);
yticks(-10:10:40);

xlabel('Simulation step');
ylabel('Estimation error');
title('Estimation Error Variation over Time');

set(gca,'FontSize',20,'FontName','Times New Roman');
