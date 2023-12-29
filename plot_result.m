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

%{
figure
%subplot(3,2,2)
hold on
data_tmp1 = data_van(2,:)';
data_tmp2 = data_reuse(2,:)';
data_tmp3 = data_hier(2,:)';
data_tmp4 = data_reuse_hier(2,:)';
data_tmp1(find(data_tmp1==0))=NaN;
data_tmp2(find(data_tmp2==0))=NaN;
data_tmp3(find(data_tmp3==0))=NaN;
data_tmp4(find(data_tmp4==0))=NaN;
vec = [data_tmp1 data_tmp2 data_tmp3 data_tmp4];
boxplot(vec);
xlim([0.5 4.5]);
%ylim([0, 100]);

set(gca,'xtick',1:4);
set(gca,'XTickLabel',{'vanilla','reuse','hierarchy','reuse+hierarchy'},'FontSize',20,'FontName','Times New Roman','FontWeight','bold'); %修改横坐标名称、字体
title('Scenario 2');

figure
%subplot(3,2,3)
hold on
data_tmp1 = data_van(3,:)';
data_tmp2 = data_reuse(3,:)';
data_tmp3 = data_hier(3,:)';
data_tmp4 = data_reuse_hier(3,:)';
data_tmp1(find(data_tmp1==0))=NaN;
data_tmp2(find(data_tmp2==0))=NaN;
data_tmp3(find(data_tmp3==0))=NaN;
data_tmp4(find(data_tmp4==0))=NaN;
vec = [data_tmp1 data_tmp2 data_tmp3 data_tmp4];
boxplot(vec);
xlim([0.5 4.5]);
%ylim([0, 100]);

set(gca,'xtick',1:4);
set(gca,'XTickLabel',{'vanilla','reuse','hierarchy','reuse+hierarchy'},'FontSize',20,'FontName','Times New Roman','FontWeight','bold'); %修改横坐标名称、字体
title('Scenario 3');

%
figure
%subplot(3,2,4)
hold on
data_tmp1 = data_van(4,:)';
data_tmp2 = data_reuse(4,:)';
data_tmp3 = data_hier(4,:)';
data_tmp4 = data_reuse_hier(4,:)';
data_tmp1(find(data_tmp1==0))=NaN;
data_tmp2(find(data_tmp2==0))=NaN;
data_tmp3(find(data_tmp3==0))=NaN;
data_tmp4(find(data_tmp4==0))=NaN;
vec = [data_tmp1 data_tmp2 data_tmp3 data_tmp4];
boxplot(vec);
xlim([0.5 4.5]);
%ylim([0, 100]);

set(gca,'xtick',1:4);
set(gca,'XTickLabel',{'vanilla','reuse','hierarchy','reuse+hierarchy'},'FontSize',20,'FontName','Times New Roman','FontWeight','bold'); %修改横坐标名称、字体
title('Scenario 4');
%

figure
%subplot(3,2,5)
hold on
data_tmp1 = data_van(5,:)';
data_tmp2 = data_reuse(5,:)';
data_tmp3 = data_hier(5,:)';
data_tmp4 = data_reuse_hier(5,:)';
data_tmp1(find(data_tmp1==0))=NaN;
data_tmp2(find(data_tmp2==0))=NaN;
data_tmp3(find(data_tmp3==0))=NaN;
data_tmp4(find(data_tmp4==0))=NaN;
vec = [data_tmp1 data_tmp2 data_tmp3 data_tmp4];
boxplot(vec);
xlim([0.5 4.5]);
%ylim([0, 100]);

set(gca,'xtick',1:4);
set(gca,'XTickLabel',{'vanilla','reuse','hierarchy','reuse+hierarchy'},'FontSize',20,'FontName','Times New Roman','FontWeight','bold'); %修改横坐标名称、字体
title('Scenario 5');

figure
%subplot(3,2,6)
hold on
data_tmp1 = data_van(6,:)';
data_tmp2 = data_reuse(6,:)';
data_tmp3 = data_hier(6,:)';
data_tmp4 = data_reuse_hier(6,:)';
data_tmp1(find(data_tmp1==0))=NaN;
data_tmp2(find(data_tmp2==0))=NaN;
data_tmp3(find(data_tmp3==0))=NaN;
data_tmp4(find(data_tmp4==0))=NaN;
vec = [data_tmp1 data_tmp2 data_tmp3 data_tmp4];
boxplot(vec);
xlim([0.5 4.5]);
%ylim([0, 100]);

set(gca,'xtick',1:4);
set(gca,'XTickLabel',{'vanilla','reuse','hierarchy','reuse+hierarchy'},'FontSize',20,'FontName','Times New Roman','FontWeight','bold'); %修改横坐标名称、字体
title('Scenario 6');
%}