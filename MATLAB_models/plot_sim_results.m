clear;
clc;
clf;
close all;
load('simdata/2020-11-6 09:22:38_sim_out');
start_time_idx = 5; 

%Reference state data
Pdes_timeseries = timeseries(out.P_des.Data(start_time_idx:end,1:2),...
                      out.P_des.Time(start_time_idx:end));
%Reference state data
x_ref_timeseries = timeseries(out.x_ref_curr.Data(start_time_idx:end,1:2),...
                      out.x_ref_curr.Time(start_time_idx:end));
% Obstacle data
obstcl_timeseries = timeseries(out.obstcl.Data(:,:,start_time_idx:end),... 
                                out.obstcl.Time(start_time_idx:end));
ellip_coeff_timeseries = timeseries(out.ellip_coeffcients.Data(start_time_idx:end,:),... 
                                out.obstcl.Time(start_time_idx:end));
%Ego car data
ego_car_timeseries = timeseries(out.ego_car.Data(:,:,start_time_idx:end),... 
                                out.ego_car.Time(start_time_idx:end));
%Ego car states predicted                            
x_pred_timeseries = timeseries(out.x_pred.Data(:,1:2,start_time_idx:end),...
                        out.x_curr.Time(start_time_idx:end));
%Ego car states actual                    
x_actual_timeseries = timeseries(out.x_curr.Data(start_time_idx:end,1:2),...
                      out.x_curr.Time(start_time_idx:end));
x1_actual_timeseries = timeseries(-out.x_curr.Data(start_time_idx:end,1),...
                      out.x_curr.Time(start_time_idx:end));
x1_actual_timeseries.Name = 'Lateral Position x2 (m)';
x2_actual_timeseries = timeseries(out.x_curr.Data(start_time_idx:end,2),...
                                  out.x_curr.Time(start_time_idx:end));
x2_actual_timeseries.Name = 'Longitudinal Position x2 (m)';
x3_actual_timeseries = timeseries(out.x_curr.Data(start_time_idx:end,3),...
                                  out.x_curr.Time(start_time_idx:end));
x3_actual_timeseries.Name = 'Heading Ψ (rad)';
x4_actual_timeseries = timeseries(out.x_curr.Data(start_time_idx:end,4),...
                                  out.x_curr.Time(start_time_idx:end));
x4_actual_timeseries.Name = 'Velocity v (m/s)';
%Ego car acceleration actual
a_actual_timeseries = timeseries(out.acc_curr.ax.Data(start_time_idx:end),...
                                  out.acc_curr.ax.Time(start_time_idx:end));
a_actual_timeseries.Name = 'Acceleration a_x (m/s)';
%Ego car control inputs actual  
mv_acc_actual_timeseries = timeseries(out.mv.Data(start_time_idx:end,1),...
                                     out.x_curr.Time(start_time_idx:end));
mv_acc_actual_timeseries.Name = 'Acceleration a_x (m/s^2)';
mv_steerang_actual_timeseries = timeseries(out.mv.Data(start_time_idx:end,2),...
                                           out.x_curr.Time(start_time_idx:end));
mv_steerang_actual_timeseries.Name = 'Steering Angle 𝛿 (rad)';

%%%Plotting everything%%%%%%%
linewidth = 2.0;
fontsize = 14;
figure;
plot(x1_actual_timeseries, 'LineWidth' , linewidth,'HandleVisibility','off');
hold on;
plot([0 x1_actual_timeseries.Time(end)] ,[196.5 196.5], 'DisplayName','Lane/Road Edges','LineStyle', ':')
plot([0 x1_actual_timeseries.Time(end)] ,[199.5 199.5],'HandleVisibility','off','LineStyle', '--')
plot([0 x1_actual_timeseries.Time(end)] ,[202.5 202.5],'HandleVisibility','off','LineStyle', ':')
ylim([195,204])
legend;
hold off;
ax1 = gca;
ax1.YAxis.FontWeight = 'bold';
ax1.YAxis.FontSize = fontsize;
xlabel('time (s)', 'FontWeight', 'bold');
xlim([x1_actual_timeseries.Time(1),x1_actual_timeseries.Time(end)])

figure;
yyaxis left;
plot(x4_actual_timeseries, 'LineWidth' , linewidth);
yyaxis right;
plot(x3_actual_timeseries, 'LineWidth' , linewidth);
ax1 = gca;
ax1.YAxis(1).FontWeight = 'bold';
ax1.YAxis(1).FontSize = fontsize;
ax1.YAxis(2).FontWeight = 'bold';
ax1.YAxis(2).FontSize = fontsize;
xlabel('time (s)', 'FontWeight', 'bold');
xlim([x1_actual_timeseries.Time(1),x1_actual_timeseries.Time(end)])

% figure;
% yyaxis left;
% plot(a_actual_timeseries, 'LineWidth' , linewidth);
% ylim([-10,10])
% ax1 = gca;
% ax1.YAxis(1).FontWeight = 'bold';
% ax1.YAxis(1).FontSize = fontsize;
% ax1.YAxis(2).FontWeight = 'bold';
% ax1.YAxis(2).FontSize = fontsize;
% xlabel('time (s)');


figure;
yyaxis left;
plot(mv_steerang_actual_timeseries, 'LineWidth' , linewidth);
yyaxis right;
plot(mv_acc_actual_timeseries, 'LineWidth' , linewidth);
ylim([-10,10])
ax1 = gca;
ax1.YAxis(1).FontWeight = 'bold';
ax1.YAxis(1).FontSize = fontsize;
ax1.YAxis(2).FontWeight = 'bold';
ax1.YAxis(2).FontSize = fontsize;
xlabel('time (s)');
xlim([x1_actual_timeseries.Time(1),x1_actual_timeseries.Time(end)])

%Plot main trajectory & other stuff
%Plot once for Label
figure;
hold all;
i=1;
c = rescale(1:size(x_pred_timeseries.Time));
%Plot desired
scatter(Pdes_timeseries.Data(i,1), Pdes_timeseries.Data(i,2), 50, 'p',...
        'MarkerEdgeColor','#008000', 'MarkerFaceColor', '#008000','DisplayName','P_d_e_s');
%Plot reference 
scatter(x_ref_timeseries.Data(i,1), x_ref_timeseries.Data(i,2), 25, 'd',...
        'MarkerEdgeColor','#CCCC00','MarkerFaceColor', '#CCCC00','DisplayName','X_r_e_f');
%Plot X trajectory    
plot(x_actual_timeseries.Data(:,1), x_actual_timeseries.Data(:,2),...
    'Color', '#0000ff', 'LineWidth' , 3.0, 'DisplayName','Ego Car Trajectory');
%Plot Ego Car Ploygon
plgn = polyshape(ego_car_timeseries.Data(:,1,i), ego_car_timeseries.Data(:,2,i));
plot(plgn,'EdgeColor', '#0000ff', 'FaceColor', 'None', 'LineStyle', ':','DisplayName','Ego Car');
%Plot obstacle polygon
plgn = polyshape(obstcl_timeseries.Data(:,1,i), obstcl_timeseries.Data(:,2,i));
plot(plgn, 'EdgeColor', '#FF0000', 'FaceColor', '#ff726f','DisplayName','Obstacle');
%Plot inflated obstacle ellipse
n=ellip_coeff_timeseries.Data(i,6);
a = ellip_coeff_timeseries.Data(i,1); b = ellip_coeff_timeseries.Data(i,2); 
xe = ellip_coeff_timeseries.Data(i,3); ye = ellip_coeff_timeseries.Data(i,4); phi = ellip_coeff_timeseries.Data(i,5);
ellip = @(x,y) ((((x-xe).*cos(phi) - (y-ye).*sin(phi)))./a).^n + ((((x-xe).*sin(phi) + (y-ye).*cos(phi)))./b).^n - 1;
fimplicit(ellip, 'Color', '#FF0000', 'LineStyle', '--','DisplayName','Obstacle Inflation')
%Plot Predicted trajectory
scatter(x_pred_timeseries.Data(:,1,i),x_pred_timeseries.Data(:,2,i), 5, 'o',...
        'DisplayName','Predicted Trajectory','MarkerFaceColor',[0 c(i) 0.1]);
legend;    
for i = 1:8:size(x_pred_timeseries.Time)
    
    %Plot desired
    scatter(Pdes_timeseries.Data(i,1), Pdes_timeseries.Data(i,2), 50, 'p',...
            'MarkerEdgeColor','#008000', 'MarkerFaceColor', '#008000','HandleVisibility','off');
    %Plot reference 
    scatter(x_ref_timeseries.Data(i,1), x_ref_timeseries.Data(i,2), 25, 'd',...
            'MarkerEdgeColor','#CCCC00','MarkerFaceColor', '#CCCC00','HandleVisibility','off');
    %Plot obstacle polygon
    plgn = polyshape(obstcl_timeseries.Data(:,1,i), obstcl_timeseries.Data(:,2,i));
    plot(plgn, 'EdgeColor', '#FF0000', 'FaceColor', '#ff726f','HandleVisibility','off');
    %Plot inflated obstacle ellipse
    n=ellip_coeff_timeseries.Data(i,6);
    a = ellip_coeff_timeseries.Data(i,1); b = ellip_coeff_timeseries.Data(i,2); 
    xe = ellip_coeff_timeseries.Data(i,3); ye = ellip_coeff_timeseries.Data(i,4); phi = ellip_coeff_timeseries.Data(i,5);
    ellip = @(x,y) ((((x-xe).*cos(phi) - (y-ye).*sin(phi)))./a).^n + ((((x-xe).*sin(phi) + (y-ye).*cos(phi)))./b).^n - 1;
    fimplicit(ellip, 'Color', '#FF0000', 'LineStyle', '--','HandleVisibility','off')
    
    %Plot Ego Car Ploygon
    plgn = polyshape(ego_car_timeseries.Data(:,1,i), ego_car_timeseries.Data(:,2,i));
    plot(plgn,'EdgeColor', '#0000ff', 'FaceColor', 'None', 'LineStyle', ':','HandleVisibility','off');
    
    %Plot Predicted trajectory
    scatter(x_pred_timeseries.Data(:,1,i),x_pred_timeseries.Data(:,2,i), 1, 'o',...
            'MarkerFaceColor',[0 c(i) 0.1],'HandleVisibility','off');
end
hold off;
title('Scenario - Bird''s View (World Frame)')
xlabel('x (m)') 
ylabel('y (m)') 
xlim([-210 -190])
