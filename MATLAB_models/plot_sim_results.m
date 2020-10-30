clear;
clc;
clf;
close all;
load('simdata/2020-10-29 18:47:34_sim_out.mat');
start_time_idx = 60; 

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
x1_actual_timeseries = timeseries(out.x_curr.Data(start_time_idx:end,1),...
                      out.x_curr.Time(start_time_idx:end));
x1_actual_timeseries.Name = 'Lateral Position (x1 (m))';
x1_actual_timeseries.TimeInfo.Units = 'secs';
x2_actual_timeseries = timeseries(out.x_curr.Data(start_time_idx:end,2),...
                      out.x_curr.Time(start_time_idx:end));
x2_actual_timeseries.Name = 'Longitudinal Position (x2 (m))';
x2_actual_timeseries.TimeInfo.Units = 'secs';
x3_actual_timeseries = timeseries(out.x_curr.Data(:,3), out.x_curr.Time);
x4_actual_timeseries = timeseries(out.x_curr.Data(:,4), out.x_curr.Time);

%Ego car control inputs actual  
mv_acc_actual_timeseries = timeseries(out.mv.Data(:,1), out.x_curr.Time);
mv_steerang_actual_timeseries = timeseries(out.mv.Data(:,2), out.x_curr.Time);

%%%Plotting everything%%%%%%%
figure;
plot(x1_actual_timeseries);
figure;
plot(x2_actual_timeseries);
figure;
plot(mv_steerang_actual_timeseries);

figure;
hold all;
%Plot X trajectory    
c = rescale(1:size(x_pred_timeseries.Time));
plot(x_actual_timeseries.Data(:,1), x_actual_timeseries.Data(:,2));
for i = 1:4:size(x_pred_timeseries.Time)
    %Plot reference 
    scatter(x_ref_timeseries.Data(:,1), x_ref_timeseries.Data(:,2), 'x',...
            'MarkerFaceColor',[0 c(i) 0.5]);   
    %Plot obstacle polygon
    plgn = polyshape(obstcl_timeseries.Data(:,1,i), obstcl_timeseries.Data(:,2,i));
    plot(plgn);
    %Plot inflated obstacle ellipse
    n=ellip_coeff_timeseries.Data(i,6);
    a = ellip_coeff_timeseries.Data(i,1); b = ellip_coeff_timeseries.Data(i,2); 
    xe = ellip_coeff_timeseries.Data(i,3); ye = ellip_coeff_timeseries.Data(i,4); phi = ellip_coeff_timeseries.Data(i,5);
    ellip = @(x,y) ((((x-xe).*cos(phi) - (y-ye).*sin(phi)))./a).^n + ((((x-xe).*sin(phi) + (y-ye).*cos(phi)))./b).^n - 1;
    fimplicit(ellip)
    
    %Plot actual 
    plgn = polyshape(ego_car_timeseries.Data(:,1,i), ego_car_timeseries.Data(:,2,i));
    plot(plgn,'FaceColor','none');
    
    %Plot Predicted trajectory
    scatter(x_pred_timeseries.Data(:,1,i),x_pred_timeseries.Data(:,2,i), 36, '.',...
            'MarkerFaceColor',[0 c(i) 0.5]);
end

hold off;