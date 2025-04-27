close all
clear all

load('IMU_data/ang_drag_2.mat')

%% Segmenting the data by time intervals
% t_all = [7.59 14; 30.73 37.1]';  % [t0_1 t0_2 ...; t1_1 t1_2 ...] 
% FOR ANG_1
t_all = [10.58 17.14; 47.7 56.1; 74.46 80.6]';
new_t = {};
new_w = {}; % Each segment will get it's own cell. Ex: new_acc = {A1, A2, A3}

for i = 1:size(t_all, 2)
    indices = t>t_all(1, i) & t <t_all(2, i);
    new_t{i} = t(indices)-t_all(1, i);
    new_w{i} = abs(w(indices));
end


%% Fitting the curve 
% Our parameter vector will be x = [C0; v0; bias1; bias2; bias3]
% Each curve has its own bias but they share C0 and v0

I = 5.4; 
options = optimset('TolFun',1e-15,'MaxFunEvals',1e5,'Maxiter',1e5,'Display','iter');
q = fsolve(@(q)residual(q, I, new_w, new_t), [20; 0.42; 0.02; -0.02; -0.02], options); 


%% Plotting results 
figure;
hold on
for i = 1:size(t_all, 2)
    w_est = EOM(I, q(1), q(2), q(2+i), new_t{i}); 
    % figure 
    subplot(1,size(t_all,2),i) 
    plot(new_t{i}, new_w{i}, new_t{i}, w_est)
    grid on
    %daspect([1 0.015 1])
    axis([0 8 0 0.5])
    if i == size(t_all, 2)
        legend('Angular Velocity','Estimated Fit')
    end
end 
han = axes(gcf,'visible','off'); 
han.XLabel.Visible = 'on';
han.YLabel.Visible = 'on';

% Set common labels
xlabel(han, 'Time (s)');
ylabel(han, 'Angular Velocity (m/s)');
sgtitle('Deceleration Profile for Angular Drag Coefficient')

hold off
disp(q)


%% Functions 
function res = residual(q, I, W, tspan)

    C0 = abs(q(1));
    w0 = abs(q(2)); 

    res = []; 
    for i = 1:size(W, 2)
        bias = q(2+i); 

        res = [res; (W{i} - EOM(I, C0, w0, bias, tspan{i}))]; % [error_curve_1; error_curve_2; error_curve_3]
    end 

end 


function w = EOM(I, C0, w0, bias, tspan)
    w = ((C0/I).*tspan + 1/w0).^-1 + bias;
end 