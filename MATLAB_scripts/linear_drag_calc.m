close all
clear all

load('IMU_data/lin_drag.mat')

%% Segmenting the data by time intervals
t_all = [29.7 37; 40.5 47; 4 10]';  % [t0_1 t0_2 ...; t1_1 t1_2 ...]
new_t = {};
new_acc = {}; % Each segment will get it's own cell. Ex: new_acc = {A1, A2, A3}

for i = 1:size(t_all, 2)
    indices = t>t_all(1, i) & t <t_all(2, i);
    new_t{i} = t(indices)-t_all(1, i);
    new_acc{i} = -acc(indices);
end


%% Fitting the curve 
% Our parameter vector will be x = [C0; v0; bias1; bias2; bias3]
% Each curve has its own bias but they share C0 and v0

m = 28.7; 
options = optimset('TolFun',1e-15,'MaxFunEvals',1e5,'Maxiter',1e5,'Display','iter');
q = fsolve(@(q)residual(q, m, new_acc, new_t), [20; 0.4; -0.5; -0.5; -0.5], options); 


%% Plotting results 
figure;
for i = 1:size(t_all, 2)
    acc_est = EOM(m, q(1), q(2), q(2+i), new_t{i}); 
    %figure 
    subplot(1,size(t_all,2),i) 
    plot(new_t{i}, new_acc{i}, new_t{i}, acc_est)
    grid on
    %daspect([0.5 0.1 1])
    axis([0 7.5 -1.5 0.3])
    if i == size(t_all, 2)
        legend('Linear Acceleration','Estimated Fit')
    end
end 
han = axes(gcf,'visible','off'); 
han.XLabel.Visible = 'on';
han.YLabel.Visible = 'on';

% Set common labels
xlabel(han, 'Time (s)');
ylabel(han, '$$\mathbf{Linear Acceleration\ (m/s^2)}$$', 'Interpreter', 'latex');
sgtitle('Deceleration Profile for Linear Drag Coefficient')
disp(q)

%% Functions 
function res = residual(q, m, A, tspan)

    C0 = abs(q(1));
    v0 = abs(q(2)); 

    res = []; 
    for i = 1:size(A, 2)
        bias = q(2+i); 

        res = [res; (A{i} - EOM(m, C0, v0, bias, tspan{i}))]; % [error_curve_1; error_curve_2; error_curve_3]
    end 

end 


function acc = EOM(m, C0, v0, bias, tspan)
    acc = -(sqrt(C0/m).*tspan + sqrt(m/C0)/v0).^-2 + bias;
end 