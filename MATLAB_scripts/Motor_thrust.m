%% Prediction
close all

V = [12;16;24];
T = [3.5;5.7;9.2].*9.8;


P = polyfit(V, T, 1);                           % Linear Fit
a = P(1);
Tfit = polyval(P, V);

SStot = sum((T-mean(T)).^2);                    % Total Sum-Of-Squares
SSres = sum((T-Tfit).^2);                       % Residual Sum-Of-Squares
Rsq = 1-SSres/SStot; 

figure;
hold on
scatter(V,T, 'filled')
plot(V,Tfit,'--')
title('Manufacturer Motor Thrust vs. Voltage')
xlabel('Voltage (V)');
ylabel('Thrust (N)');
annotation('textbox', [0.425, 0.2, 0.1, 0.1], 'String', ['Conversion Rate: ',num2str(P(1)), 'N/V - ' ,num2str(abs(P(2))), 'N'])
annotation('textbox', [0.425, 0.125,0.1,0.1], 'String',['R Squared: ', num2str(Rsq)])
hold off

disp(['Conversion Rate: ',num2str(a), ' N/V'])

%% Measured Vals

t = [.29;.24;.27;.48;.62;.53;.94;1.04;.85].*9.8./2;
D = [.2;.2;.2;.3;.3;.3;.4;.4;.4].*22.4;
V_test = [.2;.3;.4].*22.4;

pfit = polyfit(D,t,1)
Tfit3 = polyval(pfit,V_test);

SStot = sum((T-mean(T)).^2);                    % Total Sum-Of-Squares
SSres = sum((T-Tfit).^2);                       % Residual Sum-Of-Squares
Rsq = 1-SSres/SStot; 

figure;
hold on
scatter(D,t, 'filled')
plot(V_test,Tfit3,'--')
title('Measured Motor Thrust vs. Voltage')
xlabel('Voltage (V)');
ylabel('Thrust (N)');
annotation('textbox', [0.425, 0.2, 0.1, 0.1], 'String', ['Conversion Rate: ',num2str(pfit(1)), 'N/V - ' ,num2str(abs(pfit(2))), 'N'])
annotation('textbox', [0.425, 0.125,0.1,0.1], 'String',['R Squared: ', num2str(Rsq)])
hold off


%% Comparison
total_T = [t;T];
total_V = [D;V];

P2 = polyfit(total_V, total_T, 1);                           % Linear Fit
Tfit2 = polyval(P2, total_V);

SStot2 = sum((total_T-mean(total_T)).^2);                    % Total Sum-Of-Squares
SSres2 = sum((total_T-Tfit2).^2);                       % Residual Sum-Of-Squares
Rsq2 = 1-SSres2/SStot2

figure;
hold on
scatter(D,t, 'filled','b')
scatter(V,T,'filled','r')
plot(total_V,Tfit2,'--')
title('Measured and Given Motor Thrust vs. Voltage')
xlabel('Voltage (V)');
ylabel('Thrust (N)');
xlim([0,25]);
ylim([0,100])
annotation('textbox', [0.6, 0.125, 0.1, 0.1], 'String', ['Conversion Rate: ',num2str(P2(1)), 'N/V - ' ,num2str(abs(P2(2))), 'N'])
legend('Measured', 'Given', 'Location','northwest');
hold off