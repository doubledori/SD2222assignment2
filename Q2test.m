%% Set up
m = 7; % Month of birth
d = 13; % Date of birth
a = -6; % Deceleration value (m/s^2)
g = 9.81; % Gravitational acceleration (m/s^2)

l = 3.75 + d/20; % Wheelbase (m)
rr = 0.465 + m/200; % Rolling radius (m)
% lambda = 0.6; % Distance between front axle and COG

%% Loaded case
m1_ld = 6500 + 200 * (12 - m); % Mass on the front axle (kg)
m2_ld = 11500 - 200 * (12 - m); % Mass on the rear axle (kg)
lambda_ld = m2_ld/(m1_ld+m2_ld);
k_ld = (136 + d) / 5 / (84 + d); % Loaded case dimensionless parameter

%% Unloaded case
m1_uld = 4800 + 100 * (12 - m); % Mass on the front axle (kg)
m2_uld = 3700 - 100 * (12 - m); % Mass on the rear axle (kg)
lambda_uld = m2_uld/(m1_uld+m2_uld);
k_uld = 22 / (90 + d); % Unloaded case dimensionless parameter

%% Task 1 Calculations
gamma = -a / g; % Dimensionless deceleration
ar=-a; % retardation value
% For Loaded Case
N1_ld = (m1_ld+m2_ld) * g * (1 - lambda_ld) + (m1_ld+m2_ld) * ar * k_ld; % Equation (3)
N2_ld = (m1_ld+m2_ld) * g * lambda_ld - (m1_ld+m2_ld) * ar * k_ld; % Equation (4)
beta_opt_ld = 1 - lambda_ld + k_ld * gamma; % Optimal brake force distribution for loaded case

% For Unloaded Case
N1_uld = (m1_uld+m2_uld) * g * (1 - lambda_uld) + (m1_uld+m2_uld) * ar * k_uld; % Equation (3)
N2_uld = (m1_uld+m2_uld) * g * lambda_uld - (m1_uld+m2_uld) * ar * k_uld; % Equation (4)
beta_opt_uld = 1 - lambda_uld + k_uld * gamma; % Optimal brake force distribution for unloaded case


%% Task 2 --- loaded

m_tot = [(m1_ld + m2_ld), (m1_uld + m2_uld)];
pressure = 0.1:0.1:10.5;
p=pressure;
f_cylinder=[4500 5700 7000 8200 9700];

isld = 1;   % loaded:1, unloaded:2


% seitei
cylinder1 = 3;
cylinder2 = 1;

B1 = brake_force_calc(cylinder1, pressure,rr);
B2 = brake_force_calc(cylinder2, bprf(pressure, m_tot(isld)),rr);
miu1 = B1 / N1_ld;
miu2 = B2 / N2_ld;
gamma = (B1 + B2) / m_tot(isld) / g;
for i=1:length(pressure)
    if au(gamma, miu1, miu2)==0
        flag=0;
        break
    end
end

isFulfill = 0;

gamma_sup_ld=[0,0.8/7.3*(p(2:75)-0.2),p(76:105)];
gamma_inf_ld=[-10*ones([1,9]),0.1*(p(10:44)-1),(0.575-0.35)/(7.5-4.5)*(p(45:75)-4.5)+0.35,0*p(76:105)];
gamma_sup_uld=[0,0.8/5.3*(p(2:55)-0.2),p(56:105)];
gamma_inf_uld=[-10*ones([1,9]),0.4/3.5*(p(10:44)-1),(0.65-0.4)/(7.5-4.5)*(p(45:75)-4.5)+0.4,0*p(76:105)];

i11=arraycomp(gamma_sup_uld,gamma);
i22=arraycomp(gamma,gamma_inf_uld);
isFulfill=i11*i22;


if isFulfill == 0
    flag=0;
end

figure(1);
subplot(1,2,1);
plot(p,gamma,p,gamma_sup_ld,p,gamma_inf_ld,'LineWidth',2);
legend({"γ","γ_{sup} loaded","γ_{inf} loaded"}, ...
    'Location','southeast')
xlabel 'p';
ylabel '\Sigma B_i/\Sigma N_i';
xlim([0,7.5]);
ylim([0,0.8]);
title('brake force corridor -- loaded case')
grid on

figure(2)
subplot(1,2,1);
plot(gamma,miu1,gamma,miu2,LineWidth=2);
hold on;
grid on;
plot(gamma_bg,miu_gnr_bg,"-",gamma_bg,miu_sup_bg,"--", ...
    gamma_bg,miu_inf_bg,"--",gamma_bg,miu_amid_bg,":",LineWidth=2);
xlabel('x');
ylabel('k(x)');
xlim([0,0.8]);
ylim([0,0.8]);
legend({'\mu_1 loaded','\mu_2 loaded','k=(x+0.07)/0.85','k=x+0.08 & k=(x-0.02)/0.74','k=x-0.08','k=x'},'Location','southeast');
title('adhesion utilization -- loaded case')

%% Task 2 --- unloaded

isld = 2;   % loaded:1, unloaded:2

% seitei
cylinder1 = 3;      % 前气缸型号
cylinder2 = 1;      % 后气缸型号

B1 = brake_force_calc(cylinder1, pressure,rr);
B2 = brake_force_calc(cylinder2, bprf(pressure, m_tot(isld)),rr);
miu1 = B1 / N1_uld;
miu2 = B2 / N2_uld;
gamma = (B1 + B2) / m_tot(isld) / g;
for i=1:length(pressure)
    if au(gamma, miu1, miu2)==0
        flag=0;
        break
    end
end

isFulfill = 0;

gamma_sup_ld=[0,0.8/7.3*(p(2:75)-0.2),p(76:105)];
gamma_inf_ld=[-10*ones([1,9]),0.1*(p(10:44)-1),(0.575-0.35)/(7.5-4.5)*(p(45:75)-4.5)+0.35,0*p(76:105)];
gamma_sup_uld=[0,0.8/5.3*(p(2:55)-0.2),p(56:105)];
gamma_inf_uld=[-10*ones([1,9]),0.4/3.5*(p(10:44)-1),(0.65-0.4)/(7.5-4.5)*(p(45:75)-4.5)+0.4,0*p(76:105)];

i11=arraycomp(gamma_sup_uld,gamma);
i22=arraycomp(gamma,gamma_inf_uld);
isFulfill=i11*i22;


if isFulfill == 0
    flag=0;
end

figure(1)
subplot(1,2,2);
plot(p,gamma,p,gamma_sup_uld,p,gamma_inf_uld,'LineWidth',2);
legend({"γ","γ_{sup} unloaded","γ_{inf} unloaded"}, ...
    'Location','southeast')
xlabel 'p';
ylabel '\Sigma B_i/\Sigma N_i';
xlim([0,7.5]);
ylim([0,0.8]);
title('brake force corridor -- unloaded case')
grid on


gamma_bg=0.01:0.01:0.8;
miu_gnr_bg=[nan(1,9),(gamma_bg(10:61)+0.07)/0.85,nan(1,19)];
miu_sup_bg=[nan(1,14),gamma_bg(15:30)+0.08,(gamma_bg(31:62)-0.02)/0.74,nan(1,18)];
miu_inf_bg=[nan(1,14),gamma_bg(15:30)-0.08,nan(1,50)];
miu_amid_bg=gamma_bg;

figure(2)
subplot(1,2,2);
plot(gamma,miu1,gamma,miu2,LineWidth=2);
hold on;
grid on;
plot(gamma_bg,miu_gnr_bg,"-",gamma_bg,miu_sup_bg,"--", ...
    gamma_bg,miu_inf_bg,"--",gamma_bg,miu_amid_bg,":",LineWidth=2);
xlabel('x');
ylabel('k(x)');
xlim([0,0.8]);
ylim([0,0.8]);
legend({'\mu_1 unloaded','\mu_2 unloaded','k=(x+0.07)/0.85','k=x+0.08 & k=(x-0.02)/0.74','k=x-0.08','k=x'},'Location','southeast');
title('adhesion utilization -- unloaded case')


%% Display results
% disp(['Optimal brake force distribution for loaded case: ', num2str(beta_opt_ld)]);
% disp(['Optimal brake force distribution for unloaded case: ', num2str(beta_opt_uld)]);
% disp(['N1_ld ', num2str(N1_ld)]);
% disp(['N1_uld ', num2str(N1_uld)]);
% disp(['N2_ld ', num2str(N2_ld)]);
% disp(['N2_uld ', num2str(N2_uld)]);

%% brake force calculation --- Appdx. 2

%{
cylinder_type:  1= type 14;   2= type 16;   3= type 20;   4= type 24;   5= type 27;
pressure:       [0,10.5];
%} 

function B=brake_force_calc(cylinder_type, pressure, rr)

    f_cylinder=[4500 5700 7000 8200 9700];
    

    B=[0,0,f_cylinder(cylinder_type)*(pressure(3:105)-0.3)/(6.1-0.3)*3.95/rr];

end


%% Requirements --- Adhesion utilisation

function isFulfill=au(gamma, miu1, miu2)

    isFulfill = 0;
    miu_gnr = (gamma + 0.07) / 0.85;

    if gamma>0.15
        if gamma<0.3
            miu_sup = gamma + 0.08;
            miu_inf = gamma - 0.08;
            if and(sign((miu_sup-miu1)*(miu1-miu_inf))>0, ...
                    sign((miu_sup-miu2)*(miu2-miu_inf))>0)
                isFulfill = 1;
                return
            elseif miu1>miu2
                isFulfill = 1;
                return
            end
        elseif gamma<0.6
            miu_sup = (gamma - 0.02) / 0.74;
            miu_inf = 0;
            if and(sign((miu_sup-miu1)*(miu1-miu_inf))>0, ...
                    sign((miu_sup-miu2)*(miu2-miu_inf))>0)
                isFulfill = 1;
                return
            end
        else
            if and(sign(miu_gnr-miu1)>0,sign(miu_gnr-miu2)>0)
               isFulfill = 1;
               return
            end
        end
    else
        isFulfill = 1; 
    end

end

%% Requirements --- Brake force corridor

% function isFulfill=bfc(gamma, p, isld) % grade=0%

% end

%% array compare

function anybigger=arraycomp(a1, a2)    % every element in a1 should be bigger, then output will be 1
    
    anybigger = 1;
    for i = 1:length(a1)
        if a2(i)>a1(i)
            anybigger = 0;
            return
        end
    end

end
%% Brake pressure reduction factor

function p_out=bprf(p_in, m_tot)


    tgtprop = 0.56;
    p_out = (1-tgtprop)*p_in/9500*(m_tot-8500)+tgtprop*p_in;
    % plot(p_in,p_out);
    % title('Brake pressure reduction factor on REAR AXLE');
    % xlabel 'p\_in'
    % ylabel 'p\_out'


end
