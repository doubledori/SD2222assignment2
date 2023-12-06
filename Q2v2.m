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


%% Task 1 Display results
fprintf('Task 1:\n')
disp(['Optimal brake force distribution for loaded case: ', num2str(beta_opt_ld)]);
disp(['Optimal brake force distribution for unloaded case: ', num2str(beta_opt_uld)]);
disp(['N1_ld ', num2str(N1_ld)]);
disp(['N1_uld ', num2str(N1_uld)]);
disp(['N2_ld ', num2str(N2_ld)]);
disp(['N2_uld ', num2str(N2_uld)]);

%% Task 2.1 --- loaded

m_tot = [(m1_ld + m2_ld), (m1_uld + m2_uld)];           % total mass
pressure = 0.1:0.1:10.5;                                % brake system pressure, resolution=0.1 bar
f_cylinder=[4500 5700 7000 8200 9700];                  % force of the pneumatic brake cylinders are given for a brake pressure of 6.1 bar
flag_ld = ones(length(f_cylinder),length(f_cylinder));  % value of elements: 1 means the combination of cylinder satisfies all reqs in appdx.2, otherwise 0. Loaded case
flag_uld = ones(length(f_cylinder),length(f_cylinder)); % value of elements: 1 means the combination of cylinder satisfies all reqs in appdx.2, otherwise 0. Unloaded case

% % kuwashii genindachi
% flag_ac_ld = ones(length(f_cylinder),length(f_cylinder));
% flag_bfc_ld = ones(length(f_cylinder),length(f_cylinder));
% flag_ac_uld = ones(length(f_cylinder),length(f_cylinder));
% flag_bfc_uld = ones(length(f_cylinder),length(f_cylinder));

isld = 1;                                               % loaded:1, unloaded:2

for cylinder1 = 1:length(f_cylinder)
    B1 = brake_force_calc(cylinder1, pressure,rr);
    for cylinder2 = 1:length(f_cylinder)
        B2 = brake_force_calc(cylinder2, pressure, rr);
        miu1 = B1 / N1_ld;
        miu2 = B2 / N2_ld;
        gamma = (B1 + B2) / m_tot(isld) / g;
        for i=1:length(pressure)
            [str_ld(cylinder1,cylinder2),sgn] = au(gamma, miu1, miu2);
            if sgn==0
                flag_ld(cylinder1,cylinder2)=0;
                % flag_ac_ld(cylinder1,cylinder2)=0;
                
            end
        end
        if bfc(gamma, pressure, isld)==0
            flag_ld(cylinder1,cylinder2)=0;
            % flag_bfc_ld(cylinder1,cylinder2)=0;
        end
    end
end



%% Task 2.1 --- unloaded

isld = 2;                                               % loaded:1, unloaded:2

for cylinder1 = 1:length(f_cylinder)
    B1 = brake_force_calc(cylinder1, pressure, rr);
    for cylinder2 = 1:length(f_cylinder)
        B2 = brake_force_calc(cylinder2, pressure, rr);
        miu1 = B1 / N1_uld;
        miu2 = B2 / N2_uld;
        gamma = (B1 + B2) / m_tot(isld) / g;
        for i=1:length(pressure)
            [str(cylinder1,cylinder2),sgn] = au(gamma, miu1, miu2);
            if sgn==0
                flag_uld(cylinder1,cylinder2)=0;
                % flag_ac_uld(cylinder1,cylinder2)=0;
                
            end
        end
        if bfc(gamma, pressure, isld)==0
            flag_uld(cylinder1,cylinder2)=0;
            % flag_bfc_uld(cylinder1,cylinder2)=0;
        end
    end
end

%% Task 2.2 --- brake pressure reduction factor applied --- loaded

m_tot = [(m1_ld + m2_ld), (m1_uld + m2_uld)];
pressure = 0.1:0.1:10.5;
f_cylinder=[4500 5700 7000 8200 9700];
cylinder_type=[14 16 20 24 27];
flag_ld2 = ones(length(f_cylinder),length(f_cylinder));
flag_uld2 = ones(length(f_cylinder),length(f_cylinder));

% kuwashii genin
flag_ac_ld2 = ones(length(f_cylinder),length(f_cylinder));
flag_bfc_ld2 = ones(length(f_cylinder),length(f_cylinder));
flag_ac_uld2 = ones(length(f_cylinder),length(f_cylinder));
flag_bfc_uld2 = ones(length(f_cylinder),length(f_cylinder));

isld = 1;                                               % loaded:1, unloaded:2

for cylinder1 = 1:length(f_cylinder)
    B1 = brake_force_calc(cylinder1, pressure,rr);
    for cylinder2 = 1:length(f_cylinder)
        B2 = brake_force_calc(cylinder2, bprf(pressure, m_tot(isld)), rr);
        miu1 = B1 / N1_ld;
        miu2 = B2 / N2_ld;
        gamma = (B1 + B2) / m_tot(isld) / g;
        for i=1:length(pressure)
            [str(cylinder1,cylinder2),sgn] = au(gamma, miu1, miu2);
            if sgn==0
                flag_ld2(cylinder1,cylinder2)=0;
                flag_ac_ld2(cylinder1,cylinder2)=0;
                
            end
        end
        if bfc(gamma, pressure, isld)==0
            flag_ld2(cylinder1,cylinder2)=0;
            flag_bfc_ld2(cylinder1,cylinder2)=0;
        end
    end
end



%% Task 2.2 --- brake pressure reduction factor applied --- unloaded

isld = 2;                                               % loaded:1, unloaded:2


% plot x ~ k(x)

gamma_bg=0.01:0.01:0.8;
miu_gnr_bg=[nan(1,9),(gamma_bg(10:61)+0.07)/0.85,nan(1,19)];
miu_sup_bg=[nan(1,14),gamma_bg(15:30)+0.08,(gamma_bg(31:62)-0.02)/0.74,nan(1,18)];
miu_inf_bg=[nan(1,14),gamma_bg(15:30)-0.08,nan(1,50)];
miu_amid_bg=gamma_bg;

i1=zeros(5);
i2=zeros(5);

for cylinder1 = 1:length(f_cylinder)
    B1 = brake_force_calc(cylinder1, pressure, rr);
    for cylinder2 = 1:length(f_cylinder)
        B2 = brake_force_calc(cylinder2, bprf(pressure, m_tot(isld)), rr);
        miu1 = B1 / N1_uld;
        miu2 = B2 / N2_uld;
        gamma = (B1 + B2) / m_tot(isld) / g;
        [str_uld(cylinder1,cylinder2),sgn] = au(gamma, miu1, miu2);
        if sgn==0
            flag_uld2(cylinder1,cylinder2)=0;
            flag_ac_uld2(cylinder1,cylinder2)=0;

        end
        gamma_sup_ld=[0,0.8/7.3*(p(2:75)-0.2),p(76:105)];
        gamma_inf_ld=[-10*ones([1,9]),0.1*(p(10:44)-1),(0.575-0.35)/(7.5-4.5)*(p(45:75)-4.5)+0.35,0*p(76:105)];
        gamma_sup_uld=[0,0.8/5.3*(p(2:55)-0.2),p(56:105)];
        gamma_inf_uld=[-10*ones([1,9]),0.4/3.5*(p(10:44)-1),(0.65-0.4)/(7.5-4.5)*(p(45:75)-4.5)+0.4,0*p(76:105)];
        
        i1(cylinder1,cylinder2)=arraycomp(gamma_sup_uld,gamma);
        i2(cylinder1,cylinder2)=arraycomp(gamma,gamma_inf_uld);
        isFulfill=i1(cylinder1,cylinder2)*i2(cylinder1,cylinder2);

        if isFulfill==0
            flag_uld2(cylinder1,cylinder2)=0;
            flag_bfc_uld2(cylinder1,cylinder2)=0;
        end
    end
end


%% Task 2 visualization

fprintf(['\n----\nTask2:\n' ...
    'It is impossible to fulfil the requirements with the same brake force distribution for the loaded and the unloaded case, ' ...
    'and a brake pressure reduction factor must be applied.\n' ...
    'Adoptable combinations of brake cylinders:\n']);

% figure(1);
% plot(gamma_bg,miu_gnr_bg,"-",gamma_bg,miu_sup_bg,"--", ...
%     gamma_bg,miu_inf_bg,"--",gamma_bg,miu_amid_bg,":","Color","k",LineWidth=2);
% xlabel('x');
% ylabel('k(x)');
% xlim([0,0.8]);
% ylim([0,0.8]);
% hold on;
% grid on;

counter=1;
for ia=1:length(f_cylinder)
    for ib=1:length(f_cylinder)
        if flag_ld2(ia,ib)*flag_uld2(ia,ib)==1
            fprintf('#%d\t: front: type %d;\trear: type %d\n', ...
                counter,cylinder_type(ia),cylinder_type(ib));
            counter=counter+1;
        end
    end
end
fprintf(['We decide to choose combination #1 for following tasks, while the brake pressure reduction factor is 0.56.\n' ...
    'A brake pressure reduction factor applied, as shown in the figure.\n']);

%% brake force calculation --- Appdx. 2

%{
cylinder_type:  1= type 14;   2= type 16;   3= type 20;   4= type 24;   5= type 27;
pressure:       [0,10.5];
%} 

function B=brake_force_calc(cylinder_type, pressure, rr)

    f_cylinder=[4500 5700 7000 8200 9700];
    
    B=f_cylinder(cylinder_type)*(pressure-0.3)/(6.1-0.3)*3.95/rr;

end


%% Requirements --- Adhesion utilisation

%{
requirements from diagram1, appdx.3
%}

function [error,isFulfill]=au(gamma_array, miu1_array, miu2_array)

    isFulfill = 1;
    error="no";
    % miu_gnr_array = (gamma_array + 0.07) / 0.85;

    for i=1:length(gamma_array)

        gamma=gamma_array(i);
        miu1=miu1_array(i);
        miu2=miu2_array(i);

        if gamma>0.15
            if gamma<0.3
                miu_sup = gamma + 0.08;
                miu_inf = gamma - 0.08;
                if or((miu_sup-miu1)*(miu1-miu_inf)<0, ...
                        (miu_sup-miu2)*(miu2-miu_inf)<0)
                    if miu1<miu2
                        isFulfill = 0;
                        error="1";
                        return
                    end
                end
            elseif gamma<0.6
                miu_sup = (gamma - 0.02) / 0.74;
                if or(miu_sup<miu1, miu_sup<miu2)
                    isFulfill = 0;
                    error="2";
                    return
                end
            elseif gamma>0.1
                miu_gnr = (gamma + 0.07) / 0.85;
                if or(miu_gnr<miu1, miu_gnr<miu2)
                   isFulfill = 0;
                   error="3";
                   return
                end
            end
        end
    end

    % plot(gamma_array, miu1_array, gamma_array, miu2_array);
    % hold on

end

%% Requirements --- Brake force corridor

%{
requirements from diagram2, appdx.3
%}

function isFulfill=bfc(gamma, p, isld) % grade=0%

    isFulfill = 0;


% gamma_sup_ld=[0,0.8/7.3*(p(2:75)-0.2),p(76:105)];
% gamma_inf_ld=[-10*ones([1,9]),0.1*(p(10:44)-1),(0.575-0.35)/(7.5-4.5)*(p(45:75)-4.5)+0.35,0*p(76:105)];
% gamma_sup_uld=[0,0.8/5.3*(p(2:55)-0.2),p(56:105)];
% gamma_inf_uld=[-10*ones([1,9]),0.4/3.5*(p(10:44)-1),(0.65-0.4)/(7.5-4.5)*(p(45:75)-4.5)+0.4,0*p(76:105)];
    
    if isld==1
        gamma_sup_ld=[0,0.8/7.3*(p(2:75)-0.2),p(76:105)];
        % the part "0" and "p(76:105)" serves as placeholders

        gamma_inf_ld=[-10*ones([1,9]),0.1*(p(10:44)-1),(0.575-0.35)/(7.5-4.5)*(p(45:75)-4.5)+0.35,0*p(76:105)];
        % the part "-10*ones([1,9])" and "0*p(76:105)" serves as placeholders

        i1=arraycomp(gamma_sup_ld,gamma);
        i2=arraycomp(gamma,gamma_inf_ld);
        isFulfill=i1*i2;
        % isFulfill=arraycomp(gamma_sup_ld,gamma)*arraycomp(gamma,gamma_inf_ld);

        return

    elseif isld==2
        gamma_sup_uld=[0,0.8/5.3*(p(2:55)-0.2),p(56:105)];
        % the part "0" and "p(56:105)" serves as placeholders

        gamma_inf_uld=[-10*ones([1,9]),0.4/3.5*(p(10:44)-1),(0.65-0.4)/(7.5-4.5)*(p(45:75)-4.5)+0.4,0*p(76:105)];
        % the part "-10*ones([1,9])" and "0*p(76:105)" serves as placeholders
        
        i1=arraycomp(gamma_sup_uld,gamma);
        i2=arraycomp(gamma,gamma_inf_uld);
        isFulfill=i1*i2;
        % isFulfill=and(arraycomp(gamma_sup_uld,gamma),arraycomp(gamma,gamma_inf_uld));

        return

    else
        fprintf("#02 invalid isld\n");
    end

end

%% array compare

% every element in array a1 should be bigger, otherwise the output turns to 0

function anybigger=arraycomp(a1, a2)    
    
    anybigger = 1;
    
    for i = 1:length(a1)
        if a2(i)>a1(i)
            anybigger = 0;
            return
        end
    end

end

%% Brake pressure reduction factor

%{
a load sensing proportioning valve that acts on the rear axle
%}

function p_out=bprf(p_in, m_tot)

    tgtprop = 0.56;
    p_out = (1-tgtprop)*p_in/9500*(m_tot-8500)+tgtprop*p_in;
    figure(2);
    plot(p_in,p_out);
    title('Brake pressure reduction factor on REAR AXLE');
    xlabel 'p\_in'
    ylabel 'p\_out'

end
