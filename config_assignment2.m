%% set up
m = 6;
d = 11;

l = 3.75+d/20;
rr = 0.465+m/200;

%% loaded case
m1_ld = 6500+200*(12-m);
m2_ld = 11500-200*(12-m);
k_ld = (136+d)/5/(84+d);

%% unloaded case
m1_uld = 4800+100*(12-m);
m2_uld = 3700-100*(12-m);
k_uld = 22/(90+d);

%% brake force calculation

%{
cylinder_type:  1= type 14;   2= type 16;   3= type 20;   4= type 24;   5= type 27;
pressure:       [0,10.5];
%} 
function B=brake_force_calc(cylinder_type, pressure)

    f_cylinder=[4500 5700 7000 8200 9700];

    B=f_cylinder(cylinder_type)*(pressure-0.3)/(6.1-0.3)*3.95/rr;

end

