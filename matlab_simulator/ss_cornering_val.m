function ss_cornering_val
% Finds the solutions for all the cornering equilibrium states
% and stores in excel file or mat file

global vehicle
load('vehicle.mat')

global v_x delta
v_x = 1.5;

Fyf = @(rw, v_y, psi_dot) tire_dyn_f(atan2((v_y+vehicle.L_f*psi_dot),curr_v_x())-curr_delta());
Fyr = @(rw, v_y, psi_dot) tire_dyn_ry(curr_v_x(), rw, atan2((v_y-vehicle.L_r*psi_dot),curr_v_x()));
Fxr = @(rw, v_y, psi_dot) tire_dyn_rx(curr_v_x(), rw, atan2((v_y-vehicle.L_r*psi_dot),curr_v_x()));
F = @(V) [ (Fyf(V(1),V(2),V(3))*cos(curr_delta())+Fyr(V(1),V(2),V(3)))/(vehicle.m*curr_v_x())-V(3); ...
           (vehicle.L_f*Fyf(V(1),V(2),V(3))*cos(curr_delta())-vehicle.L_r*Fyr(V(1),V(2),V(3)))/vehicle.I_z; ...
           (Fxr(V(1),V(2),V(3))-Fyf(V(1),V(2),V(3))*sin(curr_delta()))/(vehicle.m*V(2))+V(3) ];

Options = optimoptions('fsolve'); 
Options.MaxIterations = 1000;
Options.MaxFunctionEvaluations = 5000;
Options.Display = 'off';
Options.Algorithm = 'levenberg-marquardt';

ssval = table();
i = 1;
%filename = ['ss_cornering_val.xlsx'];
%arr = [ {'v_x'} {'steering angle'} {'v_y'} {'psi_dot'} {'rw'} {'sb0'} ];
%xlswrite(filename,arr,'A1:F1');

%for del = -30:5:30
for del = 0:5:30
    %for no = -1:1
    for no = [-1 1]
        delta = degtorad(del);
        switch no
            case 0
                if del < 0
                    InitialGuess = [1.5;-0.05;-0.3];
                else
                    InitialGuess = [1.5;0.05;0.2];
                end
            case -1
                InitialGuess = [3;1;-1.5];
            case 1
                InitialGuess = [3;-1;1.5];
        end
        
        sol = fsolve(F, InitialGuess, Options);
        sb0 = F(sol);
        
        ssval{i,:} = [sol(1),degtorad(del),1.5,sol(2),sol(3)];
        i = i+1;
        %arr = [ {'1.5'} {num2str(del)} {num2str(sol(2),'%.9f')} ...
        %        {num2str(sol(3),'%.9f')} {num2str(sol(1),'%.9f')} ...
        %        {num2str(sb0(1),'%.4f')} {num2str(sb0(2),'%.4f')} {num2str(sb0(3),'%.4f')} ];
        %xlswrite(filename,arr,['A' num2str(i) ':H' num2str(i)]);
    end
end
ssval.Properties.VariableNames = {'rw' 'delta' 'v_x' 'v_y' 'psi_dot'};
save('ss_cornering_val.mat','ssval');
end


function ret = tire_dyn_ry(v_x, wheel_v, alpha)
    [~, ret] = tire_dyn_r(v_x, wheel_v, alpha);
end
function ret = tire_dyn_rx(v_x, wheel_v, alpha)
    [ret, ~] = tire_dyn_r(v_x, wheel_v, alpha);
end
function ret = curr_delta()
    global delta
    ret = delta;
end
function ret = curr_v_x()
    global v_x
    ret = v_x;
end
function ret = curr_v_y()
    global v_y
    ret = v_y;
end
function ret = curr_psi_dot()
    global psi_dot
    ret = psi_dot;
end
            