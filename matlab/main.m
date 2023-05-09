%% Reproduce the timing issues in matlab
clear Controller_mex

for i = 1:1000
    out = Controller_mex();
    fprintf('Measured time = %.2f [ms]\n', out.time_span*1000);
    if out.qp_iter == 0
        break;
    end
    pause(0.1)
end