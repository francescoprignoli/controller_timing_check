function make_Controller_mex()
mex_file = 'Controller_mex.cpp';
[mex_folder,~,~] = fileparts(which(mex_file));
current_dir = cd(mex_folder);

% get acados installation path
acados_install_dir = getenv('ACADOS_INSTALL_DIR');
src_folder = fullfile(mex_folder,'../src');
% include directories
include_folder = fullfile(mex_folder,'../include');
acados_include = ['-I' fullfile(acados_install_dir, 'include')];
blasfeo_include = ['-I', fullfile(acados_install_dir, 'external', 'blasfeo', 'include')];
hpipm_include = ['-I', fullfile(acados_install_dir, 'external', 'hpipm', 'include')];
% link directories
acados_lib_path = ['-L' fullfile(acados_install_dir, 'lib')];
% add executables
exec1 = fullfile(src_folder, 'acados_solver_controller.c');
exec2 = fullfile(src_folder, 'controller_dyn_disc_phi_fun.c');
exec3 = fullfile(src_folder, 'controller_dyn_disc_phi_fun_jac.c');
exec4 = fullfile(src_folder, 'controller_dyn_disc_phi_fun_jac_hess.c');

%% compile mex
disp(['compiling ', mex_file])
FLAGS = 'CXXFLAGS=$CXXFLAGS -std=c++17 -fpermissive';
mex(FLAGS, ['-I' include_folder], acados_include, blasfeo_include, hpipm_include, ...
    acados_lib_path, '-lacados', '-lhpipm', '-lblasfeo', mex_file, exec1, exec2, exec3, exec4)

cd(current_dir)
end