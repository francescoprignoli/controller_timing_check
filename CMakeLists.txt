cmake_minimum_required(VERSION 3.8)
project(Controller VERSION 1.0 LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE "Release")

set(acados_install_dir $ENV{ACADOS_INSTALL_DIR})
set(acados_include_path ${acados_install_dir}/include)
set(acados_lib_path ${acados_install_dir}/lib)
set(blasfeo_path ${acados_include_path}/blasfeo/include)
set(hpipm_path ${acados_include_path}/hpipm/include)

add_executable(controller_timing_check 
src/main.cpp
src/acados_solver_controller.c
src/controller_dyn_disc_phi_fun.c
src/controller_dyn_disc_phi_fun_jac.c
src/controller_dyn_disc_phi_fun_jac_hess.c
)

target_link_directories(controller_timing_check
PUBLIC
${acados_lib_path})

target_link_libraries(controller_timing_check
acados
hpipm
blasfeo)

target_include_directories(controller_timing_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${acados_include_path}
  ${blasfeo_path}
  ${hpipm_path}
  )

target_compile_features(controller_timing_check PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS controller_timing_check
  DESTINATION lib/${PROJECT_NAME})
