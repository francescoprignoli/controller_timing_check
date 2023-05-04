// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_controller.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include <iostream>

#define NX CONTROLLER_NX
#define NU CONTROLLER_NU
#define NBX0 CONTROLLER_NBX0
#define NY CONTROLLER_NY

class Controller
{
public:
    controller_solver_capsule *acados_ocp_capsule;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;

    double x0[NX];
    int N = CONTROLLER_N;
    double min_time_ = 1e12;
    double max_time_ = -1e12;
    double avg_time_ = 0.0;
    double time_tot_ = 0.0;
    double time_reg_ = 0.0;
    double time_lin_ = 0.0;
    double time_qp_sol_ = 0.0;
    double elapsed_time_ = 0.0;
    int sqp_iter_ = 0;
    int qp_iter_ = 0;
    int n_timings_ = 0;
    int rti_phase_ = 0;
    int status_ = 0;
    double xtraj[NX * (CONTROLLER_N + 1)];
    double utraj[NU * CONTROLLER_N];

    Controller()
    {
        acados_ocp_capsule = controller_acados_create_capsule();
        // there is an opportunity to change the number of shooting intervals in C without new code generation

        // allocate the array and fill it accordingly
        double *new_time_steps = NULL;
        int status = controller_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

        if (status)
        {
            printf("controller_acados_create() returned status %d. Exiting.\n", status);
            exit(1);
        }

        nlp_config = controller_acados_get_nlp_config(acados_ocp_capsule);
        nlp_dims = controller_acados_get_nlp_dims(acados_ocp_capsule);
        nlp_in = controller_acados_get_nlp_in(acados_ocp_capsule);
        nlp_out = controller_acados_get_nlp_out(acados_ocp_capsule);
        nlp_solver = controller_acados_get_nlp_solver(acados_ocp_capsule);
        nlp_opts = controller_acados_get_nlp_opts(acados_ocp_capsule);

        // initial condition
        int idxbx0[NBX0];
        idxbx0[0] = 0;
        idxbx0[1] = 1;
        idxbx0[2] = 2;

        double lbx0[NBX0];
        double ubx0[NBX0];
        lbx0[0] = 0;
        ubx0[0] = 0;
        lbx0[1] = 0;
        ubx0[1] = 0;
        lbx0[2] = 0;
        ubx0[2] = 0;

        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

        // initialization for state values
        double x_init[NX];
        x_init[0] = 0.0;
        x_init[1] = 0.0;
        x_init[2] = 0.0;

        x0[0] = 0.0;
        x0[1] = 0.0;
        x0[2] = 0.0;
        printf("\n--- x0 ---\n");
        d_print_exp_tran_mat(NX, 1, x0, NX);

        // initial value for control input
        double u0[NU];
        u0[0] = 0.0;

        // initialize solution
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);

        // set reference
        double yref[NY];
        yref[0] = 0.0;
        yref[1] = 20.0;
        yref[2] = 0.0;
        yref[3] = 0.0;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
        }
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref);
    };

    void solve()
    {
        // set initial conditions
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x0);

        // solve
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase_);
        status_ = controller_acados_solve(acados_ocp_capsule);

        // get stat
        get_stat();

        // get solution
        for (int ii = 0; ii <= nlp_dims->N; ii++)
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii * NX]);
        for (int ii = 0; ii < nlp_dims->N; ii++)
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii * NU]);
    };

    void get_stat()
    {
        int stat_m, stat_n;
        sqp_iter_ = 0;
        qp_iter_ = 0;

        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &time_tot_);
        ocp_nlp_get(nlp_config, nlp_solver, "time_reg", &time_reg_);
        ocp_nlp_get(nlp_config, nlp_solver, "time_lin", &time_lin_);
        ocp_nlp_get(nlp_config, nlp_solver, "time_qp_sol", &time_qp_sol_);
        ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter_);
        ocp_nlp_get(nlp_config, nlp_solver, "stat_n", &stat_n);
        ocp_nlp_get(nlp_config, nlp_solver, "stat_m", &stat_m);

        double stat[1200];
        ocp_nlp_get(nlp_config, nlp_solver, "statistics", stat);

        int nrow = sqp_iter_ + 1 < stat_m ? sqp_iter_ + 1 : stat_m;

        for (int i = 0; i < nrow; i++)
        {
            qp_iter_ += (int)stat[i + 6 * nrow];
        }
        min_time_ = MIN(time_tot_, min_time_);
        max_time_ = MAX(time_tot_, max_time_);
        elapsed_time_ += time_tot_;
        n_timings_++;
        avg_time_ = elapsed_time_ / n_timings_;
    };

    void print_stat()
    {
        printf("qp_iter = %d \t time tot = %.2f [ms] \t time reg = %.2f [ms] \t time lin = %.2f [ms] \t time qp = %.2f [ms] \n", qp_iter_, time_tot_ * 1000, time_reg_ * 1000, time_lin_ * 1000, time_qp_sol_ * 1000);
    }
    ~Controller()
    {
        // free solver
        int status = controller_acados_free(acados_ocp_capsule);
        if (status)
        {
            printf("controller_acados_free() returned status %d. \n", status);
        }
        // free solver capsule
        status = controller_acados_free_capsule(acados_ocp_capsule);
        if (status)
        {
            printf("controller_acados_free_capsule() returned status %d. \n", status);
        }
        // Print current state
        printf("\n--- x ---\n");
        d_print_exp_tran_mat(NX, 1, xtraj, NX);
        // timing
        printf("min time = %f [ms] \t max time = %f [ms] \t avg time = %f [ms]\n ", min_time_ * 1000, max_time_ * 1000, avg_time_ * 1000);
    };
};
