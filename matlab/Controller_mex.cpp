#include "mex.hpp"
#include "mexAdapter.hpp"
#include "Controller.hpp"
#include <string>
#include <memory>
#include <chrono>

using matlab::mex::ArgumentList;
using namespace matlab::data;
using namespace std::chrono;

class MexFunction : public matlab::mex::Function
{
    std::unique_ptr<Controller> controller;

    // Pointer to MATLAB engine
    std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();

    // Factory to create MATLAB data arrays
    ArrayFactory factory;

public:
    MexFunction()
    {
        controller = std::make_unique<Controller>();
        // Set initial conditions
        for (int i = 0; i < NX; i++)
        {
            controller->x0[i] = 0.0;
        }
        matlabPtr->feval(u"disp",
                         0,
                         std::vector<Array>({factory.createScalar("Created Controller object.")}));
    }

    ~MexFunction()
    {
        matlabPtr->feval(u"disp",
                         0,
                         std::vector<Array>({factory.createScalar("Destroyed Controller object.")}));
    }

    void operator()(ArgumentList outputs, ArgumentList inputs)
    {
        // Start measuring time
        auto start = high_resolution_clock::now();
        // Call to solver
        controller->solve();
        // Stop measuring time
        auto stop = high_resolution_clock::now();
        duration<double> time_span = duration_cast<duration<double>>(stop - start);
        // Advance initial conditions
        for (int i = 0; i < NX; i++)
        {
            controller->x0[i] = controller->xtraj[i + NX * 1];
        }
        // Print stats
        controller->print_stat();
        printf("Measured time = %.2f [ms]\n\n", time_span.count() * 1000.0);

        StructArray S = factory.createStructArray({1, 1}, {"time_span", "qp_iter"});
        S[0]["time_span"] = factory.createScalar(time_span.count());
        S[0]["qp_iter"] = factory.createScalar(controller->qp_iter_);
        outputs[0] = S;
    }
};