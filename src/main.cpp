#include <memory>
#include <thread>
#include <chrono>
#include "Controller.hpp"

using namespace std::chrono;

int main()
{
    std::unique_ptr<Controller> controller = std::make_unique<Controller>();
    // Set initial conditions
    for (int i = 0; i < NX; i++)
    {
        controller->x0[i] = 0.0;
    }

    for (int i = 0; i < 1000; i++)
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
        // Stop at steady state
        if (controller->qp_iter_ == 0)
        {
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
