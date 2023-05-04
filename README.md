## This is a minimal mpc controller implemented with the acados framework. This program reproduces execution time issues when using a sleep() function: the longer the sleep time, the longer the execution time.

### Usage:
- update `env.sh` with your acados folder path and `source env.sh`
- `mkdir build && cd build`
- `cmake ..`
- `make`

### Issue:
The solver execution time increases with the sleep time (try 100ms vs 10ms)