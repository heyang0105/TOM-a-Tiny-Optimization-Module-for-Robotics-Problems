#ifndef OPTIM_UTIL_TICTOC_H
#define OPTIM_UTIL_TICTOC_H

#include <ctime>
#include <cstdlib>
#include <chrono>

namespace Optim{
class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

} //namespace Optim

#endif
