

#ifndef FLUIDS_LEAPFROG_H
#define FLUIDS_LEAPFROG_H


#include "../System.h"
#include "Solver.h"

class LeapFrog  : public Solver {
public:
    void simulateStep(System* system, float h) override;
private:
    bool firstStep = true;
};


#endif //FLUIDS_LEAPFROG_H
