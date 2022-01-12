#include <iostream>
#include "Simulator.h"
int main(int argc, char **argv)
{
    Simulator simulator;
    simulator.loadData();
    simulator.init();
    simulator.run();
}
