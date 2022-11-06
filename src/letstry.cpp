#include <phase_manager/phase_manager.h>
# include <iostream>

int main()
{

    SinglePhaseManager app(10);

    Phase stance(5, "stance");

    app.registerPhase(stance);

    app.addPhase(stance);
    std::cout << app.getRegisteredPhase("penis") << std::endl;

    app._shift_phases();

}
