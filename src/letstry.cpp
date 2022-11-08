#include <phase_manager/phase_manager.h>
# include <iostream>

int main()
{

    SinglePhaseManager app(12);

    Phase::PhasePtr stance = std::make_shared<Phase>(5, "stance");


//    stance.addConstraint

    app.registerPhase(stance);
    app.addPhase(stance);
    app.addPhase(stance);
    app.addPhase(stance);
//    app.getRegisteredPhase("penis");

//    app._shift_phases();

    app.addPhase(stance);
//    app._shift_phases();
//    app._shift_phases();
//    app._shift_phases();
//    app._shift_phases();
//    app._shift_phases();
//    app._shift_phases();
//    app._shift_phases();
//    app._shift_phases();
//    app._shift_phases();
//    app._shift_phases();
//    app.addPhase(stance);

}
