#include "../external/neuromeka-package/cpp/neuromeka_cpp/indydcp3.h"
#include <iostream>

int main() {
    IndyDCP3 indy("160.39.102.52");

    // return to zero position
    // indy.movej({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // get home position
    Nrmk::IndyFramework::JointPos home_jpos;
    indy.get_home_pos(home_jpos);
    std::cout << "Home position: " << home_jpos.jpos(0) << ", " << home_jpos.jpos(1) << ", " << home_jpos.jpos(2) << ", " << home_jpos.jpos(3) << ", " << home_jpos.jpos(4) << ", " << home_jpos.jpos(5) << std::endl;

    // convert home_jpos to std::vector<float>  
    std::vector<float> home_jpos_vec(home_jpos.jpos().begin(), home_jpos.jpos().end());

    // move to home position
    // indy.movej(home_jpos_vec);

    // move to target position in task space
    std::array<float, 6> target_pose = {350.0, -186.5, 522.0, -180.0, 0.0, 180.0}; // equivalent to home position
    indy.movel(target_pose);

    indy.wait_progress(100);

    return 0;
}