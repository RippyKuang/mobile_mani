#include "core.h"

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

namespace RBD = RigidBodyDynamics;
int main()
{
    RBD::Model *model = new RBD::Model();
    if (!RBD::Addons::URDFReadFromFile("../ridgeback_ur5/urdf/ridgeback_ur5.urdf", model, true))
    {
        std::cerr << "Error loading model " << std::endl;
        abort();
    }
    std::cout << RBD::Utils::GetModelDOFOverview(*model);

    start_simulate("../ridgeback_ur5/mjcf/scene.xml");
}