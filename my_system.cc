#include <gz/common/MeshManager.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>

namespace gz::sim {

class MySystem : public System {
 public:
  MySystem() {
    auto* meshManager = gz::common::MeshManager::Instance();
    gzerr << "Meshmanager address: \t" << meshManager << std::endl;
  }
};
}  // namespace gz::sim

GZ_ADD_PLUGIN(gz::sim::MySystem, gz::sim::System)

GZ_ADD_PLUGIN_ALIAS(gz::sim::MySystem, "gz::sim::systems::MySystem")
