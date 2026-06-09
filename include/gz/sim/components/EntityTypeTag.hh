#ifndef GZ_SIM_COMPONENTS_ENTITYTYPETAG_HH_
#define GZ_SIM_COMPONENTS_ENTITYTYPETAG_HH_

#include "gz/sim/Types.hh"
#include "gz/sim/components/Component.hh"
#include "gz/sim/components/Factory.hh"
#include "gz/sim/config.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  using EntityTypeTag = Component<EntityType, class EntityTypeTagTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.EntityTypeTag", EntityTypeTag)
}
}
}
}

#endif