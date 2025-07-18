#include <gtest/gtest.h>

#include <gz/common/MeshManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/SystemLoader.hh>
#include <sdf/Element.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

TEST(SingletonTest, MeshmanagerAddress) {
  auto* meshManager = gz::common::MeshManager::Instance();
  gzerr << "Meshmanager address: \t" << meshManager << std::endl;

  constexpr std::string_view kPluginSdf = R"(
  <?xml version="1.0"?>
  <sdf version="1.12">
    <world name="default">
      <plugin filename="my-system" name="gz::sim::systems::MySystem" />
    </world>
  </sdf>)";
  ::sdf::Root sdf_root;
  auto errors = sdf_root.LoadSdfString(std::string(kPluginSdf));
  ASSERT_TRUE(errors.empty());
  const auto* world = sdf_root.WorldByIndex(0);
  auto pluginElementPtr = world->Element()->GetFirstElement();

  gz::sim::SystemLoader loader;
  auto plugin = loader.LoadPlugin(world->Plugins()[0]);
  ASSERT_NE(plugin, std::nullopt);
}
