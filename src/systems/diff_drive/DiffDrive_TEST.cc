# include <gtest/gtest.h>

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Namespace.hh"

# include "DiffDrive.cc"

TEST(DiffDriveTest, ResolvedTopicNames)
{
  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <plugin name='gz::sim::systems::DiffDrive'
              filename='gz-sim-diff-drive-system'>
        <topic>/test_cmd_vel</topic>
        <odom_topic>test_odom</odom_topic>
      </plugin>
    </sdf>)";
  ASSERT_TRUE(sdf::readString(sdfString, sdf));
  auto pluginElem = sdf->Root()->GetElement("plugin");
  
  EntityComponentManager ecm;

  auto modelEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelEntity, components::Model());
  ecm.CreateComponent(modelEntity, components::Name("diff_drive"));
  ecm.CreateComponent(modelEntity, components::Namespace("ns"));

  DiffDrivePrivate data;
  data.model = Model(modelEntity);

  std::string cmdVelTopic, enableTopic, odomTopic, tfTopic;
  data.ResolvedTopicNames(pluginElem, ecm,
      cmdVelTopic, enableTopic, odomTopic, tfTopic);

  EXPECT_EQ(cmdVelTopic, "/test_cmd_vel");
  EXPECT_EQ(enableTopic, "/ns/enable");
  EXPECT_EQ(odomTopic, "/ns/test_odom");
  EXPECT_EQ(tfTopic, "/ns/tf");
}