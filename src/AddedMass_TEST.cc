/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <gz/transport/Node.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>

#include <gz/utils/ExtraTestMacros.hh>

#include "../test/helpers/EnvTestFixture.hh"

const char *kWorldFilePath{"/test/worlds/added_mass_full_matrix.sdf"};
const char *kWorldName{"added_mass"};
const char *kModelName{"body"};
const char *kLinkName{"link"};

// Struct to define test inputs.
struct TestInputs {
  gz::math::Vector3d pos;
  gz::math::Quaternion<double> quat;
  gz::math::Vector3d lin_vel;
  gz::math::Vector3d ang_vel;
  gz::math::Vector3d force;
  gz::math::Vector3d torque;
};

// Struct to define test outputs.
struct TestOutputs {
  gz::math::Vector3d lin_acc;
  gz::math::Vector3d ang_acc;
};

// Struct to define test cases (inputs and outputs).
struct TestCase {
  TestInputs inputs;
  TestOutputs outputs;
};

// Test case #1
const TestCase kTestCase1{
  // Inputs
  TestInputs{
    // pos
    gz::math::Vector3d(
      -0.2425045525045464,
      0.33840815652747214,
      -0.15400580887792237
    ),
    // quat
    gz::math::Quaternion<double>(
      -0.2809847940832897,
      -0.0670318259306688,
      -0.6456490396607173,
      0.7068886739731832
    ),
    // lin_vel
    gz::math::Vector3d(
      -0.39036334852467514,
      -0.6319184973934608,
      0.5801096725227644
    ),
    // ang_vel
    gz::math::Vector3d(
      0.9586223737234041,
      0.3819110744112888,
      0.6723060779392143
    ),
    // force
    gz::math::Vector3d(
      -0.556637080115808,
      0.05794391212068817,
      -0.024721551602638447
    ),
    // torque
    gz::math::Vector3d(
      -0.751375873274468,
      0.02394774300273772,
      0.8181098350410643
    )
  },
  // outputs
  TestOutputs{
    // lin_acc
    gz::math::Vector3d(
      0.1192543571637511,
      1.2564925342515267,
      -1.5492015338978795
    ),
    // ang_acc
    gz::math::Vector3d(
      1.7232932412616608,
      -1.1261918994329594,
      -0.17792714753780803
    )
  }
};

// Test case #2
const TestCase kTestCase2{
  // Inputs
  TestInputs{
    // pos
    gz::math::Vector3d(
      -0.26627756134916636,
      -0.7084085992300324,
      0.10619830845744738
    ),
    // quat
    gz::math::Quaternion<double>(
      -0.3052948472230469,
      0.9269624883895875,
      0.2095263494451089,
      -0.0602852408624731
    ),
    // lin_vel
    gz::math::Vector3d(
      -0.7458530424796268,
      -0.1666649256184578,
      0.5386761683694596
    ),
    // ang_vel
    gz::math::Vector3d(
      0.17926634408173947,
      -0.12034245997240012,
      0.18516904536562961
    ),
    // force
    gz::math::Vector3d(
      -0.15924606852428447,
      -0.9253978615421046,
      -0.9435058673509571
    ),
    // torque
    gz::math::Vector3d(
      -0.13533482138078679,
      0.03828213442605044,
      -0.350293463974384
    )
  },
  // outputs
  TestOutputs{
    // lin_acc
    gz::math::Vector3d(
      0.42447316327242957,
      -0.7080776570562612,
      -1.2515188145460858
    ),
    // ang_acc
    gz::math::Vector3d(
      0.3114632455376882,
      -1.4883476738089614,
      0.9047781278329061
    )
  }
};

// Test case #3
const TestCase kTestCase3{
  // Inputs
  TestInputs{
    // pos
    gz::math::Vector3d(
      0.09486194918678748,
      -0.9099264683313666,
      0.39781798711093663
    ),
    // quat
    gz::math::Quaternion<double>(
      0.4474845382742349,
      -0.49632065154724486,
      0.5178323029314619,
      -0.5341096375220279
    ),
    // lin_vel
    gz::math::Vector3d(
      0.4774732593432809,
      0.9660133435982379,
      -0.7444575664048005
    ),
    // ang_vel
    gz::math::Vector3d(
      -0.6060956578366126,
      0.2115034622260703,
      0.9224794841428035
    ),
    // force
    gz::math::Vector3d(
      0.3454369446897425,
      0.6102403086661006,
      0.15097984754808258
    ),
    // torque
    gz::math::Vector3d(
      0.44071925082422103,
      -0.6126203759247251,
      0.9009309436931738
    )
  },
  // outputs
  TestOutputs{
    // lin_acc
    gz::math::Vector3d(
      0.24671175337681506,
      2.20705594933898,
      2.7611862436175336
    ),
    // ang_acc
    gz::math::Vector3d(
      3.253196299221303,
      -2.1915486730787777,
      2.4857656486324062
    )
  }
};

// Test case #4
const TestCase kTestCase4{
  // Inputs
  TestInputs{
    // pos
    gz::math::Vector3d(
      -0.021935430219797603,
      -0.2028619676525827,
      -0.7409964940861165
    ),
    // quat
    gz::math::Quaternion<double>(
      -0.21651876198721565,
      -0.8484495285682753,
      0.16290189513073497,
      0.4546603080791308
    ),
    // lin_vel
    gz::math::Vector3d(
      0.0422249657383702,
      -0.018269069242955682,
      -0.08889923134418964
    ),
    // ang_vel
    gz::math::Vector3d(
      0.5054783278488537,
      0.9409816563774551,
      -0.08228410518648444
    ),
    // force
    gz::math::Vector3d(
      0.5313699373453282,
      0.07366554324213204,
      -0.11253372373538961
    ),
    // torque
    gz::math::Vector3d(
      0.33872369018889503,
      0.6522277135523256,
      -0.5097460548473949
    )
  },
  // outputs
  TestOutputs{
    // lin_acc
    gz::math::Vector3d(
      0.23502926958161108,
      1.8946101802534714,
      -1.6216718048541643
    ),
    // ang_acc
    gz::math::Vector3d(
      -4.67488778909784,
      3.023629102947669,
      -4.88653326608196
    )
  }
};

// Test case #5
const TestCase kTestCase5{
  // Inputs
  TestInputs{
    // pos
    gz::math::Vector3d(
      0.9689583056406572,
      0.0017441433682259255,
      0.3493388951936456
    ),
    // quat
    gz::math::Quaternion<double>(
      -0.6634858326989433,
      -0.46031657032694107,
      0.4932102494131894,
      0.3234792957269624
    ),
    // lin_vel
    gz::math::Vector3d(
      0.5964142494679903,
      -0.3148625519885535,
      0.6209278519251678
    ),
    // ang_vel
    gz::math::Vector3d(
      -0.43590121461970166,
      0.9146114641552883,
      -0.5598223768473267
    ),
    // force
    gz::math::Vector3d(
      -0.2998167392689237,
      -0.2695157563478241,
      0.3651342901444177
    ),
    // torque
    gz::math::Vector3d(
      -0.7188323924902038,
      -0.7464458674658769,
      -0.029814015736676858
    )
  },
  // outputs
  TestOutputs{
    // lin_acc
    gz::math::Vector3d(
      0.90004165730203,
      0.7398170763472348,
      0.3374987284916062
    ),
    // ang_acc
    gz::math::Vector3d(
      -0.14893549985333834,
      -1.0660445662071873,
      -0.45274284576200496
    )
  }
};

// Vector of test cases.
const std::vector<TestCase> kTestCases{
  kTestCase1,
  kTestCase2,
  kTestCase3,
  kTestCase4,
  kTestCase5
};

// Number of test cases.
const std::size_t kNumCases{kTestCases.size()};


// Set the body's pose, velocity, and wrench, in PreUpdate; check reported
// accelerations in PostUpdate.
class AccelerationCheckPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemReset,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  public: AccelerationCheckPlugin() = default;

  public: void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr
  ) override;

  public: void Reset(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm
  ) override;

  public: void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm
  ) override;

  public: void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm
  ) override;

  // Whether to perform a check against the expected output values in
  // PostUpdate.
  public: bool performChecks{false};

  // Determines which test case to use when setting up input values in
  // PreUpdate, and when checking output values in PostUpdate.
  public: int caseNumber{0};

  // Used to ensure that all the tests have run.
  public: int testCounter{0};

  private: void InitializeModelAndLink(gz::sim::EntityComponentManager &_ecm);

  private: gz::sim::Entity model_entity{gz::sim::kNullEntity};

  private: gz::sim::Entity link_entity{gz::sim::kNullEntity};
};

// Used by Configure and Reset to initialize the body and link ECM state.
void AccelerationCheckPlugin::InitializeModelAndLink(
    gz::sim::EntityComponentManager &_ecm
  )
  {
  gzdbg << "Initializing model." << std::endl;
  if (this->model_entity == gz::sim::kNullEntity) {
    this-> model_entity = _ecm.EntityByComponents(
      gz::sim::components::Name(kModelName),
      gz::sim::components::Model()
    );
    ASSERT_NE(this->model_entity, gz::sim::kNullEntity);
  }
  gz::sim::Model model = gz::sim::Model(this->model_entity);
  ASSERT_TRUE(model.Valid(_ecm));
  ASSERT_NE(
    _ecm.CreateComponent(this->model_entity, gz::sim::components::WorldPose()),
    nullptr
  );

  gzdbg << "Initializing link." << std::endl;
  if (this->link_entity == gz::sim::kNullEntity) {
    this->link_entity = model.LinkByName(_ecm, kLinkName);
    ASSERT_NE(this->link_entity, gz::sim::kNullEntity);
  }
  gz::sim::Link link = gz::sim::Link(this->link_entity);
  ASSERT_TRUE(link.Valid(_ecm));
  ASSERT_NE(
    _ecm.CreateComponent(
      this->link_entity,
      gz::sim::components::WorldLinearVelocity()
    ),
    nullptr
  );
  ASSERT_NE(
    _ecm.CreateComponent(
      this->link_entity,
      gz::sim::components::WorldAngularVelocity()
    ),
    nullptr
  );
  ASSERT_NE(
    _ecm.CreateComponent(
      this->link_entity,
      gz::sim::components::WorldLinearAcceleration()
    ),
    nullptr
  );
  ASSERT_NE(
    _ecm.CreateComponent(
      this->link_entity,
      gz::sim::components::WorldAngularAcceleration()
    ),
    nullptr
  );
}

// Sets model and link ECM state.
void AccelerationCheckPlugin::Configure(
  const gz::sim::Entity &,
  const std::shared_ptr<const sdf::Element> &,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &
)
{
  gzdbg << "Configure happening." << std::endl;
  this->InitializeModelAndLink(_ecm);
}

// Sets model and link ECM state.
void AccelerationCheckPlugin::Reset(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm
)
{
  gzdbg << "Reset happening." << std::endl;
  this->InitializeModelAndLink(_ecm);
}

// Set pose, velocity, and wrench (force and torque).
void AccelerationCheckPlugin::PreUpdate(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm
)
{
  gzdbg << "PreUpdate happening." << std::endl;

  ASSERT_NE(this->model_entity, gz::sim::kNullEntity);
  gz::sim::Model model = gz::sim::Model(this->model_entity);
  ASSERT_TRUE(model.Valid(_ecm));
  ASSERT_NE(this->link_entity, gz::sim::kNullEntity);
  gz::sim::Link link = gz::sim::Link(this->link_entity);
  ASSERT_TRUE(link.Valid(_ecm));

  const TestInputs inputs = kTestCases[this->caseNumber].inputs;

  const gz::math::Vector3d pos = inputs.pos;
  const gz::math::Quaternion<double> quat = inputs.quat;
  const gz::math::Pose3d world_pose = gz::math::Pose3d(pos, quat);
  gzdbg << "Setting model world position to:\t" << pos << std::endl;
  gzdbg << "Setting model world orientation to:\t" << quat << std::endl;
  _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(
    this->model_entity,
    world_pose
  );

  const gz::math::Vector3d lin_vel = inputs.lin_vel;
  gzdbg << "Setting link world linear velocity to:\t" << lin_vel << std::endl;
  // `LinearVelocityCmd` sets linear velocity in the link's frame, and is
  // unaffected by the rotation performed by `WorldPoseCmd`.
  _ecm.SetComponentData<gz::sim::components::LinearVelocityCmd>(
    this->link_entity,
    lin_vel
  );
  const gz::math::Vector3d ang_vel = inputs.ang_vel;
  gzdbg << "Setting link world angular velocity to:\t" << ang_vel << std::endl;
  // `AngularVelocityCmd` sets angular velocity in the link's frame, and is
  // unaffected by the rotation performed by `WorldPoseCmd`.
  _ecm.SetComponentData<gz::sim::components::AngularVelocityCmd>(
    this->link_entity,
    ang_vel
  );

  const gz::math::Vector3d force = inputs.force;
  const gz::math::Vector3d torque = inputs.torque;
  gzdbg << "Setting link world force to: \t" << force << std::endl;
  gzdbg << "Setting link world torque to:\t" << torque << std::endl;
  // The entity component manager first registers the wrench with the link and
  // then applies the the rotation from `WorldPoseCmd`. This results in a world
  // wrench that is affected by the rotation in `WorldPoseCmd`, we need to
  // correct for this here.
  link.AddWorldWrench(_ecm, quat.Inverse() * force, quat.Inverse() * torque);
}

// Check linear and angular acceleration.
void AccelerationCheckPlugin::PostUpdate(
  const gz::sim::UpdateInfo &,
  const gz::sim::EntityComponentManager &_ecm
)
{
  gzdbg << "PostUpdate happening." << std::endl;

  if (this->performChecks) {
    gzdbg << "Performing checks." << std::endl;
    this->testCounter += 1;
    gzdbg << "Check number " << this->testCounter << std::endl;

    ASSERT_NE(this->link_entity, gz::sim::kNullEntity);
    gz::sim::Link link = gz::sim::Link(this->link_entity);
    ASSERT_TRUE(link.Valid(_ecm));
    TestOutputs outputs = kTestCases[this->caseNumber].outputs;

    std::optional<gz::math::Vector3d> maybe_lin_acc =
      link.WorldLinearAcceleration(_ecm);
    EXPECT_TRUE(maybe_lin_acc);
    gzdbg << "Expected world linear acceleration:\t" << outputs.lin_acc <<
      std::endl;
    if (maybe_lin_acc) {
      gz::math::Vector3d lin_acc = maybe_lin_acc.value();
      gzdbg << "Actual world linear acceleration:\t" << lin_acc << std::endl;
      EXPECT_LT((lin_acc - outputs.lin_acc).Length(), 1e-2);
    }
    else
    {
      gzdbg << "Unable to retrieve link world linear acceleration." <<
        std::endl;
    }

    std::optional<gz::math::Vector3d> maybe_ang_acc =
      link.WorldAngularAcceleration(_ecm);
    EXPECT_TRUE(maybe_ang_acc);
    gzdbg << "Expected world angular acceleration:\t" << outputs.ang_acc <<
      std::endl;
    if (maybe_ang_acc) {
      gz::math::Vector3d ang_acc = maybe_ang_acc.value();
      gzdbg << "Actual world angular acceleration:\t" << ang_acc <<
        std::endl;
      EXPECT_LT((ang_acc - outputs.ang_acc).Length(), 1e-2);
    }
    else
    {
      gzdbg << "Unable to retrieve link world angular acceleration." <<
        std::endl;
    }
  }
}

// Request a world reset via transport.
void requestWorldReset()
{
  std::string topic{"/world/" + std::string(kWorldName) + "/control"};
  gz::msgs::WorldControl req;
  gz::msgs::Boolean rep;
  req.mutable_reset()->set_all(true);
  gz::transport::Node node;

  gzdbg << "Requesting world reset." << std::endl;
  unsigned int timeout = 1000;
  bool result;
  bool executed =
    node.Request(topic, req, timeout, rep, result);
  ASSERT_TRUE(executed);
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());
}

class EmptyTestFixture: public InternalFixture<::testing::Test> {};

// Check that the accelerations reported for a body with added mass matche the
// expected values.
TEST_F(EmptyTestFixture,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(AddedMassAccelerationTest))
{
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(
    common::joinPaths(PROJECT_SOURCE_PATH, kWorldFilePath)
  );
  gz::sim::Server server = gz::sim::Server(serverConfig);
  std::shared_ptr<AccelerationCheckPlugin> accelerationChecker =
    std::make_shared<AccelerationCheckPlugin>();
  std::optional<bool> maybe_system_added =
    server.AddSystem(accelerationChecker);
  ASSERT_TRUE(maybe_system_added);
  if (maybe_system_added) {
    ASSERT_TRUE(maybe_system_added.value());
  }
  ASSERT_FALSE(server.Running());

  // Run one iteration in order to "initialize" physics.
  ASSERT_TRUE(server.RunOnce(false));

  // For each test case, reset the server and run checks.
  for (std::size_t i = 0; i < kNumCases; i++) {
    accelerationChecker->performChecks = false;
    accelerationChecker->caseNumber = i;
    requestWorldReset();
    // It takes two iterations for a reset to happen. Only perform checks once
    // it has happened.
    ASSERT_TRUE(server.RunOnce(false));  // Reset requested.
    ASSERT_TRUE(server.RunOnce(false));  // Reset happening.
    accelerationChecker->performChecks = true;
    ASSERT_TRUE(server.RunOnce(false));  // Checks happening.
  }

  // Ensure that the right number of checks have been performed.
  EXPECT_EQ(accelerationChecker->testCounter, kNumCases);
}
