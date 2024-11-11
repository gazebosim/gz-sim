#ifndef GZ_SIM_SYSTEMS_DRIVETOPOSECONTROLLER_HH_
#define GZ_SIM_SYSTEMS_DRIVETOPOSECONTROLLER_HH_

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
  // Forward declaration
  class DriveToPoseControllerPrivate;

  /// \brief DriveToPoseController is a simple proportional controller that
  /// is attached to a model to move it in the  y giving a pose in Gazebo's
  /// world coordinate system. This is not a standalone plugin, and requires
  /// the DiffDrive and OdometryPublisher plugins respectively.
  ///
  /// The plugin has the following tags:
  ///
  /// - `<linear_p_gain>`: (Optional) Proportional gain for the linear velocity controller
  ///                       Default: 1.0
  ///
  /// - `<angular_p_gain>`: (Optional) Proportional gain for the angular velocity controller
  ///                       Default: 2.0
  ///
  /// - `<linear_deviation>`: (Optional) Allowable linear deviation (in meters) from the desired coordinate
  ///                         Default: 0.1
  ///
  /// - `<angular_deviation>`: (Optional) Allowable angular deviation (in radians) from the desired orientation.
  ///                          Default: 0.05
  class DriveToPoseController
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: DriveToPoseController();

    /// \brief Destructor
    public: ~DriveToPoseController() override = default;

    // Documentation inherited
    public: void Configure(
                  const Entity& _entity,
                  const std::shared_ptr<const sdf::Element>& _sdf,
                  EntityComponentManager& _ecm,
                  EventManager& _eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(
                  const UpdateInfo& _info,
                  const EntityComponentManager& _ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<DriveToPoseControllerPrivate> dataPtr;
  };
}
}
}
}

#endif
