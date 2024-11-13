#ifndef GZ_SIM_SYSTEMS_REVERSIBLEMULTICOPTERMOTORMODEL_HH_
#define GZ_SIM_SYSTEMS_REVERSIBLEMULTICOPTERMOTORMODEL_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class ReversibleMulticopterMotorModelPrivate;

  /// \brief This system applies a thrust force to models with spinning
  /// propellers, considering both normal and reverse motor directions.
  class ReversibleMulticopterMotorModel
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: ReversibleMulticopterMotorModel();

    /// \brief Destructor
    public: ~ReversibleMulticopterMotorModel() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<ReversibleMulticopterMotorModelPrivate> dataPtr;
  };
  }
}
}
}

#endif
