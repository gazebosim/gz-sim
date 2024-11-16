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
  // Using real world test with a reversible motor, we discovered the best way we had to simulate a reversible motor, was to use a third order model for both the thrust and the torque. This was done this way since the enviroment in which the real world robot would be used would have no wind. This also meant that we dont need to compute drag coeficients and rolling moment coeficients
  // Please note that this pluggin is not supposed to be general but insteal used with the Space Cobot model, since we tested our motor in testbench and used the following results to create the model if you want to used this model, you will need to do this 
  // polynomails are in the type of a0 + a1*x + a2*x^2 + a3*x^3 = torque
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
