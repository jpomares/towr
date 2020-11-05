#ifndef TALOS_MODEL_H_FOK8T27I
#define TALOS_MODEL_H_FOK8T27I

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of Talos (TODO: Real values)
 */
class TalosKinematicModel : public KinematicModel {
public:
  TalosKinematicModel () : KinematicModel(2)
  {
    const double z_nominal_b = -0.7;
    const double y_nominal_b =  0.20;

    nominal_stance_.at(L) << 0.0,  y_nominal_b, z_nominal_b;
    nominal_stance_.at(R) << 0.0, -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_  << 0.25, 0.15, 0.15;
  }
};

/**
 * @brief The Dynamics of a Talos (TODO: Real values, Dynamics of legs)
 */
class TalosDynamicModel : public SingleRigidBodyDynamics {
public:
  TalosDynamicModel()
  : SingleRigidBodyDynamics(20,
                    1.209,5.583,6.056,0.005,-0.190,-0.012,
                    2) {}
};

} /* namespace towr */

#endif /* end of include guard: TALOS_MODEL_H_FOK8T27I */
