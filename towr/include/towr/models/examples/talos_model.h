#ifndef TALOS_MODEL_H_FOK8T27I
#define TALOS_MODEL_H_FOK8T27I

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of Talos
 */
class TalosKinematicModel : public KinematicModel {
public:
  TalosKinematicModel () : KinematicModel(2)
  {
    const double z_nominal_b = -0.9;
    const double y_nominal_b =  0.10;

    nominal_stance_.at(L) << 0.0,  y_nominal_b, z_nominal_b;
    nominal_stance_.at(R) << 0.0, -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_  << 0.18, 0.10, 0.060;
  }
};

/**
 * @brief The Dynamics of a Talos
 */
class TalosDynamicModel : public SingleRigidBodyDynamics {
public:
  TalosDynamicModel()
  : SingleRigidBodyDynamics(93.3357,
			    18.581, 15.4115, 4.08092,
			    -0.0081011, -0.0267051, 0.0579719, 
			    2) {}
};

} /* namespace towr */

#endif /* end of include guard: TALOS_MODEL_H_FOK8T27I */
