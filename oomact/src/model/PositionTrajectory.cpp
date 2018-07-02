#include <aslam/calibration/model/PositionTrajectory.h>

#include <boost/make_shared.hpp>
#include <glog/logging.h>

#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>
#include <aslam/calibration/calibrator/CalibrationConfI.h>
#include <bsplines/NsecTimePolicy.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/Transformation.hpp>

#include "aslam/calibration/algo/OdometryPath.h"
#include "aslam/calibration/calibrator/CalibratorI.h"
#include <aslam/calibration/DesignVariableReceiver.h>
#include "aslam/calibration/data/PoseMeasurement.h"
#include <aslam/calibration/error-terms/ErrorTermTangency.h>
#include <aslam/calibration/model/Model.h>
#include <aslam/calibration/model/ModuleTools.h>
#include <aslam/calibration/model/fragments/R3Trajectory.h>
#include <aslam/calibration/model/sensors/PostionSensor.h>
#include <aslam/calibration/tools/ErrorTermStatisticsWithProblemAndPredictor.h>
#include <aslam/calibration/tools/MeasurementContainerTools.h>

using bsplines::NsecTimePolicy;
using sm::kinematics::Transformation;
using sm::timing::NsecTime;

namespace aslam {
namespace calibration {


Eigen::Vector4d negateQuatIfThatBringsItCloser(const Eigen::Vector4d& pquat, const
    Eigen::Vector4d& cquat) {
  if ((pquat + cquat).norm() < (pquat - cquat).norm()) {
    return -cquat;
  } else {
    return cquat;
  }
}

class BaseTrajectoryBatchState : public BatchState, public R3Trajectory {
 public:
  BaseTrajectoryBatchState(PositionTrajectory & baseTrajectory)
    : R3Trajectory(baseTrajectory)
  {}

  void writeToFile(const CalibratorI & calib, const std::string & pathPrefix) const override;
};


constexpr double TAN_CONSTRAINT_VARIANCE_DEFAULT = 1e-8;

PoseTrajectory::PoseTrajectory(Model& model, const std::string& name, sm::value_store::ValueStoreRef config) :
  Module(model, name, config),
  R3TrajectoryCarrier(getMyConfig().getChild("splines"), model.getFrame(getMyConfig().getString("frame"))),
  estimate(getMyConfig().getBool("estimate", true)),
  useTanConstraint(getMyConfig().getBool("tangentialConstraint/used", false)),
  tanConstraintVariance(getMyConfig().getDouble("tangentialConstraint/variance", TAN_CONSTRAINT_VARIANCE_DEFAULT)),
  initWithPosMeasurements(getMyConfig().getBool("initWithPosMeasurements", false)),
  posSensor(*this, "PosSensor", initWithPosMeasurements),
  assumeStatic(getMyConfig().getBool("assumeStatic", false)),
  referenceFrame_(model.getFrame(getMyConfig().getString("referenceFrame")))
{
  if(isUsed()){
    if(!estimate){
      LOG(WARNING) << "Not going to estimate the " << getName() << "!";
    } else {
      LOG(INFO) << "Going to estimate the " << getName() << ".";
    }
  }
}

void PoseTrajectory::writeConfig(std::ostream& out) const {
  MODULE_WRITE_PARAM(estimate);
  MODULE_WRITE_PARAM(assumeStatic);
  MODULE_WRITE_PARAM(initWithPosMeasurements);
  MODULE_WRITE_PARAM(posSensor);
  MODULE_WRITE_PARAM(referenceFrame_);
  So3R3TrajectoryCarrier::writeConfig(out);
  if(useTanConstraint){
    MODULE_WRITE_PARAM(tanConstraintVariance);
  }
}


bool PoseTrajectory::initState(CalibratorI& calib) {
  state_ = std::make_shared<BaseTrajectoryBatchState>(*this);

  if(assumeStatic){
    LOG(INFO) << getName() << " : Initializing statically!";
    getCurrentTrajectory().initSplinesConstant(calib.getCurrentEffectiveBatchInterval(), 1);
    return true;
  } else {
    if(initWithPoseMeasurements){
      if(poseSensor.isResolved()){
        return initSplines(calib, getCurrentTrajectory(), poseSensor, getReferenceFrame());
      } else {
        throw std::runtime_error(getName() + ".initWithPoseMeasurements is true but " + poseSensor.toString() + " is not resolved!");
      }
    }
    CHECK(odometrySensor.isResolved()) << getName() << ".initWithPoseMeasurements is false but " << odometrySensor.toString() << " is not resolved!";
    return initSplines(calib, getCurrentTrajectory(), odometrySensor, poseSensor);
  }
}

void PoseTrajectory::addToBatch(const Activator & stateActivator, BatchStateReceiver & batchStateReceiver, DesignVariableReceiver & problem) {
  const bool stateActive = estimate && !assumeStatic && stateActivator.isActive(*this);
  if(!estimate){
    LOG(WARNING) << "Not going to estimate " << getName() << "!";
  }
  if(stateActive){
    LOG(INFO) << "Activating " << getName() << "'s splines.";
  }
  CHECK(state_);
  state_->addToProblem(stateActive, problem);
  batchStateReceiver.addBatchState(*this, state_);
}

void PoseTrajectory::addErrorTerms(CalibratorI & calib, const CalibrationConfI & ec, ErrorTermReceiver & problem) const {
  if(useTanConstraint && state_){
    LOG(INFO) << "Adding soft constraints error terms.";

    const bool observerOnly = !ec.getStateActivator().isActive(*this) && !ec.getCalibrationActivator().isActive(*this);

    auto & trajectory = getCurrentTrajectory();

    Timestamp
      minTime = calib.getCurrentEffectiveBatchInterval().start,
      maxTime = calib.getCurrentEffectiveBatchInterval().end;

    const double elapsedTime = maxTime - minTime;

    const int numSegments = std::ceil(getKnotsPerSecond() * 2 * elapsedTime);
    const double tangentialVariance = tanConstraintVariance;

    ErrorTermStatisticsWithProblemAndPredictor statWPAP(calib, "TangentialConstraint", problem, observerOnly);
    ErrorTermGroupReference etgr(statWPAP.getName());
    for (int i = 0; i < numSegments + 1 ; i++) {
      Timestamp timestamp = minTime + Timestamp((double)i * elapsedTime / numSegments);

      auto translationExpressionFactory = trajectory.getTranslationSpline().getExpressionFactoryAt<1>(timestamp);
      auto rotationExpressionFactory = trajectory.getRotationSpline().getExpressionFactoryAt<0>(timestamp);

      aslam::backend::EuclideanExpression v_r_mwl, v_r_mwr;
      aslam::backend::RotationExpression R_m_r;

      R_m_r = aslam::backend::Vector2RotationQuaternionExpressionAdapter::adapt(rotationExpressionFactory.getValueExpression());
      const auto v_m_mr = translationExpressionFactory.getValueExpression(1);
      const auto v_r_mr = R_m_r.inverse() * v_m_mr;

      // Is it missing a constraint on the direction of the velocity?? By construction quaternion spline is not constrained to be tangent to the pose
      // We should add that constraint. Possibly in Jerome case we could not see that since there was an error term on the veloctiy,
      // That was constraining the local direction of versor i, in robot (vehicle frame). Otherwise rotation is not exactly well constrained

      // TODO: B Add a constraint on the velocity to be parallel to the orientation i x v = 0
      auto tangency_constraint = v_r_mr.cross(aslam::backend::EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)));
      auto e_tan = boost::make_shared<ErrorTermTangency>(tangency_constraint, Eigen::Vector3d(tangentialVariance, tangentialVariance, tangentialVariance).asDiagonal(), etgr);
      statWPAP.add(timestamp, e_tan);
    }
    statWPAP.printInto(LOG(INFO));
  }
}

void BaseTrajectoryBatchState::writeToFile(const CalibratorI & calib, const std::string& pathPrefix) const {
  So3R3Trajectory::writeToFile(calib, pathPrefix);
}

PoseTrajectory::~PoseTrajectory() {
}

const So3R3Trajectory& PoseTrajectory::getCurrentTrajectory() const {
  CHECK(state_);
  return *state_;
}

So3R3Trajectory& PoseTrajectory::getCurrentTrajectory() {
  CHECK(state_);
  return *state_;
}

} /* namespace calibration */
} /* namespace aslam */
