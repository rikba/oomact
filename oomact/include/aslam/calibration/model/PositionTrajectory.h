#ifndef OOMACT_ASLAM_CALIBRATION_MODEL_POSITION_TRAJECTORY_H_
#define OOMACT_ASLAM_CALIBRATION_MODEL_POSITION_TRAJECTORY_H_

#include <aslam/calibration/calibrator/CalibrationConfI.h>
#include <aslam/calibration/model/Module.h>
#include <aslam/calibration/calibrator/StateCarrier.h>
#include <aslam/calibration/model/fragments/R3TrajectoryCarrier.h>

namespace aslam {
namespace calibration {

class BaseTrajectoryBatchState;
class R3Trajectory;
class PositionSensor;

class PositionTrajectory : public Module, public StateCarrier, public Activatable, public R3TrajectoryCarrier {
 public:
  PositionTrajectory(Model & model, const std::string & name, sm::value_store::ValueStoreRef config = sm::value_store::ValueStoreRef());

  bool initState(CalibratorI & calib) override;
  void addToBatch(const Activator & stateActivator, BatchStateReceiver & batchStateReceiver, DesignVariableReceiver & problem) override;
  void addErrorTerms(CalibratorI & calib, const CalibrationConfI & ec, ErrorTermReceiver & problem) const override;


  const R3Trajectory & getCurrentTrajectory() const;
  R3Trajectory & getCurrentTrajectory();

  virtual ~PositionTrajectory();

  bool isUseTangentialConstraint() const {
    return useTanConstraint;
  }

  bool isAssumeStatic() const {
    return assumeStatic;
  }

  const Frame & getReferenceFrame() const { return referenceFrame_; }
 protected:
  void writeConfig(std::ostream & out) const override;
 private:
  std::shared_ptr<BaseTrajectoryBatchState> state_;
  bool estimate = true;
  bool useTanConstraint;
  double tanConstraintVariance;
  bool initWithPosMeasurements;
  ModuleLink<PositionSensor> posSensor;
  bool assumeStatic;
  const Frame & referenceFrame_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* OOMACT_ASLAM_CALIBRATION_MODEL_POSITION_TRAJECTORY_H_ */
