#ifndef OOMACT_ASLAM_CALIBRATION_MODEL_R3TRAJECTORYCARRIER_H_
#define OOMACT_ASLAM_CALIBRATION_MODEL_R3TRAJECTORYCARRIER_H_
#include <aslam/calibration/model/Module.h>


namespace aslam {
namespace calibration {

class Frame;

class R3TrajectoryCarrier : public virtual Named {
 public:
  R3TrajectoryCarrier(sm::value_store::ValueStoreRef config, const Frame & frame);

  double getKnotsPerSecond() const {
    return knotsPerSecond;
  }

  void setKnotsPerSecond(double knotsPerSecond) {
    this->knotsPerSecond = knotsPerSecond;
  }

  int getTransSplineOrder() const {
    return transSplineOrder;
  }

  double getTransFittingLambda() const {
    return transFittingLambda;
  }

  const Frame & getFrame() const { return frame; }
 protected:
  void writeConfig(std::ostream & out) const;

  const Frame & frame;
  double knotsPerSecond;
  int transSplineOrder;
  double transFittingLambda;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* OOMACT_ASLAM_CALIBRATION_MODEL_R3TRAJECTORYCARRIER_H_ */
