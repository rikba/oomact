#ifndef OOMACT_ASLAM_CALIBRATION_MODEL_R3TRAJECTORY_H_
#define OOMACT_ASLAM_CALIBRATION_MODEL_R3TRAJECTORY_H_

#include <aslam/splines/OPTBSpline.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/NsecTimePolicy.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>

#include "../../SensorId.h"
#include "../../tools/Interval.h"

namespace aslam {
namespace backend {
class ErrorTermReceiver;
}
namespace calibration {
class CalibratorI;
class DesignVariableReceiver;
class R3TrajectoryCarrier;

typedef aslam::splines::OPTBSpline<typename bsplines::EuclideanBSpline<Eigen::Dynamic, 3, bsplines::NsecTimePolicy>::CONF>::BSpline TranslationSpline;

template <typename TranslationFactory>
struct ExpressionFactory {
  TranslationFactory trans;
};

template <int MaxDerivative, typename Spline>
auto getEF(Spline & s, BoundedTimeExpression t) -> decltype(s.template getExpressionFactoryAt<MaxDerivative>(t.timestampExpresion, t.lBound, t.uBound)) {
  return s.template getExpressionFactoryAt<MaxDerivative>(t.timestampExpresion, t.lBound, t.uBound);
}
template <int MaxDerivative, typename Spline>
auto getEF(Spline & s, Timestamp t) -> decltype(s.template getExpressionFactoryAt<MaxDerivative>(t)) {
  return s.template getExpressionFactoryAt<MaxDerivative>(t);
}

class R3Trajectory {
 public:
  R3Trajectory(const R3TrajectoryCarrier & carrier);

  TranslationSpline & getTranslationSpline() {
    return translationSpline;
  }

  const TranslationSpline & getTranslationSpline() const {
    return translationSpline;
  }

  void writeToFile(const CalibratorI & calib, const std::string & pathPrefix) const;
  void addToProblem(const bool stateActive, DesignVariableReceiver & designVariableReceiver);
  void addWhiteNoiseModelErrorTerms(backend::ErrorTermReceiver & errorTermReceiver, std::string name, const double invSigma) const;

  void fitSplines(const Interval& effectiveBatchInterval, const size_t numMeasurements, const std::vector<sm::timing::NsecTime> & timestamps, const std::vector<Eigen::Vector3d> & transPoses);

  void initSplinesConstant(const Interval& effectiveBatchInterval, const size_t numMeasurements, const Eigen::Vector3d & transPose = Eigen::Vector3d::Zero());

  const R3TrajectoryCarrier& getCarrier() const {
    return carrier;
  }

  template <int TranslationMaxDerivative, typename Time>
  auto getExpressionFactory(Time t) const -> ExpressionFactory<decltype(getEF<TranslationMaxDerivative>(getTranslationSpline(), t))> {
    return {getEF<TranslationMaxDerivative>(getTranslationSpline(), t)};
  }

 private:
  TranslationSpline translationSpline;
  const R3TrajectoryCarrier & carrier;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* OOMACT_ASLAM_CALIBRATION_MODEL_R3TRAJECTORY_H_ */
