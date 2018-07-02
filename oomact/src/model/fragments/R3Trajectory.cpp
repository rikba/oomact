#include <aslam/calibration/model/fragments/R3Trajectory.h>

#include <aslam/backend/QuadraticIntegralError.hpp>
#include <aslam/backend/VectorExpression.hpp>
#include <bsplines/BSplineFitter.hpp>
#include <glog/logging.h>

#include "aslam/calibration/calibrator/CalibratorI.h"
#include <aslam/calibration/model/fragments/R3TrajectoryCarrier.h>
#include <aslam/calibration/tools/SplineWriter.h>
#include <aslam/calibration/DesignVariableReceiver.h>
#include <aslam/calibration/tools/ErrorTermStatistics.h>

using aslam::backend::VectorExpression;
using aslam::backend::ErrorTermReceiver;

namespace aslam {
namespace calibration {

R3Trajectory::R3Trajectory(const R3TrajectoryCarrier & carrier) :
  translationSpline(carrier.getTransSplineOrder()),
  carrier(carrier)
{
	static_assert(std::is_same<sm::timing::NsecTime, Timestamp::Integer>::value, "");
	static_assert(1e9 == Timestamp::getDivider(), "");
}


template<typename BSpline_, typename Functor>
struct WhiteNoiseIntegrationErrorExpressionFactory {
  typedef BSpline_ BSplineT;
  typedef typename BSplineT::time_t SplineTime;
  enum {PointSize = 3 };
  typedef VectorExpression<PointSize> value_expression_t;
  typedef Eigen::Matrix<double, 1, 1>  value_t;

  const BSplineT & _bspline;
  const Eigen::Matrix<double, PointSize, PointSize> _sqrtInvR;
  const Functor _funct;

  WhiteNoiseIntegrationErrorExpressionFactory(const BSplineT & bspline, Functor funct, Eigen::Matrix<double, PointSize, PointSize> sqrtInvR) : _bspline(bspline), _sqrtInvR(sqrtInvR), _funct(funct) {}

  inline value_expression_t getExpressionAt(const BSplineT & bspline, SplineTime time) const {
    return _funct(bspline, time);
  }
  inline value_expression_t operator()(SplineTime time) const {
    return getExpressionAt(_bspline, time);
  }

  inline value_t eval(const BSplineT & spline, SplineTime t) const {
    auto error = (_sqrtInvR * getExpressionAt(spline, t).evaluate()).eval();
    return (value_t() << error.dot(error)).finished();
  }
  inline value_t getZeroValue(const BSplineT & /*spline*/) const {
    return value_t::Zero();
  }

  value_t calcIntegral() {
    return _bspline.template evalFunctorIntegral<value_t>(_bspline.getMinTime(), _bspline.getMaxTime(), *this);
  }
};

template <typename SplineT, typename Functor>
void addWhiteNoiseModelErrorTerms(ErrorTermReceiver & errorTermReceiver, const SplineT & spline, Functor f, std::string name, const Eigen::MatrixXd & sqrtInvR, int numberOfIntegrationPoints = -1){
  WhiteNoiseIntegrationErrorExpressionFactory<SplineT, Functor> integrationFunctor(spline, f, sqrtInvR);
  if(numberOfIntegrationPoints < 0){
    numberOfIntegrationPoints = (spline.getAbsoluteNumberOfSegments() + spline.getSplineOrder()) * 2;
  }
  LOG(INFO) << "Adding " << numberOfIntegrationPoints << " " << name << " error terms";
  aslam::backend::integration::addQuadraticIntegralExpressionErrorTerms<aslam::backend::integration::DefaultAlgorithm>(errorTermReceiver, spline.getMinTime(), spline.getMaxTime(), numberOfIntegrationPoints, integrationFunctor, sqrtInvR);
  LOG(INFO) << "Total initial cost " << name << ": " << integrationFunctor.calcIntegral();
}

void R3Trajectory::addWhiteNoiseModelErrorTerms(ErrorTermReceiver & errorTermReceiver, std::string name, const double invSigma) const {
  calibration::addWhiteNoiseModelErrorTerms(errorTermReceiver, getTranslationSpline(), [&](const TranslationSpline & bspline, typename TranslationSpline::time_t time){ return bspline.getExpressionFactoryAt<2>(time).getValueExpression(2);}, name + "WhiteNoiseAcceleration", Eigen::Matrix3d::Identity() * invSigma);
}


void R3Trajectory::addToProblem(const bool stateActive, DesignVariableReceiver & problem) {
  problem.addSplineDesignVariables(translationSpline, stateActive);
}


void R3Trajectory::writeToFile(const CalibratorI& calib, const std::string& pathPrefix) const {
  writeSpline(translationSpline, calib.getOptions().getSplineOutputSamplePeriod(), pathPrefix + "trans");
}

void R3Trajectory::fitSplines(const Interval& effectiveBatchInterval, const size_t numMeasurements, const std::vector<sm::timing::NsecTime> & timestamps, const std::vector<Eigen::Vector3d> & transPoses) {
  const double elapsedTime = effectiveBatchInterval.getElapsedTime();
  const int measPerSec = std::round(numMeasurements / elapsedTime);
  int numSegments;
  double splineKnotsPerSecond = getCarrier().getKnotsPerSecond();
  if (measPerSec > splineKnotsPerSecond)
    numSegments = std::ceil(splineKnotsPerSecond * elapsedTime);
  else
    numSegments = numMeasurements;

  const double transSplineLambda = getCarrier().getTransFittingLambda() * elapsedTime;
  LOG(INFO)<< "Using for the " << getCarrier().getName() << " splines numSegments=" << numSegments << ", because the batch is " << elapsedTime << "s long and splineKnotsPerSecond=" << splineKnotsPerSecond << ", transFittingLambda=" << getCarrier().getTransFittingLambda();

  bsplines::BSplineFitter<TranslationSpline>::initUniformSpline(getTranslationSpline(), effectiveBatchInterval.start, effectiveBatchInterval.end, timestamps, transPoses, numSegments, transSplineLambda);

  if(VLOG_IS_ON(1)){
    LOG(INFO) << "Computing initial offset statistics.";
    ErrorTermStatistics statTrans(getCarrier().getName() + "_trans[1]");
    for(size_t i = 0; i < timestamps.size(); i++){
      statTrans.add(pow((getTranslationSpline().getEvaluatorAt<0>(timestamps[i]).eval() - transPoses[i]).norm(), 2));
    }
    statTrans.printInto(LOG(INFO));
  }
}

void R3Trajectory::initSplinesConstant(const Interval& effectiveBatchInterval, const size_t numMeasurements, const Eigen::Vector3d & transPose) {
  const double elapsedTime = effectiveBatchInterval.getElapsedTime();
  const int measPerSec = std::round(numMeasurements / elapsedTime);
  int numSegments;
  double splineKnotsPerSecond = getCarrier().getKnotsPerSecond();
  if (measPerSec > splineKnotsPerSecond)
    numSegments = std::ceil(splineKnotsPerSecond * elapsedTime);
  else
    numSegments = numMeasurements;

  LOG(INFO)<< "Using for the " << getCarrier().getName() << " splines numSegments=" << numSegments << ", because the batch is " << elapsedTime << "s long and splineKnotsPerSecond=" << splineKnotsPerSecond;

  getTranslationSpline().initConstantUniformSpline(effectiveBatchInterval.start, effectiveBatchInterval.end, numSegments, transPose);
}


} /* namespace calibration */
} /* namespace aslam */
