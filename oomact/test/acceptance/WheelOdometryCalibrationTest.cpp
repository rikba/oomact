#include <memory>

#include <gtest/gtest.h>

#include <aslam/calibration/calibrator/CalibratorI.h>
#include <aslam/calibration/data/AccelerometerMeasurement.h>
#include <aslam/calibration/data/GyroscopeMeasurement.h>
#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/model/PoseTrajectory.h>
#include <aslam/calibration/model/sensors/WheelOdometry.h>
#include <aslam/calibration/model/sensors/PoseSensor.h>
#include <aslam/calibration/test/MockMotionCaptureSource.h>
#include <aslam/calibration/tools/SmartPointerTools.h>
#include <sm/kinematics/Transformation.hpp>


using namespace aslam::calibration;
using namespace aslam::calibration::test;

TEST(CalibrationTestSuite, testWheelOdometryCalibrationStaight) {
  auto vs = ValueStoreRef::fromFile("acceptance/wheelOdometry-pose.info");

  FrameGraphModel m(vs.getChild("model"));
  PoseSensor psA(m, "pose");
  WheelOdometry wheelOdometry(m, "wheelOdometry");
  PoseTrajectory traj(m, "traj");
  m.addModulesAndInit(psA, wheelOdometry, traj);

//  wheelOdometry.getTranslationVariable().set({0., 1., 0.});
//  const double rotUpdate[] = {0., 0.1, 0.};
//  wheelOdometry.getRotationVariable().update(rotUpdate, 3);

  auto spModel = aslam::to_local_shared_ptr(m);
  auto c = createBatchCalibrator(vs.getChild("calibrator"), spModel);

  constexpr double l = 0.2; // base length
  constexpr double d = 0.1; // wheel diameters

  for (auto& p : MmcsRotatingStraightLine.getPoses(4.0 * M_PI)) {
    psA.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
    c->addMeasurementTimestamp(p.time, psA);

    // 1 rad/s rotation (right circle positively, left  (x < -1) circle negatively)
    const double omega = p.p[0] < -1.0 ? -1 : 1;
    const double transVel = 1.0;
    const double omegaL = omega * l;
    const double rW = (transVel + omegaL) / d;
    const double lW = (transVel - omegaL) / d;
    wheelOdometry.addMeasurement(*c, p.time, { lW, rW });
    c->addMeasurementTimestamp(p.time, wheelOdometry);
  }

//  EXPECT_NEAR(1, wheelOdometry.getTranslationToParent()[1], 0.0001);
//  EXPECT_NEAR(0.05, wheelOdometry.getRotationQuaternionToParent()[1], 0.01);
  c->calibrate();
  EXPECT_NEAR(0, wheelOdometry.getTranslationToParent()[1], 0.0001);
  EXPECT_NEAR(0, wheelOdometry.getRotationQuaternionToParent()[1], 0.01);
}

TEST(CalibrationTestSuite, testWheelOdometryCalibrationEight) {
  auto vs = ValueStoreRef::fromFile("acceptance/wheelOdometry-pose.info");

  FrameGraphModel m(vs.getChild("model"));
  PoseSensor psA(m, "pose");
  WheelOdometry wheelOdometry(m, "wheelOdometry");
  PoseTrajectory traj(m, "traj");
  m.addModulesAndInit(psA, wheelOdometry, traj);

//  wheelOdometry.getTranslationVariable().set({0., 1., 0.});
//  const double rotUpdate[] = {0., 0.1, 0.};
//  wheelOdometry.getRotationVariable().update(rotUpdate, 3);

  auto spModel = aslam::to_local_shared_ptr(m);
  auto c = createBatchCalibrator(vs.getChild("calibrator"), spModel);

  constexpr double l = 0.2; // base length
  constexpr double d = 0.13; // wheel diameters

  for (auto& p : MmcsEight.getPoses(4.0 * M_PI)) {
    psA.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
    c->addMeasurementTimestamp(p.time, psA);

    // 1 rad/s rotation (right circle positively, left  (x < -1) circle negatively)
    const double omega = p.p[0] < -1.0 ? -1 : 1;
    const double transVel = 1.0;
    const double omegaL = omega * l;
    const double lW = (transVel - omegaL) / d;
    const double rW = (transVel + omegaL) / d;
    wheelOdometry.addMeasurement(*c, p.time, { lW, rW });
    c->addMeasurementTimestamp(p.time, wheelOdometry);
  }

//  EXPECT_NEAR(1, wheelOdometry.getTranslationToParent()[1], 0.0001);
//  EXPECT_NEAR(0.05, wheelOdometry.getRotationQuaternionToParent()[1], 0.01);
  c->calibrate();
  EXPECT_NEAR(0, wheelOdometry.getTranslationToParent()[1], 0.0001);
  EXPECT_NEAR(0, wheelOdometry.getRotationQuaternionToParent()[1], 0.01);
}
