package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.controls.FeedbackConstant;
import frc.robot.constants.controls.GearRatio;
import frc.robot.constants.controls.SimpleFeedforwardConstant;

public interface Constants {

  /**
   * @return Robot type/name
   */
  RobotType robot();

  /**
   * @return Folder to put logs into (nullable)
   */
  String logFolder();

  /**
   * @return Check if robot is real, sim, or replay
   */
  default Mode mode() {
    switch (robot()) {
      case ROBOT_SWERVE:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  enum RobotType {
    ROBOT_SWERVE,
    ROBOT_SIMBOT
  }

  enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  /**
   * @return Max speed in m/s
   */
  double maxLinearSpeed();

  double maxAngularSpeed();

  /**
   * @return Wheel diameter in m
   */
  double moduleWheelDiameter();

  /**
   * @return Wheel gear ratio
   */
  GearRatio moduleDriveGearRatio();

  GearRatio moduleTurnGearRatio();

  /**
   * @return Drive feedforward
   */
  SimpleFeedforwardConstant moduleDriveFF();

  SimpleFeedforwardConstant moduleTurnFF();

  /**
   * @return Turn feedback
   */
  FeedbackConstant moduleTurnFB();

  /**
   * @return Swerve kinematics
   */
  SwerveDriveKinematics kinematics();

  /**
   * @return Absolute angle offset
   */
  Rotation2d[] absoluteAngleOffset();

  double chassisDriveMaxVelocity();

  double chassisDriveMaxAcceleration();

  double chassisTurnMaxVelocity();

  double chassisTurnMaxAcceleration();

  FeedbackConstant chassisDriveFB();

  FeedbackConstant chassisTurnFB();
}
