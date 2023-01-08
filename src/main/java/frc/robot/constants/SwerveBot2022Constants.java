package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.controls.FeedbackConstant;
import frc.robot.constants.controls.GearRatio;
import frc.robot.constants.controls.SimpleFeedforwardConstant;
import java.util.Arrays;

public class SwerveBot2022Constants implements Constants {
  @Override
  public RobotType robot() {
    return RobotType.ROBOT_SWERVE;
  }

  @Override
  public String logFolder() {
    return null;
  }

  @Override
  public double driveMaxCoastVelocity() {
    //    return 0.5;
    return 100;
  }

  private Translation2d[] moduleTranslations() {
    return new Translation2d[] {
      new Translation2d(Units.inchesToMeters(11.50), Units.inchesToMeters(11.50)),
      new Translation2d(Units.inchesToMeters(11.50), -Units.inchesToMeters(11.50)),
      new Translation2d(-Units.inchesToMeters(11.50), Units.inchesToMeters(11.50)),
      new Translation2d(-Units.inchesToMeters(11.50), -Units.inchesToMeters(11.50))
    };
  }

  @Override
  public double maxLinearSpeed() {
    return 1.0;
  }

  @Override
  public double maxAngularSpeed() {
    return maxLinearSpeed()
        / Arrays.stream(moduleTranslations())
            .map(Translation2d::getNorm)
            .max(Double::compare)
            .get();
  }

  @Override
  public double moduleWheelDiameter() {
    return Units.inchesToMeters(4);
  }

  @Override
  public GearRatio moduleDriveGearRatio() {
    return new GearRatio(6.75, 1); // SDS L2
  }

  @Override
  public GearRatio moduleTurnGearRatio() {
    return new GearRatio(12.8, 1);
  }

  @Override
  public SimpleFeedforwardConstant moduleDriveFF() {
    return new SimpleFeedforwardConstant(0.19359, 0.8193, 0.38297);
  }

  @Override
  public SimpleFeedforwardConstant moduleTurnFF() {
    return new SimpleFeedforwardConstant(0.16302, 0.0089689, 0.00034929);
    //    return new SimpleFeedforwardConstant(0, 0);
  }

  @Override
  public FeedbackConstant moduleTurnFB() {
    return new FeedbackConstant(3.2526, 0.05);
    //    return new FeedbackConstant(0.1, 0);
  }

  @Override
  public SwerveDriveKinematics kinematics() {
    return new SwerveDriveKinematics(moduleTranslations());
  }

  @Override
  public Rotation2d[] absoluteAngleOffset() {
    return new Rotation2d[] {
      Rotation2d.fromDegrees(-46.75781250000001),
      Rotation2d.fromDegrees(11.162109374999998),
      Rotation2d.fromDegrees(110.478515625),
      Rotation2d.fromDegrees(-100.19531250000001)
    };
  }

  @Override
  public double chassisDriveMaxVelocity() {
    return 0;
  }

  @Override
  public double chassisDriveMaxAcceleration() {
    return 0;
  }

  @Override
  public double chassisTurnMaxVelocity() {
    return 0;
  }

  @Override
  public double chassisTurnMaxAcceleration() {
    return 0;
  }

  @Override
  public FeedbackConstant chassisDriveFB() {
    return new FeedbackConstant(0.0);
  }

  @Override
  public FeedbackConstant chassisTurnFB() {
    return new FeedbackConstant(0.0);
  }
}
