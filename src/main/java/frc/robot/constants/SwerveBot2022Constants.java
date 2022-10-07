package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.controls.FeedbackConstant;
import frc.robot.constants.controls.GearRatio;
import frc.robot.constants.controls.SimpleFeedforwardConstant;

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
    public double maxLinearSpeed() {
        return 14.0;
    }

    @Override
    public double wheelDiameter() {
        return Units.inchesToMeters(4);
    }

    @Override
    public GearRatio wheelGearRatio() {
        return new GearRatio(6.75, 1); // SDS L2
    }

    @Override
    public SimpleFeedforwardConstant driveFF() {
        return new SimpleFeedforwardConstant(0, 0, 0);
    }

    @Override
    public FeedbackConstant turnFB() {
        return new FeedbackConstant(1.0, 0.1);
    }

    @Override
    public SwerveDriveKinematics swerveKinematics() {
        return new SwerveDriveKinematics();
    }

    @Override
    public Rotation2d absoluteAngleOffset() {
        return new Rotation2d();
    }
}
