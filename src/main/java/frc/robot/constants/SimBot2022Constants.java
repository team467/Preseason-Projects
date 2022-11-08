package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.controls.FeedbackConstant;
import frc.robot.constants.controls.GearRatio;
import frc.robot.constants.controls.SimpleFeedforwardConstant;
import lib.swerveodometry.SwerveDriveKinematics;

public class SimBot2022Constants implements Constants {
    @Override
    public RobotType robot() {
        return RobotType.ROBOT_SIMBOT;
    }

    @Override
    public String logFolder() {
        return null;
    }

    private Translation2d[] moduleTranslations() {
        return new Translation2d[]{
                new Translation2d(0.65 / 2, 0.65 / 2),
                new Translation2d(0.65 / 2, -0.65 / 2),
                new Translation2d(-0.65 / 2, 0.65 / 2),
                new Translation2d(-0.65 / 2, -0.65 / 2)
        };
    }

    @Override
    public double maxLinearSpeed() {
        return Units.feetToMeters(14.5);
    }

    @Override
    public double maxAngularSpeed() {
//        return maxLinearSpeed() / Arrays.stream(moduleTranslations())
//                .map(translation -> translation.getNorm()).max(Double::compare).get();
        return Units.feetToMeters(1);
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
        return new SimpleFeedforwardConstant(0.116970, 0.133240);
    }

    @Override
    public FeedbackConstant turnFB() {
        return new FeedbackConstant(23.0, 0.0);
    }

    @Override
    public SwerveDriveKinematics swerveKinematics() {
        return new SwerveDriveKinematics(moduleTranslations());
    }

    @Override
    public Rotation2d absoluteAngleOffset() {
        return new Rotation2d();
    }
}
