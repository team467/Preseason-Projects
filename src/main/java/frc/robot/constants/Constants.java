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
        ROBOT_SWERVE, ROBOT_SIMBOT
    }

    enum Mode {
        REAL, REPLAY, SIM
    }

    /**
     * @return Max speed in m/s
     */
    double maxLinearSpeed();

    /**
     * @return Wheel diameter in m
     */
    double wheelDiameter();

    /**
     * @return Wheel gear ratio
     */
    GearRatio wheelGearRatio();

    /**
     * @return Drive feedforward
     */
    SimpleFeedforwardConstant driveFF();

    /**
     * @return Turn feedback
     */
    FeedbackConstant turnFB();

    /**
     * @return Swerve kinematics
     */
    SwerveDriveKinematics swerveKinematics();

    /**
     * @return Absolute angle offset
     */
    Rotation2d absoluteAngleOffset();
}
