package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
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
            case ROBOT_KITBOT:
            case ROBOT_COMP:
                return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

            case ROBOT_SIM:
                return Mode.SIM;

            default:
                return Mode.REAL;

        }
    }

    enum RobotType {
        ROBOT_SIM, ROBOT_KITBOT, ROBOT_COMP
    }

    enum Mode {
        REAL, REPLAY, SIM
    }

    SimpleFeedforwardConstant driveFF();
    GearRatio driveGearRatio();
    DifferentialDriveKinematics driveKinematics();
    double driveWheelDiameter();
    double driveMaxLinearVelocity();
}
