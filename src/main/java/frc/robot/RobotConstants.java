package frc.robot;

import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotType;
import frc.robot.constants.KitBot2022Constants;

public class RobotConstants {
    /**
     * TODO: Manually change this for simulation and Replay.
     */
    private static final RobotType robot = RobotType.ROBOT_KITBOT;
    private static Constants constants = new KitBot2022Constants();

    private RobotConstants() {
        throw new IllegalStateException("Utility class");
    }

    public static Constants get() {
        if (constants == null) {
            System.err.println("No constants file set");
        }

        return constants;
    }

    public static void set(Constants robot) {
        constants = robot;
    }
}
