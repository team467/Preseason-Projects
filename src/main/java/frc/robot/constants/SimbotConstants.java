package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.controls.GearRatio;
import frc.robot.constants.controls.SimpleFeedforwardConstant;

public class SimbotConstants implements Constants {
    @Override
    public RobotType robot() {
        return RobotType.ROBOT_SIM;
    }

    @Override
    public String logFolder() {
        return null;
    }

    @Override
    public SimpleFeedforwardConstant driveFF() {
        return new SimpleFeedforwardConstant(1,1,1);
    }

    @Override
    public GearRatio driveGearRatio() {
        return new GearRatio(10.71, 1);
    }

    @Override
    public DifferentialDriveKinematics driveKinematics() {
        return new DifferentialDriveKinematics(0.656);
    }

    @Override
    public double driveWheelDiameter() {
        return Units.inchesToMeters(6);
    }

    @Override
    public double driveMaxLinearVelocity() {
        return 10;
    }
}
