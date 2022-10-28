package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.RobotConstants;

public class DriveIOSim implements DriveIO {
    private final DifferentialDrivetrainSim sim;
    private double appliedVoltsLeft = 0.0;
    private double appliedVoltsRight = 0.0;
    private final double wheelRadius = RobotConstants.get().driveWheelDiameter() / 2;

    public DriveIOSim() {
        sim = DifferentialDrivetrainSim.createKitbotSim(
                DifferentialDrivetrainSim.KitbotMotor.kDoubleNEOPerSide,
                DifferentialDrivetrainSim.KitbotGearing.k10p71,
                DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
                null
        );
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        sim.update(0.02);
        inputs.leftPosition = sim.getLeftPositionMeters() / wheelRadius; // rad= m/radius
        inputs.leftVelocity = sim.getLeftVelocityMetersPerSecond() / wheelRadius;
        inputs.leftAppliedVolts = appliedVoltsLeft;
        inputs.leftCurrent = new double[]{sim.getCurrentDrawAmps()};
        inputs.leftTemp = new double[]{};

        inputs.rightPosition = sim.getRightPositionMeters() / wheelRadius; // rad= m/radius
        inputs.rightVelocity = sim.getRightVelocityMetersPerSecond() / wheelRadius;
        inputs.rightAppliedVolts = appliedVoltsRight;
        inputs.rightCurrent = new double[]{sim.getCurrentDrawAmps()};
        inputs.rightTemp = new double[]{};

        inputs.angle = sim.getHeading().getDegrees();
        inputs.rate = 0.0;
    }

    @Override
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        appliedVoltsLeft = leftVolts;
        appliedVoltsRight = rightVolts;
        sim.setInputs(leftVolts, rightVolts);
    }
}
