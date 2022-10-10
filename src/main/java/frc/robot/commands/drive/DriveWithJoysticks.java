package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class DriveWithJoysticks extends CommandBase {
    private final Drive drive;
    private final Supplier<Double> leftXSupplier;
    private final Supplier<Double> leftYSupplier;
    private final Supplier<Double> rightXSupplier;
    private final Supplier<Boolean> robotRelativeOverride;

    private static final double DEADBAND = 0.1;

    public DriveWithJoysticks(Drive drive, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier, Supplier<Double> rightXSupplier, Supplier<Boolean> robotRelativeOverride) {
        this.drive = drive;
        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;
        this.rightXSupplier = rightXSupplier;
        this.robotRelativeOverride = robotRelativeOverride;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        Logger.getInstance().recordOutput("ActiveCommands/DriveWithJoysticks", true);
        double leftX = MathUtil.applyDeadband(leftXSupplier.get(), DEADBAND);
        double leftY = MathUtil.applyDeadband(leftYSupplier.get(), DEADBAND);
        double rightX = MathUtil.applyDeadband(rightXSupplier.get(), DEADBAND);

        double leftXVelocity = leftX * RobotConstants.get().maxLinearSpeed();
        double leftYVelocity = leftY * RobotConstants.get().maxLinearSpeed();
        double rightYVelocity = rightX * RobotConstants.get().maxAngularSpeed();

        drive.speedDrive(leftXVelocity, leftYVelocity, -rightYVelocity, robotRelativeOverride.get());
    }
}
