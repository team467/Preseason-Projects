package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

import java.util.function.Supplier;

public class DriveWithJoystick extends CommandBase {
    private final Drive drive;
    private final Supplier<Double> leftYSupplier;
    private final Supplier<Double> rightXSupplier;

    private static final double DEADBAND = 0.1;

    public DriveWithJoystick(Drive drive, Supplier<Double> leftYSupplier, Supplier<Double> rightXSupplier) {
        this.drive = drive;
        this.leftYSupplier = leftYSupplier;
        this.rightXSupplier = rightXSupplier;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double leftY = MathUtil.applyDeadband(leftYSupplier.get(), DEADBAND);
        double rightX = MathUtil.applyDeadband(rightXSupplier.get(), DEADBAND);

        drive.arcadeDriveSpeed(leftY, rightX, true);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
