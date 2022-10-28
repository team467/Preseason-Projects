package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

import java.util.function.Supplier;

public class DriveWithTank extends CommandBase {
    private final Drive drive;
    private final Supplier<Double> leftYSupplier;
    private final Supplier<Double> rightYSupplier;

    private static final double DEADBAND = 0.1;

    public DriveWithTank(Drive drive, Supplier<Double> leftYSupplier, Supplier<Double> rightYSupplier) {
        this.drive = drive;
        this.leftYSupplier = leftYSupplier;
        this.rightYSupplier = rightYSupplier;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double leftY = MathUtil.applyDeadband(leftYSupplier.get(), DEADBAND);
        double rightY = MathUtil.applyDeadband(rightYSupplier.get(), DEADBAND);

        drive.tankDriveSpeed(leftY, rightY, true);
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
