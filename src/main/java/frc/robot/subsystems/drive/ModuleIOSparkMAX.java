package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

public class ModuleIOSparkMAX implements ModuleIO {
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    private final CANSparkMax turnMotor;
    private final RelativeEncoder turnEncoder;

    private final WPI_CANCoder turnEncoderAbsolute;

    private int resetCount = 0;

    public ModuleIOSparkMAX(int driveMotorId, int turnMotorId, int turnAbsEncoderId) {
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();
        turnEncoderAbsolute = new WPI_CANCoder(turnAbsEncoderId);

        // Convert rotations to radians
        double rotsToRads =
            Units.rotationsToRadians(1) * RobotConstants.get().moduleDriveGearRatio()
                .getRotationsPerInput();
        driveEncoder.setPositionConversionFactor(rotsToRads);
        turnEncoder.setPositionConversionFactor(rotsToRads);

        // Convert rotations per minute to radians per second
        driveEncoder.setVelocityConversionFactor(rotsToRads / 60);
        turnEncoder.setVelocityConversionFactor(rotsToRads / 60);

        // Invert motors
        driveMotor.setInverted(false); //TODO: check if inverted
        turnMotor.setInverted(false); //TODO: check if inverted


        driveMotor.enableVoltageCompensation(12);
        turnMotor.enableVoltageCompensation(12);

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveVelocity = driveEncoder.getVelocity();
        inputs.drivePosition = driveEncoder.getPosition();
        inputs.turnVelocity = turnEncoder.getVelocity();

        // Reset the turn encoder sometimes when not moving
        if (turnEncoder.getVelocity() < Units.degreesToRadians(0.5)) {
            if (++resetCount >= 500) {
                resetCount = 0;
                turnEncoder.setPosition(Units.degreesToRadians(turnEncoderAbsolute.getAbsolutePosition()));
            }
        } else {
            resetCount = 0;
        }
        inputs.turnPosition = turnEncoder.getPosition();

        inputs.turnPositionAbsolute = Units.degreesToRadians(turnEncoderAbsolute.getAbsolutePosition());
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnMotor.setVoltage(volts);
    }
}
