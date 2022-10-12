package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

public class ModuleIOSparkMAXNoAbs implements ModuleIO {
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    private final CANSparkMax turnMotor;
    private final RelativeEncoder turnEncoder;

    public ModuleIOSparkMAXNoAbs(int driveMotorId, int turnMotorId) {
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        // Convert rotations to radians
        double rotsToRads = Units.rotationsToRadians(1) * RobotConstants.get().wheelGearRatio().getRotationsPerInput();
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
        inputs.turnPosition = turnEncoder.getPosition();
        inputs.turnPositionAbsolute = turnEncoder.getPosition();
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
