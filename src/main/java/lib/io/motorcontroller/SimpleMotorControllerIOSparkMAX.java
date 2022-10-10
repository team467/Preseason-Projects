package lib.io.motorcontroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class SimpleMotorControllerIOSparkMAX implements SimpleMotorControllerIO {
    private final CANSparkMax motor;
    private final boolean voltageMode;

    /**
     * Create a new SimpleMotorControllerIOSparkMAX
     * <p> This will use rotations as the unit for position and velocity, avoid using this if possible
     *
     * @param motorId  The ID of the motor
     * @param inverted Whether the motor is inverted
     */
    public SimpleMotorControllerIOSparkMAX(int motorId, boolean inverted) {
        this(motorId, inverted, -1);
    }

    /**
     * Create a new SimpleMotorControllerIOSparkMAX
     *
     * @param motorId    The ID of the motor
     * @param inverted   Whether the motor is inverted
     * @param rotsToRads The conversion factor from rotations to radians
     *                   This can be done by either using the gear ratio and/or the wheel diameter
     *                   for example:
     *                   rotsToRads = 2 * Math.PI * gearRatioRotationsPerInput
     *                   OR
     *                   rotsToRads = Math.PI * wheelDiameter
     */
    public SimpleMotorControllerIOSparkMAX(int motorId, boolean inverted, double rotsToRads) {
        this.motor = new CANSparkMax(motorId, MotorType.kBrushless);
        this.motor.setInverted(inverted);
        if (rotsToRads >= 0) {
            this.motor.getEncoder().setPositionConversionFactor(rotsToRads);
            this.motor.getEncoder().setVelocityConversionFactor(rotsToRads / 60);
            voltageMode = true;
        } else {
            voltageMode = false;
        }
    }


    @Override
    public void updateInputs(SimpleMotorControllerIOInputs inputs) {
        inputs.position = motor.getEncoder().getPosition();
        inputs.velocity = motor.getEncoder().getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.current = new double[]{motor.getOutputCurrent()};
        inputs.temp = new double[]{motor.getMotorTemperature()};
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setVoltage(double volts) {
        if (voltageMode) {
            motor.setVoltage(volts);
        } else {
            DriverStation.reportWarning("Voltage mode not enabled in " + this, false);
            motor.set(volts / RobotController.getBatteryVoltage());
        }
    }
}
