package lib.io.motorcontroller;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class SimpleMotorControllerIOTalonSRX implements SimpleMotorControllerIO {
    private final WPI_TalonSRX motor;
    private double rotsToRads = 1.0;
    private final boolean voltageMode;

    public SimpleMotorControllerIOTalonSRX(int motorId, boolean inverted) {
        this(motorId, inverted, -1);
    }

    /**
     * Create a new SimpleMotorControllerIOTalonSRX
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
    public SimpleMotorControllerIOTalonSRX(int motorId, boolean inverted, double rotsToRads) {
        motor = new WPI_TalonSRX(motorId);
        motor.setInverted(inverted);
        if (rotsToRads >= 0) {
            this.rotsToRads = rotsToRads;
            voltageMode = true;
        } else {
            voltageMode = false;
        }
    }


    @Override
    public void updateInputs(SimpleMotorControllerIOInputs inputs) {
        inputs.position = motor.getSelectedSensorPosition() * rotsToRads;
        inputs.velocity = (motor.getSelectedSensorVelocity() / 60) * rotsToRads;
        inputs.appliedVolts = motor.getMotorOutputVoltage();
        inputs.current = new double[]{motor.getStatorCurrent()};
        inputs.temp = new double[]{motor.getTemperature()};
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
