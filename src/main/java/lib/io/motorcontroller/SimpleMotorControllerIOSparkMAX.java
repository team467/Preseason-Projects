package lib.io.motorcontroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

public class SimpleMotorControllerIOSparkMAX implements SimpleMotorControllerIO {
  private final CANSparkMax motor;

  /**
   * Create a new SimpleMotorControllerIOSparkMAX
   *
   * @param motorId The ID of the motor
   * @param inverted Whether the motor is inverted
   * @param gearRatio The gear ratio in inputs/outputs
   */
  public SimpleMotorControllerIOSparkMAX(int motorId, boolean inverted, double gearRatio) {
    this.motor = new CANSparkMax(motorId, MotorType.kBrushless);
    this.motor.setInverted(inverted);
    this.motor.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(1) * gearRatio);
    this.motor
        .getEncoder()
        .setVelocityConversionFactor(Units.rotationsPerMinuteToRadiansPerSecond(1));
  }

  @Override
  public void updateInputs(SimpleMotorControllerIOInputs inputs) {
    inputs.position = motor.getEncoder().getPosition();
    inputs.velocity = motor.getEncoder().getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.current = new double[] {motor.getOutputCurrent()};
    inputs.temp = new double[] {motor.getMotorTemperature()};
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
