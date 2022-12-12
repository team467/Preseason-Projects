// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.LogManager;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  // Environment Variables to move to Robot Constants
  public static final int ELEVATOR_MOTOR_CHANNEL = 1;
  public static final int ELEVATOR_ENCODER_CHANNEL_1 = 3;
  public static final int ELEVATOR_ENCODER_CHANNEL_2 = 4;
  public static final boolean ELEVATOR_REVERSE_ENCODER = false;
  public static final double ELEVATOR_TOP_DISTANCE_LIMIT = 89;
  public static final double ELEVATOR_BOTTOM_DISTANCE_LIMIT = 0;
  public static final double ELEVATOR_DISTANCE_PER_ROTATION = 10;
  public static final double ELEVATOR_POSITION_TOLERANCE = 0.5;
  public static final double ELEVATOR_kP = 1.0;
  public static final double ELEVATOR_kI = 0.0;
  public static final double ELEVATOR_kD = 100;

  public enum State {
    STOPPED,
    MOVING_UP,
    MOVING_DOWN,
    MOVE_TO_POSITON;
  }

  private State state;
  private double position;
  private double target;

  private MotorController motor;
  private Encoder encoder;
  private EncoderSim encoderSim;
  private PIDController pid;

    /* Set up Simulation Components */

  private static final double kElevatorGearing = 10.0;
  private static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  private static final double kCarriageMass = 4.0; // kg

  private static final double kMinElevatorHeight = Units.inchesToMeters(2);
  private static final double kMaxElevatorHeight = Units.inchesToMeters(50);

  private final DCMotor elevatorGearbox = DCMotor.getVex775Pro(4);
  

  private final ElevatorSim elevatorSim =
    new ElevatorSim(
      elevatorGearbox,
      kElevatorGearing,
      kCarriageMass,
      kElevatorDrumRadius,
      kMinElevatorHeight,
      kMaxElevatorHeight,
      VecBuilder.fill(0.01));

  private final Mechanism2d elevatorMechanism = new Mechanism2d(20, 50);
  private final MechanismRoot2d elevatorMechanismRoot = elevatorMechanism.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d elevatorMech2d =
    elevatorMechanismRoot.append(
          new MechanismLigament2d(
              "Elevator", Units.metersToInches(elevatorSim.getPositionMeters()), 90));

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
    motor = new PWMSparkMax(ELEVATOR_MOTOR_CHANNEL);
    encoder = new Encoder(ELEVATOR_ENCODER_CHANNEL_1, ELEVATOR_ENCODER_CHANNEL_2, ELEVATOR_REVERSE_ENCODER, EncodingType.k4X);
    encoderSim = new EncoderSim(encoder);
    pid = new PIDController(ELEVATOR_kP, ELEVATOR_kI, ELEVATOR_kD);
    pid.setTolerance(ELEVATOR_POSITION_TOLERANCE);

    state = State.STOPPED;
    position = 0;
    encoder.reset();
    encoder.setDistancePerPulse(ELEVATOR_DISTANCE_PER_ROTATION);

    // Publish Mechanism2d to SmartDashboard
    SmartDashboard.putData("Elevator Sim", elevatorMechanism);

  }

  public State state() {
    return state;
  }

  private void stateMachine() {
    position = encoder.getDistance();
    switch (state) {

      case MOVING_UP:
        if (position >= ELEVATOR_TOP_DISTANCE_LIMIT) {
          state = State.STOPPED;
          motor.set(0);
        } else {
          motor.set(0.7);
        }
        break;

      case MOVING_DOWN:
        if (position <= ELEVATOR_BOTTOM_DISTANCE_LIMIT) {
          state = State.STOPPED;
          motor.set(0);
        } else {
          motor.set(-1 * 0.7);
        }
        break;

      case MOVE_TO_POSITON:
        if (position <= ELEVATOR_BOTTOM_DISTANCE_LIMIT) {
        } else {
          motor.set(MathUtil.clamp(pid.calculate(encoder.getDistance(), target), -1 * 0.7, 0.7));
        }
        break;

      case STOPPED:
        break;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    stateMachine();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevatorSim.setInput(motor.get() * RobotController.getBatteryVoltage());
    elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    encoderSim.setDistance(elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    elevatorMech2d.setLength(Units.metersToInches(elevatorSim.getPositionMeters()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    // Put the current state on the Smart Dashboard
    builder.addStringProperty(".state",
        () -> state() != null ? state().toString(): "none", null);
   
  }
  
}
