// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  // Environment Variables to move to Robot Constants
  public static final int ELEVATOR_MOTOR_CHANNEL = 1;
  public static final int ELEVATOR_ENCODER_CHANNEL = 2;
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
  private AnalogEncoder encoder;
  private EncoderSim encoderSim;
  private PIDController pid;



  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
    motor = new PWMSparkMax(ELEVATOR_MOTOR_CHANNEL);
    encoder = new AnalogEncoder(ELEVATOR_ENCODER_CHANNEL);
    encoderSim = EncoderSim.createForChannel(ELEVATOR_ENCODER_CHANNEL);
    pid = new PIDController(ELEVATOR_kP, ELEVATOR_kI, ELEVATOR_kD);
    pid.setTolerance(ELEVATOR_POSITION_TOLERANCE);

    state = State.STOPPED;
    position = 0;
    encoder.reset();
    encoder.setDistancePerRotation(ELEVATOR_DISTANCE_PER_ROTATION);

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
    stateMachine();
  }
}
