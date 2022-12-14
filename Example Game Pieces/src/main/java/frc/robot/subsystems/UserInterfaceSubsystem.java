package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UserInterfaceSubsystem extends SubsystemBase {

    public final static int XBOX_PORT = 0;
    
    private final XboxController xbox = new XboxController(XBOX_PORT);
    private final XboxControllerSim xboxSim = new XboxControllerSim(xbox);

    private boolean elevatorUp = false;
    public boolean isElevatorUp() {
        return elevatorUp && !elevatorDown;
    }

    void setElevatorUp(boolean elevatorUp) {
        this.elevatorUp = elevatorUp;
    }

    public boolean isElevatorDown() {
        return elevatorDown;
    }

    void setElevatorDown(boolean elevatorDown) {
        this.elevatorDown = elevatorDown;
    }

    private boolean elevatorDown = false;

    @Override
    public void periodic() {
      // This method will be called once per scheduler run

        // This maps controller buttons to status variable
        elevatorUp = xbox.getAButton();
        elevatorDown = xbox.getBButton();

    }

    /**
     * Updates simulation data such as faking voltage or distance.
     */
    @Override
    public void simulationPeriodic() {

        // This maps controller buttons to status variable


    }

    /**
     * Sets the telemetry that is posted to the Smart Dashboard on associated log. 
     * 
     * @param builder the manager for automated Network Table telemetry
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("User Interface");

        // Put the current state on the Smart Dashboard
        builder.addBooleanProperty(".elevatorUp", this::isElevatorUp, this::setElevatorUp);
        builder.addBooleanProperty(".elevatorDown", this::isElevatorDown, this::setElevatorDown);

    }
  
}
