

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Module extends SubsystemBase {

    CANSparkMax driveMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax turnMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);


    public Module() {
        super();

    }

    public void forward() {
        driveMotor.set(0.2);
    }
    
    public void backward() {
        driveMotor.set(-0.2);
    }

    public void disableDriveMotor() {
        driveMotor.stopMotor();
    }

    public void right() {
        turnMotor.set(0.2);
    }
    
    public void left() {
        turnMotor.set(-0.2);
    }

    public void disableTurnMotor() {
        turnMotor.stopMotor();
    }

    

}