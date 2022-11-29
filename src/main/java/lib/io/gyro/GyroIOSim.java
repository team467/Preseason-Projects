package lib.io.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;

public class GyroIOSim implements GyroIO {
  private Supplier<Rotation2d> angle;

  public GyroIOSim(Supplier<Rotation2d> angle) {
    this.angle = angle;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.angle = angle.get().getDegrees();
    inputs.rate = -1;
  }
}
