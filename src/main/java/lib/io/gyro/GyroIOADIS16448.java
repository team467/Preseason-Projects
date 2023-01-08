package lib.io.gyro;

import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class GyroIOADIS16448 implements GyroIO {
  private final ADIS16448_IMU gyro;

  public GyroIOADIS16448() {
    this.gyro = new ADIS16448_IMU();
    gyro.setYawAxis(ADIS16448_IMU.IMUAxis.kY);
    gyro.calibrate();
    gyro.reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.angle = gyro.getAngle();
    inputs.rate = gyro.getRate();
  }
}
