package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class DriveConstants {

  public final int kLeftMotor1 = 12;
  public final int kRightMotor1 = 13;
  public final double kWheelRadius = Units.inchesToMeters(2);
  public final double kEncoderResolution = 2048;
  public final double kGearRatio = (double) 62 / 8;
}
