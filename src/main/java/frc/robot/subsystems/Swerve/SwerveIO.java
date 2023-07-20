package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
    @AutoLog
    public static class SwerveIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveOutputVoltageVolts = 0.0;
        public double[] driveCurrentAmps = new double[] {};
        public double[] driveTempCelcius = new double[] {};
    
        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] {};
        public double[] turnTempCelcius = new double[] {};

        public boolean connected = false;
        public double rollPositionRad = 0.0;
        public double pitchPositionRad = 0.0;
        public double yawPositionRad = 0.0;
        public double rollVelocityRadPerSec = 0.0;
        public double pitchVelocityRadPerSec = 0.0;
        public double yawVelocityRadPerSec = 0.0;
      }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}
}
