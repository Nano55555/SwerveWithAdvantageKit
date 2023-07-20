package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREModuleState;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class SwerveModuleFalcon500 implements SwerveIO{
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    public TalonFX mAngleMotor;
    public TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    
    public static WPI_Pigeon2 gyro;

    private final double[] yprDegrees = new double[3];
    private final double[] xyzDps = new double[3];
    
    private static CTREConfigs ctreConfigs;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SWERVE.DRIVE_KS, Constants.SWERVE.DRIVE_KV, Constants.SWERVE.DRIVE_KA);

    public SwerveModuleFalcon500(int moduleNumber){
        switch(moduleNumber){
            case 0:    
                /* Angle Encoder Config */
                angleEncoder = new CANCoder(10);
                configAngleEncoder();

                /* Angle Motor Config */
                mAngleMotor = new TalonFX(12);
                configAngleMotor();

                /* Drive Motor Config */
                mDriveMotor = new TalonFX(11);
                configDriveMotor();
                this.angleOffset = Rotation2d.fromDegrees(0.0);
            case 1:
                /* Angle Encoder Config */
                angleEncoder = new CANCoder(20);
                configAngleEncoder();

                /* Angle Motor Config */
                mAngleMotor = new TalonFX(22);
                configAngleMotor();

                /* Drive Motor Config */
                mDriveMotor = new TalonFX(21);
                configDriveMotor();
                this.angleOffset = Rotation2d.fromDegrees(0.0);
            case 2:
                /* Angle Encoder Config */
                angleEncoder = new CANCoder(30);
                configAngleEncoder();

                /* Angle Motor Config */
                mAngleMotor = new TalonFX(32);
                configAngleMotor();

                /* Drive Motor Config */
                mDriveMotor = new TalonFX(31);
                configDriveMotor();
                this.angleOffset = Rotation2d.fromDegrees(0.0);
            case 3:
                /* Angle Encoder Config */
                angleEncoder = new CANCoder(40);
                configAngleEncoder();

                /* Angle Motor Config */
                mAngleMotor = new TalonFX(42);
                configAngleMotor();

                /* Drive Motor Config */
                mDriveMotor = new TalonFX(41);
                configDriveMotor();
                this.angleOffset = Rotation2d.fromDegrees(0.0);                         
                }

        gyro = new WPI_Pigeon2(Constants.CAN_IDS.PIDGEON);
        gyro.configFactoryDefault();

        this.moduleNumber = moduleNumber;
  

        ctreConfigs = new CTREConfigs();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SWERVE.MAX_SPEED;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SWERVE.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.SWERVE.ANGLE_GEAR_RATIO));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.SWERVE.ANGLE_GEAR_RATIO));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.SWERVE.ANGLE_GEAR_RATIO);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.SWERVE.ANGLE_MOTOR_INVERT);
        mAngleMotor.setNeutralMode(Constants.SWERVE.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.SWERVE.DRIVE_MOTOR_INVERT);
        mDriveMotor.setNeutralMode(Constants.SWERVE.DRIVE_NEUTRAL_MODE);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.drivePositionMeters = Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO);
        inputs.driveVelocityRadPerSec = Conversions.falconToMeters(mDriveMotor.getSelectedSensorVelocity(), Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO);
       
        inputs.driveOutputVoltageVolts = mDriveMotor.getMotorOutputPercent() * mDriveMotor.getBusVoltage(); // Might be Incorrect

        inputs.driveCurrentAmps = new double[] {mDriveMotor.getOutputCurrent()};
        inputs.driveTempCelcius = new double[] {mDriveMotor.getTemperature()};
    
        inputs.turnAbsolutePositionRad =
            MathUtil.angleModulus(
                new Rotation2d(
                        mAngleMotor.getMotorOutputVoltage()
                            / RobotController.getVoltage5V()
                            * 2.0
                            * Math.PI)
                    .minus(angleOffset)
                    .getRadians());

        inputs.turnPositionRad = Conversions.falconToMeters(mAngleMotor.getSelectedSensorPosition(), Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO);
        inputs.turnVelocityRadPerSec =Conversions.falconToMeters(mDriveMotor.getSelectedSensorVelocity(), Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO);

        inputs.turnAppliedVolts = mAngleMotor.getMotorOutputPercent() * mAngleMotor.getBusVoltage(); // Might be Incorrect
        inputs.turnCurrentAmps = new double[] {mAngleMotor.getOutputCurrent()};
        inputs.turnTempCelcius = new double[] {mAngleMotor.getTemperature()};

        gyro.getYawPitchRoll(yprDegrees);
        gyro.getRawGyro(xyzDps);
        inputs.connected = gyro.getLastError().equals(ErrorCode.OK);
        inputs.rollPositionRad = Units.degreesToRadians(yprDegrees[1]);
        inputs.pitchPositionRad = Units.degreesToRadians(-yprDegrees[2]);
        inputs.yawPositionRad = Units.degreesToRadians(yprDegrees[0]);
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(xyzDps[1]);
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-xyzDps[0]);
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(xyzDps[2]);
}
}