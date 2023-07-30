// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.lib.math.Conversions;

/** Add your docs here. */
public class SwerveModule {
    public Rotation2d angleOffset;

    public int moduleNumber;
    public final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    
    public SwerveModule(SwerveModuleIO io, int moduleNumber){
        this.io = io;
        this.moduleNumber = moduleNumber;

        switch(moduleNumber){
            case 0:
            angleOffset = Rotation2d.fromDegrees(56.6);
            break;
            case 1:
            angleOffset = Rotation2d.fromDegrees(35.4);
            break;
            case 2:
            angleOffset = Rotation2d.fromDegrees(348.9);
            break;
            case 3:
            angleOffset = Rotation2d.fromDegrees(306.9);
            break;
        }
    }
    
    public void period(){
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("SwerveModule" +moduleNumber, inputs);
    }
    
    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(inputs.angleMotorSensorPosition, Constants.SWERVE.ANGLE_GEAR_RATIO));
    }
    
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(inputs.angleEncoderAbsolutePos);
    }
    
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(inputs.driveMotorSensorVelocity, Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(inputs.driveMotorSensorPos, Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }

}
