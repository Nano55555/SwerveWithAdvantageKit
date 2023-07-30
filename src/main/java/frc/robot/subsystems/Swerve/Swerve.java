// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.math.Conversions;

/** Class with methods that get used in states of DrivetrainStateMachine */
public class Swerve extends SubsystemBase{

    public static SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;

    public GyroIO gyro;

    public static Timer timer = new Timer();

    private static double rollOffset = 0.0;
    private static double pitchOffset = 0.0;

    private final static GyroIOInputsAutoLogged GyroInputs = new GyroIOInputsAutoLogged();
   
    public Swerve(
    GyroIO gyro,
    SwerveModuleIO flModuleIO,
    SwerveModuleIO frModuleIO,
    SwerveModuleIO blModuleIO,
    SwerveModuleIO brModuleIO) {
        this.gyro = gyro;
        mSwerveMods = new SwerveModule[]{
            new SwerveModule(flModuleIO, 0),
            new SwerveModule(frModuleIO, 1),
            new SwerveModule(blModuleIO, 2),
            new SwerveModule(brModuleIO, 3),
        };

        swerveOdometry = new SwerveDrivePoseEstimator(
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Rotation2d.fromDegrees(GyroInputs.yaw), 
            getModulePositions(), 
            new Pose2d()
        );
    }
    
    @Override
    public void periodic(){
        for(SwerveModule mod : mSwerveMods){
            mod.period();
        }
        gyro.updateInputs(GyroInputs);
        Logger.getInstance().processInputs("Gyro", GyroInputs);
    }

    public static void zeroRoll() {
        rollOffset = GyroInputs.roll;
    }

    public static double getRoll() {
        return GyroInputs.roll - rollOffset;
    }

    public static void zeroPitch() {
        pitchOffset = GyroInputs.pitch;
    }

    public static double getPitch() {
        return  GyroInputs.pitch - pitchOffset;
    }
    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.SWERVE.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SWERVE.MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.io.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop, mod.getState());
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SWERVE.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.io.setDesiredState(desiredStates[mod.moduleNumber], false, mod.getState());
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Rotation2d getYaw() {
        return (Constants.SWERVE.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - GyroInputs.yaw) : Rotation2d.fromDegrees(GyroInputs.yaw);
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            double absolutePosition = Conversions.degreesToFalcon(mod.getCanCoder().getDegrees() - mod.angleOffset.getDegrees(), Constants.SWERVE.ANGLE_GEAR_RATIO);
            mod.io.resetToAbsolute(absolutePosition);
        }
    }
}