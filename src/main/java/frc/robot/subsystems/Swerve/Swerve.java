// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

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


/** Class with methods that get used in states of DrivetrainStateMachine */
public class Swerve extends SubsystemBase{

    public static SwerveDrivePoseEstimator swerveOdometry;
    public static SwerveModuleFalcon500[] mSwerveMods;

    public static Timer timer = new Timer();

    private static double rollOffset = 0.0;
    private static double pitchOffset = 0.0;


    private final SwerveIO flSwerveIO;
    private final SwerveIO frSwerveIO;
    private final SwerveIO blSwerveIO;
    private final SwerveIO brSwerveIO;
    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
   
    public Swerve( SwerveIO flSwerveIO,
    SwerveIO frSwerveIO,
    SwerveIO blSwerveIO,
    SwerveIO brSwerveIO ) {
        this.flSwerveIO  = flSwerveIO;
        this.frSwerveIO = frSwerveIO;
        this.blSwerveIO = blSwerveIO;
        this.brSwerveIO = brSwerveIO;

        swerveOdometry = new SwerveDrivePoseEstimator(
            Constants.SWERVE.SWERVE_KINEMATICS, 
            SwerveModuleFalcon500.gyro.getRotation2d(), 
            getModulePositions(), 
            new Pose2d()
        );
    
    }

    public static void zeroRoll() {
        rollOffset =  SwerveModuleFalcon500.gyro.getRoll();
    }

    public static double getRoll() {
        return  SwerveModuleFalcon500.gyro.getRoll() - rollOffset;
    }

    public static void zeroPitch() {
        pitchOffset =  SwerveModuleFalcon500.gyro.getPitch();
    }

    public static double getPitch() {
        return  SwerveModuleFalcon500.gyro.getPitch() - pitchOffset;
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

        for(SwerveModuleFalcon500 mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SWERVE.MAX_SPEED);
        
        for(SwerveModuleFalcon500 mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
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
        for(SwerveModuleFalcon500 mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModuleFalcon500 mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        SwerveModuleFalcon500.gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.SWERVE.INVERT_GYRO) ? Rotation2d.fromDegrees(360 -  SwerveModuleFalcon500.gyro.getYaw()) : Rotation2d.fromDegrees( SwerveModuleFalcon500.gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModuleFalcon500 mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    @Override
  public void periodic(){
    flSwerveIO.updateInputs(inputs);
    frSwerveIO.updateInputs(inputs);
    blSwerveIO.updateInputs(inputs);
    brSwerveIO.updateInputs(inputs);
    }
}