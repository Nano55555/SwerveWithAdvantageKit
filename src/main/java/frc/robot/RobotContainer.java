// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve.GyroIO;
import frc.robot.subsystems.Swerve.GyroPigeon2;
import frc.robot.subsystems.Swerve.Swerve;

import frc.robot.subsystems.Swerve.SwerveModuleFalcon500;
import frc.robot.subsystems.Swerve.SwerveModuleIO;
import frc.robot.subsystems.Swerve.SwerveModuleSim;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
      
      // double translationVal = -1.0 * MathUtil.applyDeadband(-m_driverController.getRawAxis(Constants.DriverControls.TRANSLATION_VAL) , Constants.STICK_DEADBAND);
      // double strafeVal = -1.0 * MathUtil.applyDeadband(-m_driverController.getRawAxis(Constants.DriverControls.STRAFE_VAL), Constants.STICK_DEADBAND);
      // double rotationVal = MathUtil.applyDeadband(-m_driverController.getRawAxis(Constants.DriverControls.ROTATION_VAL), Constants.STICK_DEADBAND);

  private Swerve swerve; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    switch(Constants.currentMode){
      case REAL:
          swerve = new Swerve(
          new GyroPigeon2(Constants.CAN_IDS.PIDGEON),
          new SwerveModuleFalcon500(Constants.SWERVE.Mod0.constants),
          new SwerveModuleFalcon500(Constants.SWERVE.Mod1.constants),
          new SwerveModuleFalcon500(Constants.SWERVE.Mod2.constants),
          new SwerveModuleFalcon500(Constants.SWERVE.Mod3.constants));
          Timer.delay(1.0);
          swerve.resetModulesToAbsolute();
          swerve.gyro.zeroGyro();
          break;
      case REPLAY:
      swerve = new Swerve(
          new GyroIO(){},
          new SwerveModuleIO(){},
          new SwerveModuleIO(){},
          new SwerveModuleIO(){},
          new SwerveModuleIO(){});
      case SIM:
          new Swerve(
          new GyroIO(){},
          new SwerveModuleSim(){},
          new SwerveModuleSim(){},
          new SwerveModuleSim(){},
          new SwerveModuleSim(){});
          break;
      default:
      swerve = new Swerve(
          new GyroPigeon2(Constants.CAN_IDS.PIDGEON),
          new SwerveModuleFalcon500(Constants.SWERVE.Mod0.constants),
          new SwerveModuleFalcon500(Constants.SWERVE.Mod1.constants),
          new SwerveModuleFalcon500(Constants.SWERVE.Mod2.constants),
          new SwerveModuleFalcon500(Constants.SWERVE.Mod3.constants));
          break;

}
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    if(Constants.currentMode == Constants.Mode.REAL){
  //   swerve.setDefaultCommand(new RunCommand(()-> swerve.drive(
  //     new Translation2d(translationVal, strafeVal).times(Constants.SWERVE.MAX_SPEED), 
  //     rotationVal * Constants.SWERVE.MAX_ANGULAR_VELOCITY, 
  //     true,
  //     true
  // ),swerve));
  //   }
  }
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
