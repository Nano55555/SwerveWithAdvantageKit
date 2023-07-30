// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotContainer;


/** Add your docs here. */
public class SwerveModuleSim implements SwerveModuleIO {
  // private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
  // private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);
  
    public SwerveModuleSim() {
        System.out.println("[Init] Creating ModuleIOSim");
      }
    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
         inputs.driveMotorSensorPos = RobotContainer.m_driverController.getLeftY();
    }
  
}
