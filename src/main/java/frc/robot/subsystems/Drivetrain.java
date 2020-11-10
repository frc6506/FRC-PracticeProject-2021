/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Drivetrain extends SubsystemBase {

  Spark leftBack = new Spark(Constants.MOTOR_LEFT_BACK_ID);
  Spark leftFront = new Spark(Constants.MOTOR_LEFT_FRONT_ID);
  Spark rightBack = new Spark(Constants.MOTOR_RIGHT_BACK_ID);
  Spark rightFront = new Spark(Constants.MOTOR_RIGHT_FRONT_ID);
  
  SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftBack, leftFront);
  SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightBack, rightFront);

  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    
  }

  public void driveWithJoysticks(XboxController controller) {
    drive.arcadeDrive(controller.getRawAxis(1), controller.getRawAxis(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
