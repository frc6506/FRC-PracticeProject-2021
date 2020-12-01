/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivetrain extends SubsystemBase {

  CANSparkMax leftMotor = new CANSparkMax(Constants.MOTOR_LEFT_ID, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(Constants.MOTOR_RIGHT_ID, MotorType.kBrushless);
  

  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
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
