/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  // TalonSRX armMotor = new TalonSRX(Constants.MOTOR_ARM_ID);
  /**
   * Creates a new Arm.
   */
  public Arm() {
    
  }

  public void turn(XboxController controller) {
    // armMotor.set(ControlMode.PercentOutput, controller.getRawAxis(Constants.JOYSTICK_ARM_CONTROL_ID));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
