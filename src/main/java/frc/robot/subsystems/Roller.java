/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Roller extends SubsystemBase {
  VictorSPX rollerMotor = new VictorSPX(Constants.MOTOR_ROLLER_ID);
  /**
   * Creates a new Roller.
   */
  public Roller() {

  }
  
  //Run roller motors based off of Xbox controller values
  public void intake(XboxController controller) {
    /*if (controller.getRawAxis(Constants.JOYSTICK_INTAKE_ID) > 0.5) {
      rollerMotor.set(ControlMode.PercentOutput, 1.0);
      System.out.println("LEFT_BUMPER_BUTTON_ID- intake motor forward");
    } else if (controller.getRawAxis(Constants.JOYSTICK_OUTPUT_ID) > 0.5) {
      rollerMotor.set(ControlMode.PercentOutput, -1.0);
      System.out.print("RIGHT_BUMPER_BUTTON_ID- intake motor reverse"); 
    } else {
      rollerMotor.set(ControlMode.PercentOutput, 0);
      System.out.println("Turning off roller");
    }
    */
    double intake = controller.getRawAxis(Constants.JOYSTICK_INTAKE_ID);
    double output = controller.getRawAxis(Constants.JOYSTICK_OUTPUT_ID);
    rollerMotor.set(ControlMode.PercentOutput, intake - output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}