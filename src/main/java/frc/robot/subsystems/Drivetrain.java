/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import java.lang.Math;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {
  

  CANSparkMax leftMotor = new CANSparkMax(Constants.MOTOR_LEFT_ID, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(Constants.MOTOR_RIGHT_ID, MotorType.kBrushless);
  
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  // how robot need to go
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(26));
  // how robot positioned
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

  // how robot turn wheel
  PIDController leftPIDController = new PIDController(0,0,0);
  PIDController rightPIDController = new PIDController(0,0,0);

  Pose2d pose;

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

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {

    //https://www.revrobotics.com/content/sw/max/sw-docs/java/com/revrobotics/CANEncoder.html#getVelocity()
    //https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html
    //https://www.andymark.com/products/am14u4-kit-of-parts-chassis
    //multiply the velocity of the motor to get to radians, and then divide by the gearing ratio

    double lm = leftMotor.getVelocity() * ((2*Math.Pi)/60)) * (1/10.75);
    double rm = rightMotor.getVelocity() * ((2*Math.Pi)/60)) * (1/10.75);
    
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }
  
  public void setOutput(double leftVolts, double rightVolts) {
    leftMotor.set(leftVolts);
    rightMotor.set(leftVolts);
  }

}
