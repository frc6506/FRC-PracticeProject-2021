/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
//import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.Math;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {
  
  //Motor definitions
  CANSparkMax leftMotor = new CANSparkMax(Constants.MOTOR_LEFT_ID, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(Constants.MOTOR_RIGHT_ID, MotorType.kBrushless);
  
  //NavX gyroscope
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  // how robot need to go
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(26));
  // how robot positioned
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.4, 1, 0.4);

  // how robot turn wheel
  PIDController leftPIDController = new PIDController(0.1,0.0,0.05);
  PIDController rightPIDController = new PIDController(0.1,0.0,0.05); 

  Pose2d pose;

  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    
  }

  /**
   * @param XboxController controller
   */
  public void driveWithJoysticks(XboxController controller) {
    drive.arcadeDrive((controller.getRawAxis(1)) * -1*.5, controller.getRawAxis(0)*.5,true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), (0.0254 * (leftMotor.getEncoder().getPosition() * (1/10.75) * 6 * Math.PI)), (0.0254 * (rightMotor.getEncoder().getPosition() * (1/10.75) * 6 * Math.PI)));
    SmartDashboard.putNumber("Gyro Angle", -gyro.getAngle()); 
  }

  /**
   * Returns rotation of the robot
   * @return Angle of the gyroscope on a 2d plane in degrees
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  /**
   * Generates DifferentialDriveWheelSpeeds instance (how fast either wheel is
   * going)
   * 
   * @return new DifferentialDriveWheelSpeeds(leftMotor, rightMotor)
   */
  public DifferentialDriveWheelSpeeds getSpeeds() {

    //https://www.revrobotics.com/content/sw/max/sw-docs/java/com/revrobotics/CANEncoder.html#getVelocity()
    //https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html
    //https://www.andymark.com/products/am14u4-kit-of-parts-chassis
    
    //multiply the velocity of the motor to get to radians, and then divide by the gearing ratio, and multiply the radius of the wheel in meters to get tangential velocity in m/s
    double lm = leftMotor.getEncoder().getVelocity() * ((2*Math.PI)/60) * (1/10.75) * 0.0762;
    double rm = rightMotor.getEncoder().getVelocity() * ((2*Math.PI)/60) * (1/10.75) * 0.0762;
    //return statement is just rewritten simpler
    return new DifferentialDriveWheelSpeeds(lm, rm);
    
  }

  /**
   * converts overall robot movement into left/right wheel velocities
   * 
   * @return DifferentialDriveKinematics kinematics, Robot's current left/right
   *         wheel velocities
   */
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Returns current robot orientation.
   * Includes position and rotation, but primarily used for
   * position
   * @return pose, Robot's current Pose2d
   */
  public Pose2d getPose() {
    return pose;
  }

  /**
   * Estimates PID value to speed up predictions and reduce error
   * @return SimpleMotorFeedforward feedforward, the feedforward value
   */
  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }
  
  /**make wheel go zoom
   * @param double leftVolts
   * @param double rightVolts
   */ 
  public void setOutput(double leftVolts, double rightVolts) {
    leftMotor.set(0);
    rightMotor.set(0);
  }

}
