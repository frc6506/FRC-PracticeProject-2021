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
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import java.lang.Math;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {
  
  // Motor definitions
  CANSparkMax leftMotor = new CANSparkMax(Constants.MOTOR_LEFT_ID, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(Constants.MOTOR_RIGHT_ID, MotorType.kBrushless);

  // Simulated encoder definitions
  CANEncoder encoderLeft = leftMotor.getEncoder(); 
  CANEncoder encoderRight = rightMotor.getEncoder();

  SimDeviceSim simEncoderLeft = new SimDeviceSim("SPARK MAX [" + Constants.MOTOR_LEFT_ID + "]");
  SimDeviceSim simEncoderRight = new SimDeviceSim("SPARK MAX [" + Constants.MOTOR_RIGHT_ID + "]");

  SimDouble leftPos = simEncoderLeft.getDouble("Position");
  SimDouble leftVel = simEncoderLeft.getDouble("Velocity");
  SimDouble rightPos = simEncoderRight.getDouble("Position");
  SimDouble rightVel = simEncoderLeft.getDouble("Velocity");

  // Simulated field
  private Field2d m_field = new Field2d();
  
  // NavX gyroscope
  public AHRS gyro = new AHRS(SPI.Port.kMXP);

  // Simulated gyroscope(?)
  //May definately need to change some values later
  //SimDeviceSim simNavX = new SimDeviceSim("NavX Sensor [4]");  //Old: "NavX Sensor [0]" The kMXP port is 4 (See SPI.Port.kMXP enum thingy)
  int simNavX = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  SimDouble simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(simNavX, "Yaw"));
  double angle_deg = simAngle.get();
  
  // Robot kinematics (movement)
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(26));
  // Robot change in position over tim
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  // Feedforward controller for Ramsete
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.137, 3.24, 0.592);

  // how robot turn wheel
  PIDController leftPIDController = new PIDController(2.21,0.0,0.0);
  PIDController rightPIDController = new PIDController(2.21,0.0,0.0); 

  Pose2d pose;

  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  //Create drivetrain simulation
  static final double KvLinear = 3.24; 
  static final double KaLinear = 0.592;
  static final double KvAngular = 1.5; 
  static final double KaAngular = 0.3; 
  static final double trackWidthMeters = 0.51;

  private DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular), 
    DCMotor.getNEO(2), 
    10.75,
    0.509, 
    Units.inchesToMeters(3),

    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)); 

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    // pass Field2d over NetworkTables
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Called once per scheduler run
   */
  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), (0.0254 * (leftMotor.getEncoder().getPosition() * (1 / 10.75) * 6 * Math.PI)),
        (0.0254 * (rightMotor.getEncoder().getPosition() * (1 / 10.75) * 6 * Math.PI)));

    SmartDashboard.putNumber("Gyro Angle", -gyro.getAngle());
  }
  //Simulation Periodic
  public void simulationPeriodic(){
    m_driveSim.setInputs(leftMotor.get() * RobotController.getInputVoltage(), 
                         rightMotor.get() * RobotController.getInputVoltage());

    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.07); 

    //Update sensors
    simEncoderLeft.getDouble("Position").set(m_driveSim.getLeftPositionMeters()); 
    simEncoderLeft.getDouble("Velocity").set(m_driveSim.getLeftVelocityMetersPerSecond());
    simEncoderRight.getDouble("Position").set(m_driveSim.getRightPositionMeters()); 
    simEncoderRight.getDouble("Velocity").set(m_driveSim.getRightVelocityMetersPerSecond());
    //Update gyro
    simAngle.set(-m_driveSim.getHeading().getDegrees());
  }
  /**
   * @param XboxController controller
   */
  public void driveWithJoysticks(XboxController controller) {
    drive.arcadeDrive((controller.getRawAxis(1)) * -1, controller.getRawAxis(0),true);
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

    // https://www.revrobotics.com/content/sw/max/sw-docs/java/com/revrobotics/CANEncoder.html#getVelocity()
    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html
    // https://www.andymark.com/products/am14u4-kit-of-parts-chassis
    
    // multiply the velocity of the motor to get to radians, and then divide by the gearing ratio, and multiply the radius of the wheel in meters to get tangential velocity in m/s
    double lm = leftMotor.getEncoder().getVelocity() * ((2*Math.PI)/60) * (1/10.75) * 0.0762;
    double rm = rightMotor.getEncoder().getVelocity() * ((2*Math.PI)/60) * (1/10.75) * 0.0762;
    // return statement is just rewritten simpler
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
    leftMotor.set(leftVolts / 12);
    rightMotor.set(rightVolts / 12);
  }

}
