/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmMove;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Roll; 
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Winch;
import frc.robot.subsystems.Roller; 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // subsystem
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Arm arm = new Arm();
  private final Winch winch = new Winch();
  private final Roller roller = new Roller(); 

  // commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain);
  private final ArmMove armMove = new ArmMove(arm);
  private final Roll roll = new Roll(roller); 

  public static final XboxController controller = new XboxController(Constants.CONTROLLER_1_PORT_ID);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // setting default commands
    drivetrain.setDefaultCommand(arcadeDrive);
    arm.setDefaultCommand(armMove);
    roller.setDefaultCommand(roll); 
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
    config.setKinematics(drivetrain.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
      config);

    RamseteCommand command = new RamseteCommand(
      trajectory,
      drivetrain::getPose,
      new RamseteController(2.0, 0.7),
      drivetrain.getFeedforward(),
      drivetrain.getKinematics(),
      drivetrain::getSpeeds,
      drivetrain.getLeftPIDController(),
      drivetrain.getRightPIDController(),
      drivetrain::setOutput,
      drivetrain
    );

    return command;
  }
  
  /**
   * Resets the yaw to zero.  Does not recalbiarate.  
   */
  public void zeroGyro(){
    drivetrain.gyro.reset();
  }
}
