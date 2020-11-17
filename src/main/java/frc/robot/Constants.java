/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // controller port
    public static final int CONTROLLER_1_PORT_ID = 0; // switches temporarily

    // arm ID
    public static final int MOTOR_ARM_ID = 20;

    // intake ID
    public static final int MOTOR_MAILBOX_ID = 31;

    // drivetrain motors
    public static final int MOTOR_LEFT_BACK_ID = 12;
    public static final int MOTOR_RIGHT_BACK_ID = 10;
    public static final int MOTOR_LEFT_FRONT_ID = 13;
    public static final int MOTOR_RIGHT_FRONT_ID = 11;

    public static final int MOTOR_CLIMB_EXTENDER_ID = 40;
    public static final int MOTOR_CLIMB_WINCH_ID = 50;

    // left joystick (horizontal)
    public static final int JOYSTICK_DRIVE_FORWARDS_ID = 1;
    // left joystick (horizontal)
    public static final int JOYSTICK_DRIVE_ROTATION_ID = 0;
    // right joystick (vertical)
    public static final int JOYSTICK_ARM_CONTROL_ID = 3;
    // triggers (L,R accordingly)
    public static final int JOYSTICK_INTAKE_ID = 2;
    public static final int JOYSTICK_OUTPUT_ID = 3;
    // buttons
    public static final int A_BUTTON_ID = 1;
    public static final int B_BUTTON_ID = 2;
    public static final int X_BUTTON_ID = 3;
    public static final int LEFT_BUMPER_BUTTON_ID = 5;
    public static final int RIGHT_BUMPER_BUTTON_ID = 6;

    // backwards button
    public static final int LB_BUTTON_ID = 4;

    // arm setpoints
    public static final double ARM_POS_MIN = 0;
    public static final double ARM_POS_MAX = 60;

    // arm positions
    public static final double ARM_POS_LOWER = -1082.0;
    // real position is 566.0, but because of hard stop lowered it down a tiny bit
    public static final double ARM_POS_UPPER = 550.0;
}
