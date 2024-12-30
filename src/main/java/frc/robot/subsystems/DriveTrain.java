// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.signals.InvertedValue; // not used
import com.ctre.phoenix6.signals.NeutralModeValue; // not used
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Trajectory Classes
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class DriveTrain extends SubsystemBase {
 
  // Declare Motors
  private final WPI_VictorSPX LEFT_MOTOR_1;
  private final WPI_VictorSPX LEFT_MOTOR_2;
  private final WPI_VictorSPX RIGHT_MOTOR_1;
  private final WPI_VictorSPX RIGHT_MOTOR_2;

  // Declare motor groups
  private final MotorControllerGroup LEFT_MOTORS;
  private final MotorControllerGroup RIGHT_MOTORS;

  // Declare diffDrive
  private DifferentialDrive diffDrive;
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {

    // Initialize Motors
    LEFT_MOTOR_1 = new WPI_VictorSPX(Constants.LEFT_MOTOR_1_ID);
    LEFT_MOTOR_2 = new WPI_VictorSPX(Constants.LEFT_MOTOR_2_ID);
    RIGHT_MOTOR_1 = new WPI_VictorSPX(Constants.RIGHT_MOTOR_1_ID);
    RIGHT_MOTOR_2 = new WPI_VictorSPX(Constants.RIGHT_MOTOR_2_ID);

    // Group motors on each side, using MotorControllerGroup
    LEFT_MOTORS = new MotorControllerGroup(LEFT_MOTOR_1, LEFT_MOTOR_2);
    RIGHT_MOTORS = new MotorControllerGroup(RIGHT_MOTOR_1, RIGHT_MOTOR_2);
    
    // Set neutral mode for front and back motors
    LEFT_MOTOR_1.setNeutralMode(NeutralMode.Coast);
    LEFT_MOTOR_2.setNeutralMode(NeutralMode.Coast);
    RIGHT_MOTOR_1.setNeutralMode(NeutralMode.Brake);
    RIGHT_MOTOR_2.setNeutralMode(NeutralMode.Brake);

    // Configure motor inversion
    LEFT_MOTOR_1.setInverted(false);
    LEFT_MOTOR_2.setInverted(false);
    RIGHT_MOTOR_1.setInverted(true);
    RIGHT_MOTOR_2.setInverted(true);
    
    // Robot's drive
    diffDrive = new DifferentialDrive(LEFT_MOTORS, RIGHT_MOTORS);
  }

  // method that takes throttle and turn parameters and applies them to a call of the arcadeDrive method, so diffDrive doesn't have to be imported elsewhere
  public void manualDrive(double throttle, double turn) {
    diffDrive.arcadeDrive(throttle, turn);
  }

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
