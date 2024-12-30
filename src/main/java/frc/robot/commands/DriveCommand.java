// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;

/** An example command that uses an example subsystem. */
public class DriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain driveTrain;
  private XboxController m_controller;

  // Throttle and turn variables
  private double throttle;
  private double turn;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveTrain subsystem) {
    driveTrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Initialize the XboxController
    m_controller = new XboxController(0); // Replace '0' with the appropriate port for your controller
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_controller.getLeftY() > Math.abs(0.05)) { // Checks if left joystick Y is greater than deadzone
      throttle = m_controller.getLeftY();
    }
    else {
      throttle = 0;
    }
    if(m_controller.getRightX() > Math.abs(0.05)) { // Checks if right joystick X is greater than deadzone
      turn = m_controller.getRightX();
    }
    else {
      turn = 0;
    }

    // Drive the robot with the current throttle and turn values
    driveTrain.manualDrive(throttle, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
