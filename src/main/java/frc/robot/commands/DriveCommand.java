// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.chrono.ThaiBuddhistChronology;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveCommand extends CommandBase {
  private DifferentialDrive drive;
  private MotorControllerGroup leftControllerGroup;
  private MotorControllerGroup rightControllerGroup;
  /** Creates a new DriveCommand. */
  public DriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leftControllerGroup = new MotorControllerGroup(RobotContainer.l1, RobotContainer.l2);
    this.rightControllerGroup = new MotorControllerGroup(RobotContainer.r1, RobotContainer.r2);

    this.leftControllerGroup.setInverted(true);
    drive = new DifferentialDrive(this.leftControllerGroup, this.rightControllerGroup);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  
  public void execute() {
    drive.arcadeDrive(RobotContainer.stick.getY(), RobotContainer.stick.getX());
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
