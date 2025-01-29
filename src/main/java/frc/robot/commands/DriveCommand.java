// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;





/** An example command that uses an example subsystem. */
public class DriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final DoubleSupplier m_x;
  private final DoubleSupplier m_y;
  private DriveSubsystem m_driveSubsystem;

  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem m_driveSubsytem,DoubleSupplier x, DoubleSupplier y) {
   
    m_x=x;
    m_y=y;
    this.m_driveSubsystem=m_driveSubsytem;
    addRequirements(m_driveSubsytem); 

  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    double x=m_x.getAsDouble();
    double y=m_y.getAsDouble();
    m_driveSubsystem.set(x, y);
    
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
