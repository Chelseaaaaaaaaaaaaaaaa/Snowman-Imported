// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private TalonSRX leftMotor1 = new TalonSRX(0);
  private TalonSRX leftMotor2 = new TalonSRX(1);
  private TalonSRX rightMotor1 = new TalonSRX(2);
  private TalonSRX rightMotor2 = new TalonSRX(3);
  public DriveSubsystem() {

  

  }
  public void set(double x, double y){
    leftMotor1.set(ControlMode.PercentOutput, y+x);
    leftMotor2.set(ControlMode.PercentOutput, y+x);
    rightMotor1.set(ControlMode.PercentOutput, -y+x);
    rightMotor2.set(ControlMode.PercentOutput, -y+x);
  


  }




  
  /**
   * Example command factory method.
   *
   * @return a command
   */
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
