// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class silly extends CommandBase {
  /** Creates a new silly. */
  public silly() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  //OUTTAKE
  public static final CANSparkMax c2 = new CANSparkMax(14, MotorType.kBrushless);
  public static final CANSparkMax a1 = new CANSparkMax(11, MotorType.kBrushless);


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c2.set(5);
    a1.set(5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c2.set(0);
    a1.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
