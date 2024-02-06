// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.Intake;

public class Outtake extends CommandBase {
  /** Creates a new silly. */
  public Outtake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  //OUTTAKE
  public static final CANSparkMax OUT1 = new CANSparkMax(14, MotorType.kBrushless);
  public static final CANSparkMax OUT2 = new CANSparkMax(11, MotorType.kBrushless);
  public static final CANSparkMax IN1 = Intake.intake1;
  public static final CANSparkMax IN2 = Intake.intake2;


  double rpm1 = OUT1.getEncoder().getVelocity();
  double rpm2 = OUT2.getEncoder().getVelocity();
  double avgRPM = (rpm1 + rpm2)/2;
  double rpm3 = IN1.getEncoder().getVelocity();
  double rpm4 = IN2.getEncoder().getVelocity();


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Intake.hasRing == true){
      
    if (avgRPM <= 5000) { //IF THE MOTORS ARE SPINNING FAST ENOUGH 
      OUT1.set(1); //SET UP OUTTAKE MOTOR 1 FOR SHOOTING
      OUT2.set(1); //SET UP OUTTAKE MOTOR 2 FOR SHOOTING
      IN1.set(0.6); //USE INTAKE MOTOR 1 TO FEED INTO OUTTAKE
      IN2.set(0.6); //USE INTAKE MOTOR 2 TO FEED INTO OUTTAKE
      //ADD A METHOD MAKE THE BOTTOM LIGHTS GREEN TO SHOW THAT ITS READY TO SHOOT

    } else if (avgRPM <= 5000) { //IF THE MOTORS ARE NOT AT THE RIGHT SPEED
      OUT1.set(1); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
      OUT2.set(1); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
      //ADD A METHOD MAKE THE BOTTOM LIGHTS RED TO SHOW THAT ITS NOT READY TO SHOOT
      }
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    OUT1.set(0);
    OUT2.set(0);
    IN1.set(0);
    IN2.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
