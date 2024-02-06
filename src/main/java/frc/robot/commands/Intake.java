// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class Intake extends CommandBase {
  /** Creates a new silly. */
  public static final I2C.Port I2CPort = I2C.Port.kOnboard;
  public static final ColorSensorV3 colorSensor = new ColorSensorV3(I2CPort);
  private final ColorMatch colorMatcher = new ColorMatch(); //CREATES A NEW COLOR MATCHER (USED TO REGISTER AND DETECT KNOWN COLORS)
  private final Color orange = new Color(0.25, 0.73, 0.48); //CREATES A COLOR
  public Intake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  //INTAKE
  public static final CANSparkMax a2 = new CANSparkMax(12, MotorType.kBrushless);
  public static final CANSparkMax c1= new CANSparkMax(13, MotorType.kBrushless);


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorMatcher.addColorMatch(orange); //ADD A COLOR FOR REFERENCE TO THE COLOR MATCHER
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Color detectedColor = colorSensor.getColor(); //DETECT THE COLOR FROM THE COLOR SENSOR

    colorMatcher.setConfidenceThreshold(0.5);
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    colorSensor.getRawColor();
    colorMatcher.matchClosestColor(detectedColor);

    if (match.color != orange) {
      c1.set(0.6);
      a2.set(0.6);
    } else if (match.color == orange) {
    c1.set(0);
    a2.set(0);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c1.set(0);
    a2.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
