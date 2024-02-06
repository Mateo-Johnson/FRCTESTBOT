// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class Intake extends CommandBase {
  /** Creates a new silly. */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  private final Color kOrangeTarget = new Color(0.453, 0.405, 0.141); //CREATES A COLOR
  public Intake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  //INTAKE
  public static final CANSparkMax intake1 = new CANSparkMax(12, MotorType.kBrushless);
  public static final CANSparkMax intake2= new CANSparkMax(13, MotorType.kBrushless);


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget); 
    m_colorMatcher.addColorMatch(kOrangeTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      intake1.set(0.6);
      intake2.set(0.6);
    } else if (match.color == kOrangeTarget){
      colorString = "Orange";
      intake1.set(0);
      intake2.set(0);
    } else {
      colorString = "Unknown";
      intake1.set(0.6);
      intake2.set(0.6);
    }
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake1.set(0);
    intake2.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
