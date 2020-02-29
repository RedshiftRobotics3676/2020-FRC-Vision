/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Auto extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;

  public Auto(Drivetrain drive) 
  {
    drivetrain = drive;
    drivetrain.setRamp(SpeedConstants.autoDriveRampSpeed);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() 
  {
    drivetrain.reset();
  }

  @Override
  public void execute() 
  {
    drivetrain.drive(0, -SpeedConstants.autoDriveSpeed);
    //System.out.println(drivetrain.leftEncoder());
  }

  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.setRamp(SpeedConstants.driveRampSpeed);
    drivetrain.drive(0, 0);
  }

  @Override
  public boolean isFinished() 
  {
    return -drivetrain.leftEncoder() > AutoConstants.driveDistance;
  }
}