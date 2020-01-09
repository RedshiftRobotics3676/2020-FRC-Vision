/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TrackTarget extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Vision visionProcessing;
  private final Drivetrain drivetrain;

  private double tx;
  private double tv;
  private double speed;
  private double steer;

  public TrackTarget(Vision subsystem, Drivetrain drive) 
  {
    visionProcessing = subsystem;
    drivetrain = drive;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() 
  {
      speed = 0;
      steer = 0;
  }

  @Override
  public void execute() 
  {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    
    if(tv < 1.0)
    {
        drivetrain.drive(0, 0);
    }
    else
    {
        steer = tx*VisionConstants.kP;

        drivetrain.drive(speed, steer);
    }
  }

  @Override
  public void end(boolean interrupted) 
  {
  }

  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
