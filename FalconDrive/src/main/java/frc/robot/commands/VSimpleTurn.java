/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.SpeedConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VSimpleTurn extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;

  private double tx;
  private double tv;
  //private double speed;
  private double offset;
  private double steer;
  private double sum;
  private int fin_time;
  private Joystick stick;
  public static boolean status;

  public VSimpleTurn(Drivetrain drive, Joystick s, double offset) 
  {
    drivetrain = drive;
    this.offset = offset;
    stick = s;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() 
  {
      sum = 0;
      //speed = 0;
      steer = 0;
      fin_time = 0;
      status = false;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  @Override
  public void execute() 
  {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0) + offset;
    if(tv < 1.0)
        end(true);
        //drivetrain.drive(0, 0);
    else
    {
        //P-Loop
        if(tx < 0)
          steer = -VisionConstants.minDriveSpeed + tx*VisionConstants.kP;
        else
          steer = VisionConstants.minDriveSpeed + tx*VisionConstants.kP;
        //I-Loop

        if(Math.abs(tx) > VisionConstants.minThreshold)
          sum += tx*VisionConstants.kI;
        else
          sum = 0;  

        steer += sum;

        if(steer > VisionConstants.maxSteer)
          steer = VisionConstants.maxSteer;
        if(steer < -VisionConstants.maxSteer)
          steer = -VisionConstants.maxSteer;

        System.out.printf("steer:%f tx:%f sum:%f fin_time:%d\n", steer, tx, sum, fin_time);
        //System.out.println(tx);  
        //BORKE MAKE SURE DIRECTION IS CORRECT
        drivetrain.drive(steer, -stick.getRawAxis(1)*SpeedConstants.driveSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    status = true;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    drivetrain.drive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(tx) < VisionConstants.minThreshold && fin_time++ >= 10;
  }
}
