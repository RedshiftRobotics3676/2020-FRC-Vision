/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.PortConstants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase
{
  private final WPI_TalonSRX leftTalon = new WPI_TalonSRX(PortConstants.LeftTalon);
  private final WPI_TalonSRX rightTalon = new WPI_TalonSRX(PortConstants.RightTalon);
  private final DifferentialDrive drive = new DifferentialDrive(leftTalon, rightTalon);

  private final WPI_VictorSPX leftVictor = new WPI_VictorSPX(PortConstants.LeftVictor);
  private final WPI_VictorSPX rightVictor = new WPI_VictorSPX(PortConstants.RightVictor);

  private final AHRS navX = new AHRS(SerialPort.Port.kMXP);

  private final Joystick js = new Joystick(0);

  public Drivetrain()
  {
    drive.setSafetyEnabled(false);

    leftVictor.follow(leftTalon);
    leftVictor.setInverted(InvertType.FollowMaster);
    rightVictor.follow(rightTalon);
    rightVictor.setInverted(InvertType.FollowMaster);

    leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftTalon.setSelectedSensorPosition(0);
    rightTalon.setSelectedSensorPosition(0);
  }

  public void drive(double speed, double steer)
  {
    drive.arcadeDrive(speed, steer, true);
  }

  @Override
  public void periodic() 
  {
    drive.arcadeDrive(-js.getRawAxis(1), js.getRawAxis(4), true);
  }
}
