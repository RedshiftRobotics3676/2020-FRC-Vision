/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SpeedConstants;

public class Spinner extends SubsystemBase 
{  
    private final WPI_TalonSRX spinner = new WPI_TalonSRX(PortConstants.spinner);
    //private final ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kOnboard);
    private SpinState state = SpinState.Color;

    public Spinner()
    {
        spinner.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        spinner.setNeutralMode(NeutralMode.Brake);
    }

    public void spin()
    {
        spinner.set(ControlMode.PercentOutput, SpeedConstants.spinnerSpeed);
    }

    public SpinState getState()
    {
        return state;
    }

    @Override
    public void periodic() 
    {
    
    }
}

enum SpinState
{
    Color, Distance,
}
