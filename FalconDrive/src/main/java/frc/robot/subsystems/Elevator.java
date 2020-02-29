/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SpeedConstants;

public class Elevator extends SubsystemBase 
{   
    //private PosState elevatorState = PosState.Default;

    private final WPI_TalonFX lElevator = new WPI_TalonFX(PortConstants.elevator1);
    private final WPI_TalonFX rElevator = new WPI_TalonFX(PortConstants.elevator2);
    /**
     * constructor for elevator class
     */
    public Elevator()
    {
        rElevator.follow(lElevator);
        rElevator.setInverted(InvertType.OpposeMaster);
    }
    /**
     * @apiNote makes elevator go up
     */
    public void up()
    {
        lElevator.set(ControlMode.PercentOutput, SpeedConstants.elevatorSpeed);
    }
    /**
     * @apiNote makes elevator go down
     */
    public void down()
    {
        lElevator.set(ControlMode.PercentOutput, -SpeedConstants.elevatorSpeed);
    }
    /**
     * @apiNote stops motor by setting its speed to 0
     */
    public void stop()
    {
        lElevator.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() 
    {
    
    }
}

enum PosState
{
    Default,
}
