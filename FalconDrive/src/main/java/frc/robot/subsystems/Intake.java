/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SpeedConstants;

public class Intake extends SubsystemBase 
{
    private final WPI_VictorSPX intakeVictor = new WPI_VictorSPX(PortConstants.intake);
    private final DoubleSolenoid leftPiston = new DoubleSolenoid(PortConstants.fPiston, PortConstants.rPiston);
    /**
     * @apiNote for Intake Class, no parameters
     */
    public Intake()
    {
        intakeVictor.configClosedloopRamp(SpeedConstants.rampSpeed);
        intakeVictor.setNeutralMode(NeutralMode.Brake);

        //leftPiston.set(Value.kForward);
    }

    /**
     * @apiNote intake to take in balls
     */
    public void deploy()
    {
        stop();
        leftPiston.set(Value.kForward);
    }

    /**
     * @apiNote intake
     */
    public void unploy()
    {
        intake();
        leftPiston.set(Value.kReverse);
    }
    /**
     * @apiNote victors clockwise so the intake can take in the balls
     */
    public void intake()
    {
        intakeVictor.set(ControlMode.PercentOutput, SpeedConstants.intakeSpeed);
    }
    /**
     * @apiNote victors counter clockwise to push balls out (makes sure we dont get past 5)
     */
    public void outtake()
    {
        intakeVictor.set(ControlMode.PercentOutput, -SpeedConstants.intakeSpeed);
    }
    /**
     * @apiNote input of victors to 0
     */
    public void stop()
    {
        intakeVictor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() 
    {
       
    }
}
