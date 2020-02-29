/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.SpeedConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.VSimpleTurn;
import frc.robot.subsystems.BetterShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  // needs to be implemented
  private final Elevator elevator = new Elevator();
  private final Feeder feeder = new Feeder();
  private final Intake intake = new Intake();
  private final Magazine magazine = new Magazine();
  //private final Shooter shooter = new Shooter();
  private final BetterShooter shooter = new BetterShooter();

  // might need to be implemented
  // private final Spinner spinner = new Spinner();

  public static final Joystick stick1 = new Joystick(0);
  private final Joystick stick2 = new Joystick(1);

  private final UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture();

  //shoots the ball from line
  Command shootSequence = new InstantCommand(() -> shooter.shootNear(), shooter)
                            .andThen(new WaitUntilCommand(() -> shooter.atSpeed()))
                            .andThen(() -> feeder.feed(), feeder)
                            .andThen(() -> magazine.load(), magazine);

  //shoots the ball from trench
  Command shootSequenceFar = new InstantCommand(() -> shooter.shootFar(), shooter)
                            .andThen(new WaitUntilCommand(() -> shooter.atSpeed()))
                            .andThen(() -> feeder.feed(), feeder)
                            .andThen(() -> magazine.load(), magazine);

  Command blehBall = new InstantCommand(shooter::shoot, shooter)       
                     .andThen(feeder::feed, feeder);

  /*Command takeBack = new InstantCommand(intake::unploy)
                     .andThen(magazine::unload, shooter)       
                     .andThen(feeder::regurgitate, feeder);
  */
  //stops all motors except drivetrain
  Command kill = new InstantCommand(() -> intake.stop(), intake)
                  .andThen(() -> shooter.stop(), shooter)
                  .andThen(() -> magazine.stop(), magazine)
                  .andThen(() -> feeder.stop(), feeder);

  //centers on vision target
  Command visionTrack = new VSimpleTurn(drivetrain, stick1,VisionConstants.offset);

  public RobotContainer() 
  {
    //drive command
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(stick1.getRawAxis(4)*SpeedConstants.driveSpeed, -stick1.getRawAxis(1)*SpeedConstants.driveSpeed), drivetrain));
    //intake.deploy();
    configureLimelight();
    configureButtonBindings();
    configureShuffleboard();
  }

  /**
   * @apiNote configures limelight
   */
  private void configureLimelight(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  /**
   * @apiNote binds commmands to specific buttons
   */
  private void configureButtonBindings() 
  {
    JoystickButton a1, b1, x1, y1, start1, back1, lb1, rb1, lt1,
                   a2, b2, x2, y2, start2, back2, lb2, rb2, lt2;

    a1 = new JoystickButton(stick1, 1);
    b1 = new JoystickButton(stick1, 2);
    x1 = new JoystickButton(stick1, 3);
    y1 = new JoystickButton(stick1, 4);
    lb1 = new JoystickButton(stick1, 5);
    rb1 = new JoystickButton(stick1, 6);
    back1 = new JoystickButton(stick1, 7);
    start1 = new JoystickButton(stick1, 8);

    a2 = new JoystickButton(stick2, 1);
    b2 = new JoystickButton(stick2, 2);
    x2 = new JoystickButton(stick2, 3);
    y2 = new JoystickButton(stick2, 4);
    lb2 = new JoystickButton(stick2, 5);
    rb2 = new JoystickButton(stick2, 6);
    back2 = new JoystickButton(stick2, 7);
    start2 = new JoystickButton(stick2, 8);
    //auto-aims and then shoots (from trench)
    x1.whenPressed(new VSimpleTurn(drivetrain, stick1, VisionConstants.offsetFar).andThen(shootSequenceFar)).whenReleased(kill);
    //manual shoot button (from initiation line)
    lb1.whenPressed(shootSequence).whenReleased(kill);
    //auto-aims and then shoots (from initiation line)
    a1.whenPressed(new VSimpleTurn(drivetrain, stick1,VisionConstants.offsetNear).andThen(shootSequence)).whenReleased(kill);
    //rb1.whenPressed(shootSequenceFar).whenReleased(kill);
    rb1.toggleWhenPressed(new StartEndCommand(() -> intake.outtake(), () -> intake.stop(), intake));
    back1.toggleWhenPressed(new StartEndCommand(() -> intake.deploy(), () -> intake.unploy(), intake));
    start1.whenPressed(kill);
    //needs to be tested
    //shoots the ball from trench
    
    
    //testing commands only



    //second controller
    b2.whenPressed(blehBall).whenReleased(kill);//BLEH
    //a2.whenPressed(takeBack).whenReleased(kill);//apoptosis
    y2.toggleWhenPressed(new StartEndCommand(() -> magazine.load(), () -> magazine.stop(), magazine));
    lb2.whenPressed(() -> elevator.up()).whenReleased(() -> elevator.stop());
    rb2.whenPressed(() -> elevator.down()).whenReleased(() -> elevator.stop());

  }
  /**
   * Creates a trajectory
   * @return Command
   */
  public Command getAutonomousCommand() 
  {
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.AutoConstants.ksVolts,
                                   Constants.AutoConstants.kvVoltSecondsPerMeter,
                                   Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
                                   Constants.AutoConstants.kDriveKinematics,
                                   10);

    TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.AutoConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    //simple forward command?
    // Trajectory forward = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0, 0, new Rotation2d(0)),
    //   List.of(new Translation2d(0.25, 0)),
    //   new Pose2d(0.5, 0, new Rotation2d(0)),
    //   config);
    
    Trajectory forward = createTrajectory("/home/lvuser/deploy/paths/output/circ.wpilib.json");

    //translates trajectory to current robot pose
    RamseteCommand aForward = createRamsete(forward.relativeTo(drivetrain.getPose()));
  
    //runs trajectory, stops robot, aims with vision, shoots

    return aForward.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    //return aForward.andThen(aBackward).andThen(() -> drivetrain.tankDriveVolts(0, 0));
    //return auto.andThen(() -> drivetrain.tankDriveVolts(0, 0)).andt;//.andThen(new VSimpleTurn(drivetrain));//.andThen(shootSequence);
  }

  /**
   * creates a Ramsete Command for trajectory stuff
   * @param trajectory
   * @return RamseteCommand
   */
  private RamseteCommand createRamsete(Trajectory trajectory)
  {
    return new RamseteCommand(
              trajectory,
              drivetrain::getPose,
              new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
              new SimpleMotorFeedforward(Constants.AutoConstants.ksVolts,
                  Constants.AutoConstants.kvVoltSecondsPerMeter,
                  Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
                  Constants.AutoConstants.kDriveKinematics,
              drivetrain::getWheelSpeeds,
              new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
              new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
              drivetrain::tankDriveVolts,
              drivetrain
              );
  }
/**
 * creates trajectory path for autonomous
 * @param filePath
 * @return Trajectory
 */
  private Trajectory createTrajectory(String filePath){
    Trajectory trajectory = null;
    String trajectoryJSON = filePath;
    System.out.println("hiii" + Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON));
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    return trajectory;
  }

  /**
   * @apiNote configures the shuffleboard in order the view the information we want to see
   */
  private void configureShuffleboard(){

    // SendableChooser<String> autoChooser = new SendableChooser<>();
    // autoChooser.addOption("cat", "one");
    // autoChooser.addOption("dog", "two");
    // autoChooser.addOption("log", "three");
    // autoChooser.setDefaultOption("bat", "one");
    // SendableRegistry.setName(autoChooser, "autoChooser");

    // Shuffleboard.getTab("AutoChooser")
    // .add(autoChooser)
    // .withWidget(BuiltInWidgets.kComboBoxChooser)
    // .withSize(8, 4);
    
    //configures the camera output b/c the output is sideways
    Shuffleboard.getTab("HEhee")
    .add(cam0)
    .withWidget(BuiltInWidgets.kCameraStream)
    .withProperties(Map.of("Show crosshar", true, "Crosshair color", "white", "Rotation", "QUARTER_CCW"));
  }
}
