// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.classes.Gyro;
import frc.robot.classes.Lighting;
import frc.robot.classes.Lighting.Colors;
import frc.robot.classes.NCOrchestra;
import frc.robot.classes.NCPose;
import frc.robot.classes.Vision;
import frc.robot.constants.*;
import frc.robot.utils.CTREConfigs;
import frc.robot.utils.InputAxis;
import frc.robot.utils.NCDebug;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.AimerSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    public static final CTREConfigs ctreConfigs = new CTREConfigs();
    public static final Lighting lighting = Lighting.getInstance();
    public static final Gyro gyro = Gyro.getInstance();
    public static final Vision vision = Vision.getInstance();
    private final NCOrchestra orchestra = NCOrchestra.getInstance();
    // public static final DriveSubsystem drive = DriveSubsystem.getInstance(); //must be after gyro
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); //must be after gyro
    public static final NCPose pose = NCPose.getInstance(); //must be after drive
    public static final PowerDistribution power = new PowerDistribution(1,ModuleType.kRev);
    public static final ClimberSubsystem climber = ClimberSubsystem.getInstance();
    public static final CoralSubsystem coral = CoralSubsystem.getInstance();
    // public static final AlgaeSubsystem algae = AlgaeSubsystem.getInstance();
    
    public static Optional<Alliance> m_alliance;

    private AutoFactory autoFactory;
    private AutoRoutines autoRoutines;
        
    private final AutoChooser autoChooser = new AutoChooser();
    //Sendables definitions
    private SendableChooser<Command> m_auto_chooser = new SendableChooser<>();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(SwerveConstants.kMaxAngularRate).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);
    // public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotdrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandStadiaController dj = new CommandStadiaController(OIConstants.JoyDriverID);
    private final CommandStadiaController oj = new CommandStadiaController(OIConstants.JoyOperID);

    public RobotContainer() {
        final InputAxis m_fieldX = new InputAxis("Forward", dj::getLeftY)
            .withDeadband(OIConstants.kMinDeadband)
            .withInvert(true)
            .withSquaring(false);
        final InputAxis m_fieldY = new InputAxis("Strafe", dj::getLeftX)
            .withDeadband(OIConstants.kMinDeadband)
            .withInvert(true)
            .withSquaring(false);
        final InputAxis m_rotate = new InputAxis("Rotate", dj::getRightX)
            .withDeadband(OIConstants.kMinDeadband)
            .withInvert(true);
        autoFactory = new AutoFactory(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            drivetrain::followPath,
            true,
            drivetrain
        );
        autoRoutines = new AutoRoutines(autoFactory);
        
        initOrchestra();
        configureBindings();
        buildDashboards();

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(m_fieldX.getAsDouble() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(m_fieldY.getAsDouble() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(m_rotate.getAsDouble() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    }
    
    private void initOrchestra() {
        /**
         * This adds instruments to the orchestra. 
         * It is recommended there is at least 6-8 TalonFX devices in the orchestra.
         * Falcon500 are much louder than Krakens
         * 
         * This depends on each subsystem having a getMotors() method that returns an array of the TalonFX devices
         */
        if(AudioConstants.isEnabled) {
            ArrayList<TalonFX> instrumentsAll = new ArrayList<>();
            // The following subsystems return an array of motors (TalonFX[])
            for (TalonFX motor: drivetrain.getMotors()) {
                instrumentsAll.add(motor);
            }
            // for (TalonFX motor: shooter.getMotors()) {
            //   instrumentsAll.add(motor);
            // }
            TalonFX[] instruments = instrumentsAll.toArray(new TalonFX[instrumentsAll.size()]);
            orchestra.apply(instruments);
        }
    }

    /**
     * This performs robot reset initializations when the disabled() trigger fires.
     */
    private void resetRobot() {
        lighting.init();
        pose.init();
        drivetrain.init();
        // aimer.init();
        climber.init();
        // indexer.init();
        // intake.init();
        // arm.init();
        coral.init();
        // algae.init();
    }
    
    // Returns true if the alliance is red, otherwise false (blue)
    public static boolean isAllianceRed() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }
    
    private void configureBindings() {
        // bind to the disabled() trigger which happens any time the robot is disabled
        RobotModeTriggers.disabled().onTrue(
            new InstantCommand(this::resetRobot).ignoringDisable(true)
            .alongWith(
                new RepeatCommand(  //Lighting Disco Party!
                    lighting.setColorCommand(Colors.NCBLUE)
                    .andThen(new WaitCommand(0.5))
                    .andThen(lighting.setColorCommand(Colors.NCGREEN))
                    .andThen(new WaitCommand(0.5))
                    .andThen(lighting.setColorCommand(Colors.NCBLUE))
                    .andThen(new WaitCommand(0.15))
                    .andThen(lighting.setColorCommand(Colors.OFF))
                    .andThen(new WaitCommand(0.15))
                    .andThen(lighting.setColorCommand(Colors.NCBLUE))
                    .andThen(new WaitCommand(0.15))
                    .andThen(lighting.setColorCommand(Colors.NCGREEN))
                    .andThen(new WaitCommand(0.25))
                ).until(RobotModeTriggers.disabled().negate())
            )
        );
        // bind to the autonomous() and teleop() trigger which happens any time the robot is enabled in either of those modes
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(
            new InstantCommand(orchestra::stop).ignoringDisable(true)
            .andThen(() -> lighting.setColorCommand(Colors.OFF))
                // .andThen(climber.runOnce(climber::ratchetLock)).andThen(new WaitCommand(0.5)).andThen(climber.runOnce(climber::ratchetFree))
        );

        if(AudioConstants.isEnabled) {
            /** Manage Music - Song list
            *  Brawl-Theme.chrp
            *  Megalovania.chrp
            *  Rickroll.chrp
            *  Still-Alive.chrp
            */
            dj.stadia().onTrue(new InstantCommand(() -> {
                if(orchestra.isPlaying()) {
                    orchestra.stop();
                } else {
                    orchestra.withMusic("Still-Alive.chrp").play();
                }
            }).ignoringDisable(true));
        }

        //POV left and right are robot-centric strafing
        dj.povLeft().whileTrue(drivetrain.applyRequest(() -> 
                robotdrive.withVelocityX(0).withVelocityY(SwerveConstants.kAlignStrafeSpeed)
            ).alongWith(
                new InstantCommand(() -> { NCDebug.Debug.debug("Debug: StrafeLeft"); })
            )
        );
        dj.povRight().whileTrue(drivetrain.applyRequest(() -> 
            robotdrive.withVelocityX(0).withVelocityY(-SwerveConstants.kAlignStrafeSpeed)
            ).alongWith(
                new InstantCommand(() -> { NCDebug.Debug.debug("Debug: StrafeRight"); })
            )
        );

        // dj.frame().onTrue((Commands.runOnce(drivetrain::zeroGyro)));
        // dj.stadia().onTrue(Commands.runOnce(drivetrain::addFakeVisionReading));

        //hold A to apply brake
        dj.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //hold B to point the wheels forward
        dj.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-dj.getLeftY(),-dj.getLeftX()))
        ));

        // Run SysId routines when holding ellipses/google and X/Y.
        // Note that each routine should be run exactly once in a single log.
        dj.ellipses().and(dj.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        dj.ellipses().and(dj.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        dj.google().and(dj.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        dj.google().and(dj.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on hamburger button press
        dj.hamburger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    /**
     * Use this to pass the named command to the main Robot class.
     * @return command
     */
    public Command getAutonomousCommand() {
        if(AutonConstants.isDisabled) {
            NCDebug.Debug.debug("Robot: Selected auton routine is "+m_auto_chooser.getSelected().getName());
            return m_auto_chooser.getSelected();
        } else {
            NCDebug.Debug.debug("Robot: Selected auton routine is "+autoChooser.selectedCommand().getName());
            return autoChooser.selectedCommand();
        }
    }

    //From here down is all used for building the shuffleboard
    public void buildDashboards(){
        //List of Widgets: https://github.com/Gold872/elastic-dashboard/wiki/Widgets-List-&-Properties-Reference
        buildAutonChooser();
        buildDriverTab();
        // buildTab("Swerve");
        // buildTab("System");
        // buildPowerTab();
        // buildDebugTab();
        gyro.buildDashboards();
    }    
    
    public void buildAutonChooser() {
        //This builds the auton chooser, giving driver friendly names to the commands from above
        if(AutonConstants.isDisabled) {
            m_auto_chooser.setDefaultOption("None (Auto Disabled)", Commands.none());
        } else {
            // m_auto_chooser.setDefaultOption("Do Nothing", new cg_autonDoNothing(drive));
            if(AutonConstants.kUseChoreo) {
                autoFactory = drivetrain.createAutoFactory();
                autoRoutines = new AutoRoutines(autoFactory);
                // autoChooser.addRoutine("None (Do Nothing)", autoRoutines::doNothingAuto);
                autoChooser.addRoutine("Simple Forward", autoRoutines::simpleForwardAuto);
                // SmartDashboard.putData("Autonomous Chooser", autoChooser);
            // m_auto_chooser = autoChooser;
            }
        }
    }

    private void buildDriverTab(){
        ShuffleboardTab driverTab = buildTab("Driver");
        // Match Time - Cannot be programmatically placed, but we put it here for informative reasons
        driverTab.add("Match Time", "")
            .withPosition(0,2)
            .withSize(8,3)
            .withProperties(Map.of("time_display_mode","Minutes and Seconds","red_start_time",15,"yellow_start_time",30)) //mode: "Seconds Only" or "Minutes and Seconds"
            .withWidget("Match Time");
        // Auton Chooser
        if(AutonConstants.isDisabled) {
            driverTab.add("Autonomous Chooser", m_auto_chooser)
                .withPosition(0, 5)
                .withSize(8, 2)
                .withProperties(Map.of("sort_options",true))
                .withWidget("ComboBox Chooser");
        } else {
            // SmartDashboard.putData("Autonomous Chooser", autoChooser);
            driverTab.add("Autonomous Chooser", autoChooser)
                .withPosition(0, 5)
                .withSize(8, 2)
                .withProperties(Map.of("sort_options",true))
                .withWidget("ComboBox Chooser");
        }
            // FMS Info - Cannot be programmatically placed, but we put it here for informative reasons
            // driverTab.add("FMS Info", "")
            //   .withPosition(0,5)
            //   .withSize(8,2)
            //   .withWidget("FMSInfo");
            // Alerts
            // driverTab.add("Alerts", "")
            //   .withPosition(8,0)
            //   .withSize(11,7)
            //   .withWidget("Alerts");
        // Camera
        // driverTab.add("Camera", Robot.camera)
        //     .withPosition(18,0)
        //     .withSize(6,5)
        //     // .withProperties(Map.of("Glyph","CAMERA_RETRO","Show Glyph",true,"Show crosshair",true,"Crosshair color","#CCCCCC","Show controls",false))
        //     .withWidget("Camera Stream");
    }

    private void buildDebugTab(){
        ShuffleboardTab debugTab = buildTab("Debug");
        debugTab.add("Command Scheduler", CommandScheduler.getInstance())
            .withPosition(0,2);      
    }

    private void buildPowerTab(){
        ShuffleboardTab powerTab = buildTab("Power");
        powerTab.add("Power", power)
            .withPosition(0, 0);
            // .withSize(1, 1);
    }

    private ShuffleboardTab buildTab(String tabname) {
        return Shuffleboard.getTab(tabname);
    }

}
