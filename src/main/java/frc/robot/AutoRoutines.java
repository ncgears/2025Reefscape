package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.classes.Targeting.Targets;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.NCDebug;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine doNothingAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Do Nothing");
        final AutoTrajectory simplePath = routine.trajectory("Nothing");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    public AutoRoutine simpleForwardAuto() {
      final AutoRoutine routine = m_factory.newRoutine("SimpleForward");
      final AutoTrajectory simplePath = routine.trajectory("SimpleForward");

      RobotContainer.targeting.resetPose(simplePath.getInitialPose().get());
      routine.active().onTrue(
          simplePath.resetOdometry()
              .andThen(simplePath.cmd())
      );
      return routine;
    }

    public AutoRoutine moveOffLine() {
      final AutoRoutine routine = m_factory.newRoutine("MoveOffLine");
      final AutoTrajectory path = routine.trajectory("MoveOffLine");
    
      RobotContainer.targeting.resetPose(path.getInitialPose().get());
      routine.active().onTrue(
          path.resetOdometry()
              .andThen(path.cmd())
      );
      return routine;
    }

    public AutoRoutine leftAlgaeDouble() {
      final AutoRoutine routine = m_factory.newRoutine("LeftAlgaeDouble");
      final AutoTrajectory path1 = routine.trajectory("sLCb-rBL_c");
      final AutoTrajectory path2 = routine.trajectory("rBL_c-bC");
      final AutoTrajectory path3 = routine.trajectory("bC-aL-bC");
    
      path1.atTime("algae_high_intake").onTrue(
          log("EVENT(barge_ready)")
          .andThen(RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.ALGAEHIGH))
            .until(RobotContainer.elevator::isAtTarget)
          .andThen(wait(0.5))
          .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.REEF))
          .andThen(RobotContainer.algae.startToroC(false))
          .andThen(wait(0.5))
      );

      path2.atTime("barge_ready").onTrue(
        log("EVENT(barge_ready)")
        .andThen(RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.BARGE))
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP))
      );
      path2.atTime("barge_score").onTrue(
        log("EVENT(barge_score)")
        .andThen(RobotContainer.algae.startToroC(true))
        .andThen(wait(0.2))
        .andThen(RobotContainer.algae.stopToroC())
      );

      path3.atTime("spike_intake").onTrue(
        log("EVENT(spike_intake)")
        .andThen(RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.ALGAEHIGH))
        .andThen(wait(0.5))
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.REEF))
        .andThen(RobotContainer.algae.startToroC(false))

      );
      path3.atTime("algae_transit").onTrue(
        log("EVENT(algae_transit)")
        .andThen(noop())
      );
      path3.atTime("barge_ready").onTrue(
        log("EVENT(barge_ready)")
        .andThen(RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.BARGE))
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP))
      );
      path3.atTime("barge_score").onTrue(
        log("EVENT(barge_score)")
        .andThen(RobotContainer.algae.startToroC(true))
        .andThen(wait(0.2))
        .andThen(RobotContainer.algae.stopToroC())
      );

      path1.done().onTrue(
        runPath(path2)
      );

      path2.done().onTrue(
        runPath(path3)
      );
      path3.active().onTrue(
        RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.FLOOR)
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(runPath(path1))
      );

      return routine;
    }

    //#region Convenience
    private void seedPose(AutoTrajectory path) {
      try {
        RobotContainer.targeting.resetPose(path.getInitialPose().get());
      } catch(Exception e) {
        log("EXCEPTION! Bad Path Name?");
      };
    }
    //This command is a shortcut for WaitCommand
    private Command wait(double seconds) {
      return new WaitCommand(seconds);
    }
    //This command does nothing
    private Command noop() {
      return Commands.none();
    }
    //This command runs the specified path
    private Command runPath(AutoTrajectory path) {
      return
        log("PATH("+path.getRawTrajectory().name()+")")
        .andThen(path.cmd());

    }
    private Command log(String msg) {
      return NCDebug.Debug.debugC("Auton: "+msg);
    }
    //#endregion
}
