package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.NCDebug;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
        ConfigureGlobalBindings();
    }

    public AutoRoutine doNothingAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Do Nothing");
        routine.active().onTrue(
          noop()
        );
        return routine;
    }

    public AutoRoutine sCmoveOffLine() {
      final AutoRoutine routine = m_factory.newRoutine("sCMoveOffLine");
      final AutoTrajectory path1 = routine.trajectory("sC-MoveOffLine");

      path1.done().onTrue(log("Routine Complete!"));

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
            .andThen(runPath(path1))
      );
      return routine;
    }

    public AutoRoutine sCScoreAlgae() {
      final AutoRoutine routine = m_factory.newRoutine("sCScoreAlgae");
      final AutoTrajectory path1 = routine.trajectory("sC-rBC");

      path1.done().onTrue(log("Routine Complete!"));

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
            .andThen(runPath(path1))
      );
      return routine;
    }

    public AutoRoutine sLLmoveOffLine() {
      final AutoRoutine routine = m_factory.newRoutine("sLLMoveOffLine");
      final AutoTrajectory path1 = routine.trajectory("sLL-MoveOffLine");
    
      path1.done().onTrue(log("Routine Complete!"));

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
            .andThen(runPath(path1))
      );
      return routine;
    }

    public AutoRoutine sRRmoveOffLine() {
      final AutoRoutine routine = m_factory.newRoutine("sRRMoveOffLine");
      final AutoTrajectory path1 = routine.trajectory("sRR-MoveOffLine");
    
      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
            .andThen(runPath(path1))
      );
      return routine;
    }

    public AutoRoutine leftAlgaeDouble() {
      final AutoRoutine routine = m_factory.newRoutine("LeftAlgaeDouble");
      final AutoTrajectory path1 = routine.trajectory("sLCb-rBL_c");
      final AutoTrajectory path2 = routine.trajectory("rBL_c-bC");
      final AutoTrajectory path3 = routine.trajectory("bC-aL-bC");
    
      path1.done().onTrue(runPath(path2));
      path2.done().onTrue(runPath(path3));
      path3.done().onTrue(log("Routine Complete!"));

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(runPath(path1))
      );

      return routine;
    }

    public AutoRoutine left4Coral() {
      final AutoRoutine routine = m_factory.newRoutine("Left4Coral");
      final AutoTrajectory path1 = routine.trajectory("sLR-rBL_r");
      final AutoTrajectory path2 = routine.trajectory("rBL_r-hL");
      final AutoTrajectory path3 = routine.trajectory("hL-rFL_l");
      final AutoTrajectory path4 = routine.trajectory("rFL_l-hL");
      final AutoTrajectory path5 = routine.trajectory("hL-rFL_r");
      final AutoTrajectory path6 = routine.trajectory("rFL_r-hL");
      final AutoTrajectory path7 = routine.trajectory("hL-rFC_l");
    
      path1.done().onTrue(runPath(path2));
      path2.done().onTrue(runPath(path3));
      path3.done().onTrue(runPath(path4));
      path4.done().onTrue(runPath(path5));
      path5.done().onTrue(runPath(path6));
      path6.done().onTrue(runPath(path7));
      path7.done().onTrue(log("Routine Complete!"));

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(runPath(path1))
      );

      return routine;
    }

    //#region Global Bindings
    /** This method binds event names to their commands */
    private void ConfigureGlobalBindings() {
      m_factory
        .bind("readyL2",log("EVENT(readyL4)").andThen(ReadyL2()))
        .bind("readyL3",log("EVENT(readyL4)").andThen(ReadyL3()))
        .bind("readyL4",log("EVENT(readyL4)").andThen(ReadyL4()))
        .bind("transit",log("EVENT(transit)").andThen(Transit()))
        .bind("intakeCoral",log("EVENT(intakeCoral)").andThen(IntakeCoral()))
        .bind("scoreCoral",log("EVENT(scoreCoral)").andThen(ScoreCoral()))
        .bind("readyBarge",log("EVENT(readyBarge)").andThen(ReadyBarge()))
        .bind("intakeLow",log("EVENT(intakeLow)").andThen(IntakeAlgaeLow()))
        .bind("intakeHigh",log("EVENT(intakeHigh)").andThen(IntakeAlgaeHigh()))
        .bind("intakeSpike",log("EVENT(intakeSpike)").andThen(IntakeAlgaeSpike()))
        .bind("scoreAlgae",log("EVENT(scoreAlgae)").andThen(ScoreAlgae()));
    }
    //#endregion Global Bindings

    //#region AutoCommands
    private Command ReadyL2() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.L2);
    }
    private Command ReadyL3() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.L3);
    }
    private Command ReadyL4() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.L4);
    }
    private Command ScoreCoral() {
      return RobotContainer.elevator.ScoreC()
        .until(RobotContainer.elevator::isAtTarget)
        .andThen(wait(0.2))
        .andThen(RobotContainer.coral.CoralPositionC(CoralSubsystem.Position.SCORE));
    }
    private Command IntakeCoral() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.HP)
        .andThen(RobotContainer.coral.CoralPositionC(CoralSubsystem.Position.OUT))
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP));
    }
    private Command ReadyBarge() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.BARGE)
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP));
    }
    private Command IntakeAlgaeLow() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.ALGAELOW)
          .until(RobotContainer.elevator::isAtTarget)
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.REEF))
        .andThen(RobotContainer.algae.startToroC(false))
        .andThen(wait(0.5))
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP));
    }
    private Command IntakeAlgaeHigh() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.ALGAEHIGH)
          .until(RobotContainer.elevator::isAtTarget)
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.REEF))
        .andThen(RobotContainer.algae.startToroC(false))
        .andThen(wait(0.5))
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP));
    }
    private Command IntakeAlgaeSpike() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.FLOOR)
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.FLOOR))
        .andThen(RobotContainer.algae.startToroC(false));
    }
    private Command ScoreAlgae() {
      return RobotContainer.algae.startToroC(true)
      .andThen(wait(0.2))
      .andThen(RobotContainer.algae.stopToroC());
    }
    private Command Transit() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.HP)
        .andThen(RobotContainer.coral.CoralPositionC(CoralSubsystem.Position.SCORE));
    }
    //#endregion AutoCommands

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
