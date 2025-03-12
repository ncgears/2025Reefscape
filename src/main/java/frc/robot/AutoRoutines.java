package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

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

      routine.active().onTrue(
          simplePath.resetOdometry()
              .andThen(simplePath.cmd())
      );
      return routine;
  }
  public AutoRoutine moveOffLine() {
    final AutoRoutine routine = m_factory.newRoutine("MoveOffLine");
    final AutoTrajectory path = routine.trajectory("MoveOffLine");

    routine.active().onTrue(
        path.resetOdometry()
            .andThen(path.cmd())
    );
    return routine;
  }

}
