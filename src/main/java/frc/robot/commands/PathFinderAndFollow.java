package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** A command that runs pathfindThenFollowPath based on the current path name. */
public class PathFinderAndFollow extends Command {
  private Command scoreCommand;
  private Command pathRun;
  private String pathName;

  /**
   * Creates a new PathFinderAndFollow command.
   *
   * @param pathName the path name
   */
  public PathFinderAndFollow(String pathName) {
    this.pathName = pathName;
  }

  @Override
  public void initialize() {
    runNewAutonPath();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    scoreCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return pathRun.isFinished();
  }

  /** Runs a new autonomous path based on the current path name. */
  public void runNewAutonPath() {
    PathPlannerPath ampPath = PathPlannerPath.fromPathFile(pathName);
    PathConstraints constraints =
        new PathConstraints(4.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    pathRun = AutoBuilder.pathfindThenFollowPath(ampPath, constraints, 0.0);
    scoreCommand = Commands.sequence(pathRun);
    scoreCommand.schedule();
  }
}