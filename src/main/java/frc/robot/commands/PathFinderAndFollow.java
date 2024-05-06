// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** A command that runs pathfindThenFollowPath based on the current path name. */
public class PathFinderAndFollow extends Command {
  // Command to score
  private Command scoreCommand;
  // Command to run the path
  private Command pathRun;
  // Name of the path to follow
  private String pathName;

  /**
   * Creates a new PathFinderAndFollow command.
   *
   * @param pathName the path name
   */
  public PathFinderAndFollow(String pathName) {
    this.pathName = pathName;
  }

  // Initializes the command and runs a new autonomous path
  @Override
  public void initialize() {
    runNewAutonPath();
  }

  // Ends the command and cancels the score command
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    scoreCommand.cancel();
  }

  // Checks if the path run command is finished
  @Override
  public boolean isFinished() {
    return pathRun.isFinished();
  }

  /**
   * Runs a new autonomous path based on the current path name. It creates a new path from the path
   * file, sets the path constraints, and schedules the path run command.
   */
  public void runNewAutonPath() {
    PathPlannerPath ampPath = PathPlannerPath.fromPathFile(pathName);
    PathConstraints constraints =
        new PathConstraints(4.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    pathRun = AutoBuilder.pathfindThenFollowPath(ampPath, constraints, 0.0);
    scoreCommand = Commands.sequence(pathRun);
    scoreCommand.schedule();
  }
}
