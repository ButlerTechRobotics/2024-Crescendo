// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.gamepieceVision;

import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;

public class GamepieceVisionIOPhoton implements GamepieceVisionIO {
  private final PhotonCamera camera;

  public GamepieceVisionIOPhoton(String identifier) {
    camera = new PhotonCamera(identifier);
  }

  @Override
  public void updateInputs(GamepieceVisionIOInputs inputs) {
    inputs.hasTarget = camera.getLatestResult().hasTargets();
    inputs.timestamp = Timer.getFPGATimestamp();

    if (inputs.hasTarget == true) {
      inputs.tx = camera.getLatestResult().getBestTarget().getYaw();
      inputs.ty = camera.getLatestResult().getBestTarget().getPitch();
    }

    // var hwQueue = hwSub.readQueue();
    // if (hwQueue.length > 0) {
    // var last = hwQueue[hwQueue.length - 1];
    // if (last.value.length != 4) {
    // return;
    // }

    inputs.latency = camera.getLatestResult().getLatencyMillis();
    inputs.lastFPSTimestamp = Timer.getFPGATimestamp();
  }
}
