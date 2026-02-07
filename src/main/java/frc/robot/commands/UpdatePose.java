package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

public class UpdatePose extends Command {
    private final Vision vision;

    public UpdatePose(Vision vision) {
        this.vision = vision;
    }

    @Override
    public void execute() {
        vision.addVisionMeasurement();
    }
}
