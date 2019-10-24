package org.frcteam2910.o2019;

import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.DPadButton;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.o2019.commands.SetArmAngleCommand;

public class OI {
    private final XboxController controller = new XboxController(0);

    public OI() {
        getStrafeAxis().setInverted(true);
        getRotationAxis().setInverted(true);
    }

    public void bindButtons() {
        controller.getDPadButton(DPadButton.Direction.UP)
                .whenPressed(new SetArmAngleCommand(Math.toRadians(100.0)));
        controller.getDPadButton(DPadButton.Direction.RIGHT)
                .whenPressed(new SetArmAngleCommand(Math.toRadians(63.0)));
        controller.getDPadButton(DPadButton.Direction.DOWN)
                .whenPressed(new SetArmAngleCommand(Math.toRadians(2.0)));
    }

    public Axis getForwardAxis() {
        return controller.getLeftYAxis();
    }

    public Axis getStrafeAxis() {
        return controller.getLeftXAxis();
    }

    public Axis getRotationAxis() {
        return controller.getRightXAxis();
    }
}
