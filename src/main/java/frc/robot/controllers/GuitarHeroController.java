package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;

public class GuitarHeroController extends GenericHID
{
    public GuitarHeroController(final int port) {
        super(port);
    }

    public boolean getGreenButton() {
        return getRawButton(1);
    }
    public boolean getRedButton() {
        return getRawButton(2);
    }

    public boolean getYellowButton() {
        return getRawButton(3);
    }

    public boolean getBlueButton() {
        return getRawButton(4);
    }

    public boolean getOrangeButton() {
        return getRawButton(5);
    }

    public boolean getStrumUp() {
        return getRawButton(6);
    }

    public boolean getStartButton() {
        return getRawButton(7);
    }

    public boolean getMinusButton() {
        return getRawButton(8);
    }

    public boolean getStrumDown() {
        return getRawButton(9);
    }

    public double getJoystickX() {
        return getRawAxis(0);
    }

    public double getJoystickY() {
        return getRawAxis(1);
    }

    public double getFiddleStick() {
        return getRawAxis(2);
    }
}
