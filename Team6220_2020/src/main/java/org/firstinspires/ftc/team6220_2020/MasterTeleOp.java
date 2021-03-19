package org.firstinspires.ftc.team6220_2020;

public abstract class MasterTeleOp extends MasterOpMode
{
    public void driveMecanumWithJoysticks()
    {
        double turningPower = gamepad1.left_stick_x;
        double driveAngle = Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x);
        double drivePower = Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x);
        driveMecanum(driveAngle, drivePower, turningPower);
    }

    public void driveZiptieWithTrigger()
    {
        //power of motor (if you need to change power universally, makes it easier)
        double power = 0.8;

        //drives collector inwards if dpad up, outwards when down, and no power at all when no input.
        if(gamepad2.dpad_up)
        {
            driveZiptie(power);
        }
        else if(gamepad2.dpad_down)
        {
            driveZiptie(-power);
        }
        else
        {
            driveZiptie(0);
        }

    }
}

