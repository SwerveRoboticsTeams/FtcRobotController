package org.firstinspires.ftc.team8923_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous (name = "AutoTest")
@Disabled
public class AutoTest extends MasterAutonomous{
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        while(opModeIsActive()){


            moveAuto(0, 5, 20, 10);
            //imuPivot();




            //grab wobblegoal

            //move forward



            //drop wobblegoal

            //move backward

            //park


        }

    }

}
