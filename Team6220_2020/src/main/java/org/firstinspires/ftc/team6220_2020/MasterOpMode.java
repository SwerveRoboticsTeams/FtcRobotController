package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class MasterOpMode extends LinearOpMode
{
    //Drive motors
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;

    //Misc. Motors
    DcMotor motorZiptie;

    //Other Devices

    public void Initialize(){
        //Initialize

            //Drive motors
        motorFrontLeft = hardwareMap.dcMotor.get("motorFL");
        motorFrontRight = hardwareMap.dcMotor.get("motorFR");
        motorBackLeft = hardwareMap.dcMotor.get("motorBL");
        motorBackRight = hardwareMap.dcMotor.get("motorBR");

            //Misc. motors
        motorZiptie = hardwareMap.dcMotor.get("motorZiptie");

        //Resets drive motors' encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Starts misc. motors' - No encoder
        motorZiptie.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveMecanum(double driveAngle, double drivePower, double w)
    {
        double x = drivePower * Math.cos(driveAngle);
        double y = drivePower * Math.sin(driveAngle);

        double motorFLPower = x + y + -w;
        double motorFRPower = x + -y + -w;
        double motorBLPower = -x + y + -w;
        double motorBRPower = -x + -y + -w;

        double scaleFactor = Math.max(Math.max(motorFLPower, motorFRPower), Math.max(motorBLPower, motorBRPower));

        if(scaleFactor > 1){
            motorFrontLeft.setPower(motorFLPower / scaleFactor);
            motorFrontRight.setPower(motorFRPower / scaleFactor);
            motorBackLeft.setPower(motorBLPower / scaleFactor);
            motorBackRight.setPower(motorBRPower / scaleFactor);
        } else {
            motorFrontLeft.setPower(motorFLPower);
            motorFrontRight.setPower(motorFRPower);
            motorBackLeft.setPower(motorBLPower);
            motorBackRight.setPower(motorBRPower);
        }
    }

    public void driveZiptie(double power)
    {
        motorZiptie.setPower(power);
    }
}