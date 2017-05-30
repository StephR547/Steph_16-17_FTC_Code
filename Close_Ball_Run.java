package org.firstinspires.ftc.teamcode.Robotics_2016_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by StephanieRamirez on 5/16/17.
 */
@Autonomous(name="Auton_Ball_Close", group ="Tests")
public class Close_Ball_Run extends LinearOpMode{

    DcMotor flick, motorLeft, motorRight;
    Servo ball, holder ;

    final double ballup = .9;
    final double balldown = 0;
    final double holderup= 1;
    final double holderdown= 0;


    @Override
    public void runOpMode() throws InterruptedException {

        flick = this.hardwareMap.dcMotor.get("flick");
        motorLeft = this.hardwareMap.dcMotor.get("1");
        motorRight = this.hardwareMap.dcMotor.get("2");
        ball = this.hardwareMap.servo.get("ball");
        holder = this.hardwareMap.servo.get("holder");

        ball.setPosition(balldown);
        holder.setPosition(holderdown);

        //motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flick.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        //Flicker shooting ball
        //Flick 1 ball
        flick.setPower(-.5);
        Thread.sleep(800);
        flick.setPower(0);
        //Servo set up
        ball.setPosition(ballup);
        Thread.sleep(1000);
        ball.setPosition(balldown);
        Thread.sleep(1000);
        //Flick 2 ball
        flick.setPower(-.5);
        Thread.sleep(2000);
        flick.setPower(0);
        // Go Forward to hit ball
        Forward(1, 1, 3000);
        // 3 second delay
        Thread.sleep(3000);
        //Forward to park on platform
        Forward(1,1,900);
    }
    public void Forward(double powerLeft, double powerRight, int distance) throws InterruptedException {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setTargetPosition(-distance);
        motorRight.setTargetPosition(-distance);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setPower(-powerLeft);
        motorRight.setPower(-powerRight);

        // while (motorLeft.isBusy() && motorRight.isBusy()) ;
        int count = motorLeft.getCurrentPosition();
        while ((Math.abs(count) < distance) && opModeIsActive()) {
            count = motorLeft.getCurrentPosition();
            telemetry.addData("Encoder", Math.abs(count));
            telemetry.addData("Distance", distance);
            telemetry.update();
        }


        motorLeft.setPower(0);
        motorRight.setPower(0);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
    public void flick() throws InterruptedException {
        int distance = -2200;
        flick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        flick.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        flick.setTargetPosition(distance);


        flick.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        flick.setPower(-.6);

        // while (motorLeft.isBusy() && motorRight.isBusy()) ;
        int count = flick.getCurrentPosition();
        while ((Math.abs(count) < Math.abs(distance)) && opModeIsActive()) {
            count = flick.getCurrentPosition();
            telemetry.addData("Encoder", Math.abs(count));
            telemetry.addData("Distance", distance);
            telemetry.update();


        }
        flick.setPower(0);

        flick.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        flick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}





