package org.firstinspires.ftc.teamcode.Robotics_2016_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by StephanieRamirez on 5/16/17.
 */
@Autonomous(name="Auton_Flick_Move", group ="Tests")
public class Far_Flick extends LinearOpMode {

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

        // motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flick.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        //Move Forward 1 ft to position
        Forward(.3,.3,996);

        //Flick 1  ball
        flick();
        //Servo set up
        ball.setPosition(ballup);
        Thread.sleep(1000);
        ball.setPosition(balldown);
        Thread.sleep(1000);
        //Flick 2 ball
        flick();


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

