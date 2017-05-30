package org.firstinspires.ftc.teamcode.Sensor_Test;

import android.app.Activity;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by StephanieRamirez on 5/30/17.
 */
@Autonomous (name="White_LineTest")
public class Line_Detect extends LinearOpMode {
    DcMotor LeftF, LeftB, RightF, RightB;
    OpticalDistanceSensor Optic;


    @Override
    public void runOpMode() throws InterruptedException {
        LeftF = hardwareMap.dcMotor.get("LF");
        LeftB = hardwareMap.dcMotor.get("LB");
        RightF = hardwareMap.dcMotor.get("RF");
        RightB = hardwareMap.dcMotor.get("RB");
        Optic = hardwareMap.opticalDistanceSensor.get("Optic");


        LeftF.setDirection(DcMotor.Direction.FORWARD);
        LeftB.setDirection(DcMotor.Direction.FORWARD);
        RightB.setDirection(DcMotor.Direction.REVERSE);
        RightF.setDirection(DcMotor.Direction.REVERSE);

        LeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double reflectance = Optic.getRawLightDetected();
        final double WHITE_THRESHOLD;

        waitForStart();
        WHITE_THRESHOLD = Optic.getRawLightDetected();
        ForwardLine(.1, .1, 5000, WHITE_THRESHOLD + 0.01);


    }

    public void Forward(double power, int distance) throws InterruptedException {

        LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        LeftF.setTargetPosition(-distance);
        RightF.setTargetPosition(-distance);
        LeftB.setTargetPosition(-distance);
        RightB.setTargetPosition(-distance);


        LeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LeftF.setPower(-power);
        RightF.setPower(-power);
        LeftB.setPower(-power);
        RightB.setPower(-power);

        int count = LeftF.getCurrentPosition();
        while ((Math.abs(count) < distance) && opModeIsActive()) {
            count = LeftF.getCurrentPosition();
            telemetry.addData("Encoder", Math.abs(count));
            telemetry.addData("Distance", distance);
            telemetry.update();

        }

        LeftF.setPower(0);
        LeftB.setPower(0);
        RightF.setPower(0);
        RightB.setPower(0);

        LeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void Turn(double power, int distance) throws InterruptedException {

        LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        LeftF.setTargetPosition(distance);
        RightF.setTargetPosition(-distance);
        LeftB.setTargetPosition(distance);
        RightB.setTargetPosition(-distance);


        LeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LeftF.setPower(power);
        RightF.setPower(-power);
        LeftB.setPower(power);
        RightB.setPower(-power);

        int count = LeftF.getCurrentPosition();
        while ((Math.abs(count) < distance) && opModeIsActive()) {
            count = LeftF.getCurrentPosition();
            telemetry.addData("Encoder", Math.abs(count));
            telemetry.addData("Distance", distance);
            telemetry.update();

        }

        LeftF.setPower(0);
        LeftB.setPower(0);
        RightF.setPower(0);
        RightB.setPower(0);

        LeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void ForwardLine(double powerLeft, double powerRight, int distance, double WHITE_THRESHOLD) throws InterruptedException {
        LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftF.setTargetPosition(-distance);
        LeftB.setTargetPosition(-distance);
        RightF.setTargetPosition(-distance);
        RightB.setTargetPosition(-distance);

        LeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftF.setPower(-powerLeft);
        LeftB.setPower(-powerLeft);
        RightF.setPower(-powerRight);
        RightB.setPower(-powerLeft);

        Thread.sleep(500);
        // while (LeftF.isBusy() && RightF.isBusy()) ;
        int count = LeftF.getCurrentPosition();
        while ((Math.abs(count) < distance) && opModeIsActive() && (Optic.getRawLightDetected() < WHITE_THRESHOLD)) {
            count = LeftF.getCurrentPosition();
            telemetry.addData("Encoder", Math.abs(count));
            telemetry.addData("Distance", distance);
            telemetry.addData("White_Tape", WHITE_THRESHOLD);
            telemetry.update();
        }


        LeftF.setPower(0);
        LeftB.setPower(0);
        RightF.setPower(0);
        RightB.setPower(0);

        LeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}


