package org.firstinspires.ftc.teamcode.Newbi_Code;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by StephanieRamirez on 5/9/17.
 */
@Autonomous (name="beacon_run")
public class beacon_run extends LinearOpMode {
    DcMotor LeftF, LeftB, RightF, RightB;
    ColorSensor Color;
    OpticalDistanceSensor Optic;
    ModernRoboticsI2cGyro G;
    public int gyroDistance;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftF = hardwareMap.dcMotor.get("LF");
        LeftB = hardwareMap.dcMotor.get("LB");
        RightF = hardwareMap.dcMotor.get("RF");
        RightB = hardwareMap.dcMotor.get("RB");
        Color = hardwareMap.colorSensor.get("Color");
        Optic = hardwareMap.opticalDistanceSensor.get("Optic");
        G = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        LeftF.setDirection(DcMotor.Direction.FORWARD);
        LeftB.setDirection(DcMotor.Direction.FORWARD);
        RightB.setDirection(DcMotor.Direction.REVERSE);
        RightF.setDirection(DcMotor.Direction.REVERSE);

        LeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Color.enableLed(false);

        double reflectance = Optic.getRawLightDetected();
        final double WHITE_THRESHOLD;

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        int red = Color.red();
        int blue = Color.blue();

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        G.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && G.isCalibrating())  {
            sleep(50);
            idle();
        }
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();
        WHITE_THRESHOLD = Optic.getRawLightDetected();
        resetDelta();
        FwdG(.5, 8000, 3);
        Turn(.5,456);
        Forward(.5, 3200);
        Turn(.5, 400);
        ForwardLine(.1,.1, 2000, WHITE_THRESHOLD + 0.01);
        Forward(.7,2000);
        ForwardLine(.1,.1, 5000, WHITE_THRESHOLD + 0.01);
       //FwdGLine(.1, .1, 8000, WHITE_THRESHOLD + 0.01, 3);
        // Forward(.5,260);


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

    int gyroDelta() {
        return gyroDistance - G.getIntegratedZValue();
    }

    void resetDelta() {
        gyroDistance = G.getIntegratedZValue();
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

    public void FwdGLine(double powerLeft, double powerRight, int distance, double WHITE_THRESHOLD, int degrees) throws InterruptedException {
        LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        float direction = Math.signum(gyroDelta() - degrees);

        LeftF.setTargetPosition(-distance);
        LeftB.setTargetPosition(-distance);
        RightF.setTargetPosition(-distance);
        RightB.setTargetPosition(-distance);

        LeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftF.setPower(-powerLeft* .3);
        LeftB.setPower(-powerLeft* .3);
        RightF.setPower(-powerRight* .3);
        RightB.setPower(-powerLeft* .3);


        Thread.sleep(500);
        // while (motorLeft.isBusy() && motorRight.isBusy()) ;
        int count = LeftF.getCurrentPosition();
        while ((Math.abs(count) < distance) && opModeIsActive() && (Math.abs(count) < distance / 2 || Optic.getRawLightDetected() < WHITE_THRESHOLD)) {
            direction = gyroDelta() - degrees;
            if (direction < 0) {
                LeftF.setPower(-powerLeft * .5);
                LeftB.setPower(-powerLeft * .5);
                RightF.setPower(-powerRight);
                RightB.setPower(-powerRight);
            } else if (direction > 0) {
                LeftF.setPower(-powerLeft);
                LeftB.setPower(-powerLeft);
                RightF.setPower(-powerRight * .5);
                RightB.setPower(-powerRight* .5);
            } else {
               LeftF.setPower(-powerLeft);
                LeftB.setPower(-powerLeft);
                RightF.setPower(-powerRight);
                RightB.setPower(-powerLeft);
            }

            count = LeftF.getCurrentPosition();
            telemetry.addData("Encoder", Math.abs(count));
            telemetry.addData("Direction", direction);
            telemetry.addData("White_Thresh", WHITE_THRESHOLD);
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

    public void FwdG(double power, int distance, int degrees) throws InterruptedException {
        double turnPwr = 0.7;
        LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        float direction = Math.signum(gyroDelta() - degrees);

        LeftF.setTargetPosition(-distance);
        LeftB.setTargetPosition(-distance);
        RightF.setTargetPosition(-distance);
        RightB.setTargetPosition(-distance);

        LeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftF.setPower(-power * .3);
        LeftB.setPower(-power * .3);
        RightF.setPower(-power * .3);
        RightB.setPower(-power * .3);

        Thread.sleep(500);
        // while (motorLeft.isBusy() && motorRight.isBusy()) ;
        int count = LeftF.getCurrentPosition();
        while ((Math.abs(count) < distance) && opModeIsActive()) {
            direction = gyroDelta() - degrees;
            if (direction < 0) {
                LeftF.setPower(-power * turnPwr);
                LeftB.setPower(-power * turnPwr);
                RightF.setPower(-power);
                RightB.setPower(-power);
            } else if (direction > 0) {
                LeftF.setPower(-power);
                LeftB.setPower(-power);
                RightF.setPower(-power * turnPwr);
                RightB.setPower(-power * turnPwr);
            } else {
                LeftF.setPower(-power);
                LeftB.setPower(-power);
                RightF.setPower(-power);
                RightB.setPower(-power);
            }

            count = LeftF.getCurrentPosition();
            telemetry.addData("Encoder", Math.abs(count));
            telemetry.addData("Direction", direction);
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

   public void gTurn(double power, int degrees) throws InterruptedException {

       LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       LeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       LeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       RightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       RightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       LeftF.setPower(-power);
       LeftB.setPower(-power);
       RightF.setPower(-power);
       RightB.setPower(-power);


       while ((Math.abs(degrees - gyroDelta()) >= 1) && opModeIsActive()) {

           float direction = Math.signum(gyroDelta() - degrees);

           //motorLeft.setPower(-power * direction);
           //motorRight.setPower(power* direction);
           LeftF.setPower(-((Math.abs(degrees - gyroDelta()) > 15) ? power : .05) * direction);
           LeftB.setPower(-((Math.abs(degrees - gyroDelta()) > 15) ? power : .05) * direction);
           RightF.setPower(((Math.abs(degrees - gyroDelta()) > 15) ? power : .05) * direction);
           RightB.setPower(-((Math.abs(degrees - gyroDelta()) > 15) ? power : .05) * direction);

           telemetry.addData("gyrodelta", gyroDelta());
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

       // Thread.sleep(1000);
       //telemetry.addData("gyrodelta", gyroDelta());
       // telemetry.update();
       //Thread.sleep(1000);

   }
}
