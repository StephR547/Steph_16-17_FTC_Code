package org.firstinspires.ftc.teamcode.Robotics_2016_2017;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by StephanieRamirez on 5/16/17.
 */
@Autonomous(name="Blue_Test")
public class Blue_Auton extends LinearOpMode {

    DcMotor motorLeft, motorRight, pusher, flick;
    ColorSensor colorSensor;;
    OpticalDistanceSensor odsfront;
    Servo ball;
    ModernRoboticsI2cGyro G;
    public int gyroDistance;

    final double ballup = .7;
    final double balldown = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft = this.hardwareMap.dcMotor.get("1");
        motorRight = this.hardwareMap.dcMotor.get("2");
        pusher = this.hardwareMap.dcMotor.get("pusher");
        flick = this.hardwareMap.dcMotor.get("flick");
        ball = this.hardwareMap.servo.get("ball");
        colorSensor = this.hardwareMap.colorSensor.get("color");
        G = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        odsfront = this.hardwareMap.opticalDistanceSensor.get("front_ods");

        colorSensor.enableLed(false);
        ball.setPosition(balldown);


        double reflectance = odsfront.getRawLightDetected();
        final double WHITE_THRESHOLD;


        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        int red = colorSensor.red();
        int blue = colorSensor.blue();


        //motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flick.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
        WHITE_THRESHOLD = odsfront.getRawLightDetected();
        resetDelta();
        //Flick 1  ball
        flick();
        //Servo set up
        ball.setPosition(ballup);
        Thread.sleep(1000);
        ball.setPosition(balldown);
        Thread.sleep(1000);
        //Flick 2 ball
        flick();
        //Forward to move away from Back wall
        //FwdG(.4,7400,90);
        //Thread.sleep(20000);
        Forward(.4,.27,6858);
        Forward(.3,.3,742);
        //Back
        //Back(.2,516);
        //GTurn Right
        gTurn(.1,3);
        BackLine(.5,.5,300,WHITE_THRESHOLD + 8.1);
        FwdGLine(.1,.1,800,WHITE_THRESHOLD + 0.1,-3);
        Forward(.1,.1,183);

        //Color Sensor values
        red = colorSensor.red();// + colorSensor2.red();
        blue = colorSensor.blue();// + colorSensor2.blue();
        if (red > blue) {
            telemetry.addData("STATE (R): ", "Red");
            Back(.1,60);
            Pusher(.4, -2580);
            Pusher(.4, 2580);

        } else if (red <= blue) {
            telemetry.addData("STATE (B): ", "Blue");
            Back(.2,300);
            Pusher(.4, -2580);
            Pusher(.4, 2580);
        }
        //red2 = colorSensor2.red();
        // blue2 = colorSensor2.blue();
      /* if (red2 > blue2) {
            telemetry.addData("STATE (R): ", "Red2");
        } else if (red2 < blue2) {
            telemetry.addData("STATE (B): ", "Blue2");
            Thread.sleep(5000);
            Back(.2, .2, 287);
            Forward(.5, 350);

        }*/
        telemetry.addData("STATE (R): ", red);
        telemetry.addData("STATE (B): ", blue);
        // telemetry.addData("STATE (R2): ", red2);
        // telemetry.addData("STATE (B2): ", blue2);
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
        telemetry.update();
        //telemetry.update();
        // gTurn(.1, 0);
        telemetry.addData("gyrodelta", gyroDelta());
        telemetry.update();

        // Forward(.2, 1000);
        FwdGLine(.4,.4,7000,WHITE_THRESHOLD + 0.1, -5);
        BackLine(.1,.1,300,WHITE_THRESHOLD + 0.1);
        //Back Up To Second Beacon
        //ForwardLine(.3, .3, 4000, WHITE_THRESHOLD + 0.1);
        // telemetry.addData("front_ods: ", odsfront.getRawLightDetected());
        // telemetry.update();

        //Color Sensor values
        red = colorSensor.red();
        blue = colorSensor.blue();
        if (red > blue) {
            telemetry.addData("STATE (R): ", "Red");
            Pusher(.4, -2500);
            Pusher(.4, 2500);
        } else if (red <= blue) {
            telemetry.addData("STATE (B): ", "Blue");
            Back(.2,500);
            Pusher(.4, -2500);
            Pusher(.4, 2500);
        }
        //red2 = colorSensor2.red();
        // blue2 = colorSensor2.blue();
      /*  if (red2 > blue2) {
            telemetry.addData("STATE (R): ", "Red2");
        } else if (red2 < blue2) {
            telemetry.addData("STATE (B): ", "Blue2");
            Thread.sleep(5000);
            Back(.2, .2, 287);
            Forward(.5, 350);

        }*/
        //telemetry.addData("STATE (R): ", red);
        // telemetry.addData("STATE (B): ", blue);
        // telemetry.addData("STATE (R2): ", red2);
        // telemetry.addData("STATE (B2): ", blue2);
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
        telemetry.update();
        //telemetry.update();


        //}
    }


    public void Back(double power, int distance) throws InterruptedException {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setTargetPosition(distance);
        motorRight.setTargetPosition(distance);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setPower(power);
        motorRight.setPower(-power);


        // while (motorLeft.isBusy() && motorRight.isBusy()) {
        // Wait for both motors to finish
        // }
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

    public void BackLine(double powerLeft, double powerRight, int distance, double WHITE_THRESHOLD) throws InterruptedException {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setTargetPosition(distance);
        motorRight.setTargetPosition(distance);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setPower(powerLeft);
        motorRight.setPower(-powerRight);


        // while (motorLeft.isBusy() && motorRight.isBusy()) {
        // Wait for both motors to finish
        // }
        int count = motorLeft.getCurrentPosition();
        while ((Math.abs(count) < distance) && opModeIsActive() && (odsfront.getRawLightDetected() < WHITE_THRESHOLD)) {
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

    public void TurnLeft(double powerLeft, double powerRight, int distance) throws InterruptedException {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setTargetPosition(-distance);
        motorRight.setTargetPosition(distance);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setPower(-powerLeft);
        motorRight.setPower(powerRight);

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

    public void TurnRight(double powerLeft, double powerRight, int distance) throws InterruptedException {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setTargetPosition(distance);
        motorRight.setTargetPosition(-distance);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setPower(powerLeft);
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

    public void ForwardLine(double powerLeft, double powerRight, int distance, double WHITE_THRESHOLD) throws InterruptedException {
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

        Thread.sleep(500);
        // while (motorLeft.isBusy() && motorRight.isBusy()) ;
        int count = motorLeft.getCurrentPosition();
        while ((Math.abs(count) < distance) && opModeIsActive() && (odsfront.getRawLightDetected() < WHITE_THRESHOLD)) {
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

    public void Forward(double powerR, double powerL, int distance) throws InterruptedException {

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setTargetPosition(-distance);
        motorRight.setTargetPosition(-distance);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setPower(-powerL);
        motorRight.setPower(-powerR);

        // while (motorLeft.isBusy() && motorRight.isBusy())
        int count = motorLeft.getCurrentPosition();
        while ((Math.abs(motorLeft.getCurrentPosition()) < distance) && (Math.abs(motorRight.getCurrentPosition()) < distance) && opModeIsActive()) {
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

    int gyroDelta() {
        return gyroDistance - G.getIntegratedZValue();
    }

    void resetDelta() {
        gyroDistance = G.getIntegratedZValue();
    }

    /*   public void gTurn(int degrees, double power) throws InterruptedException {
           resetDelta();               //Reset Gyro
           float direction = Math.signum(degrees); //get +/- sign of target
           //move in the right direction
           while ( Math.abs(gyroDelta()) < Math.abs(degrees)){
               gLeft(-direction * power, direction * power);
           }
           Forward(0, 0);
           telemetry.addData("Gyro", gyroDelta());
       }
   */
    public void gTurn(double power, int degrees) throws InterruptedException {

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while ((Math.abs(degrees - gyroDelta()) >= 1) && opModeIsActive()) {

            float direction = Math.signum(gyroDelta() - degrees);

            //motorLeft.setPower(-power * direction);
            //motorRight.setPower(power* direction);
            motorLeft.setPower(-((Math.abs(degrees - gyroDelta()) > 15) ? power : .05) * direction);
            motorRight.setPower(((Math.abs(degrees - gyroDelta()) > 15) ? power : .05) * direction);

            telemetry.addData("gyrodelta", gyroDelta());
            telemetry.update();
        }
        motorLeft.setPower(0);
        motorRight.setPower(0);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Thread.sleep(1000);
        //telemetry.addData("gyrodelta", gyroDelta());
        // telemetry.update();
        //Thread.sleep(1000);

    }
    public void FwdGLine(double powerLeft, double powerRight, int distance, double WHITE_THRESHOLD, int degrees) throws InterruptedException {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        float direction = Math.signum(gyroDelta() - degrees);

        motorLeft.setTargetPosition(-distance);
        motorRight.setTargetPosition(-distance);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setPower(-powerLeft *.3);
        motorRight.setPower(-powerRight *.3);
        Thread.sleep(500);
        // while (motorLeft.isBusy() && motorRight.isBusy()) ;
        int count = motorLeft.getCurrentPosition();
        while ((Math.abs(count) < distance) && opModeIsActive() && (Math.abs(count) < distance/2 || odsfront.getRawLightDetected() < WHITE_THRESHOLD)) {
            direction = gyroDelta() - degrees;
            if (direction < 0){
                motorLeft.setPower(-powerLeft *.7);
                motorRight.setPower(-powerRight);
            }
            else if (direction > 0){
                motorLeft.setPower(-powerLeft);
                motorRight.setPower(-powerRight *.7);
            }
            else {
                motorLeft.setPower(-powerLeft);
                motorRight.setPower(-powerRight);
            }

            count = motorLeft.getCurrentPosition();
            telemetry.addData("Encoder", Math.abs(count));
            telemetry.addData("Direction", direction);
            telemetry.update();
        }


        motorLeft.setPower(0);
        motorRight.setPower(0);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void FwdG(double power, int distance, int degrees) throws InterruptedException {
        double turnPwr = 0.7;
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        float direction = Math.signum(gyroDelta() - degrees);

        motorLeft.setTargetPosition(-distance);
        motorRight.setTargetPosition(-distance);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setPower(-power *.3);
        motorRight.setPower(-power *.3);
        Thread.sleep(500);
        // while (motorLeft.isBusy() && motorRight.isBusy()) ;
        int count = motorLeft.getCurrentPosition();
        while ((Math.abs(count) < distance) && opModeIsActive()) {
            direction = gyroDelta() - degrees;
            if (direction < 0){
                motorLeft.setPower(-power * turnPwr);
                motorRight.setPower(-power);
            }
            else if (direction > 0){
                motorLeft.setPower(-power);
                motorRight.setPower(-power * turnPwr);
            }
            else {
                motorLeft.setPower(-power);
                motorRight.setPower(-power);
            }

            count = motorLeft.getCurrentPosition();
            telemetry.addData("Encoder", Math.abs(count));
            telemetry.addData("Direction", direction);
            telemetry.update();
        }


        motorLeft.setPower(0);
        motorRight.setPower(0);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void Pusher(double power, int distance) throws InterruptedException {

        pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pusher.setTargetPosition(distance);

        pusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pusher.setPower(power);


        // while (motorLeft.isBusy() && motorRight.isBusy()) {
        // Wait for both motors to finish
        // }
        int count = pusher.getCurrentPosition();
        while ((Math.abs(count) < (Math.abs(distance)) && opModeIsActive())) {
            count = pusher.getCurrentPosition();
            telemetry.addData("Encoder", Math.abs(count));
            telemetry.addData("Distance", distance);
            telemetry.update();
        }

        pusher.setPower(0);

        pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void PusherB(double power, int distance) throws InterruptedException {

        pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pusher.setTargetPosition(distance);

        pusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pusher.setPower(-power);


        // while (motorLeft.isBusy() && motorRight.isBusy()) {
        // Wait for both motors to finish
        // }
        int count = pusher.getCurrentPosition();
        while ((Math.abs(count) < distance) && opModeIsActive()) {
            count = pusher.getCurrentPosition();
            telemetry.addData("Encoder", Math.abs(count));
            telemetry.addData("Distance", distance);
            telemetry.update();
        }

        pusher.setPower(0);

        pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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


