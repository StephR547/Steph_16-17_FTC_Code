package org.firstinspires.ftc.teamcode.Robotics_2016_2017;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by StephanieRamirez on 5/16/17.
 */
@TeleOp(name = "Robot_Drive")
public class Robot_Drive extends LinearOpMode{
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor pull = null;
    public DcMotor flick = null;
    public DcMotor pusher = null;
    public Servo holder = null;
    public Servo ball = null;
    public TouchSensor touch= null;

    ColorSensor colorSensor;


    HardwareMap hwMap = null;

    public static final double holderDown = 0;
    public static final double holderUp = 1;
    public static final double balldown = 0;
    public static final double ballup = .7;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hwMap = hardwareMap;
        leftMotor = hwMap.dcMotor.get("1");
        rightMotor = hwMap.dcMotor.get("2");
        pull = hwMap.dcMotor.get("pulley");
        flick = hwMap.dcMotor.get("flick");
        pusher = hwMap.dcMotor.get("pusher");
        holder = hwMap.servo.get("holder");
        ball = hwMap.servo.get("ball");
        touch = hwMap.touchSensor.get("touch");

        colorSensor = hardwareMap.colorSensor.get("color");

        boolean fudgy = touch.isPressed();

        holder.setPosition(holderDown);
        ball.setPosition(balldown);



        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        int x = 1;


        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();

            if (touch.isPressed())

            {
                leftMotor.setPower(.5);
                rightMotor.setPower(.5);

            }
            else

            {

                leftMotor.setPower(-gamepad1.left_stick_y * (.75 + .25 * x));
                rightMotor.setPower(-gamepad1.right_stick_y * (.75 + .25 * x));

            }

            if (gamepad1.x) {
                x = -1 * x;
                while (gamepad1.x) {
                }
            }

            flick.setPower((-gamepad2.left_stick_y) / 3);

            if (gamepad2.x == true) {
                pull.setPower(1);
            } else if (gamepad2.a)
                pull.setPower(-1);
            else
                pull.setPower(0);

            if (gamepad2.right_bumper) {
                holder.setPosition(holderUp);
            }else holder.setPosition(holderDown);

            if (gamepad2.dpad_left){
                pusher.setPower(1);
            }else if (gamepad2.dpad_right)
                pusher.setPower(-1);
            else
                pusher.setPower(0);





            float hsvValues[] = {0F, 0F, 0F};
            final float values[] = hsvValues;
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Blue ", colorSensor.blue());



            idle();


        }


    }
}


