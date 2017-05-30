package org.firstinspires.ftc.teamcode.Newbi_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by StephanieRamirez on 5/2/17.
 */
@TeleOp(name = "ntt")
public class nubii_teleop_test extends LinearOpMode {
    public DcMotor LeftF = null;
    public DcMotor LeftB = null;
    public DcMotor RightF = null;
    public DcMotor RightB = null;

    HardwareMap hwMap = null;

    @Override

    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hwMap = hardwareMap;

        LeftF = hwMap.dcMotor.get("LF");
        LeftB = hwMap.dcMotor.get("LB");
        RightF = hwMap.dcMotor.get("RF");
        RightB = hwMap.dcMotor.get("RB");

        LeftF.setDirection(DcMotor.Direction.FORWARD);
        LeftB.setDirection(DcMotor.Direction.FORWARD);
        RightB.setDirection(DcMotor.Direction.REVERSE);
        RightF.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();

            float leftpower = -gamepad1.left_stick_y;
            float rightpower = -gamepad1.right_stick_y;

            LeftF.setPower(leftpower);
            LeftB.setPower(leftpower );

            RightF.setPower(rightpower );
            RightB.setPower(rightpower);

            telemetry.addData("Left", leftpower);
            telemetry.addData("Right", rightpower);

            telemetry.update();


            idle();
        }


    }
}
