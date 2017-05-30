package org.firstinspires.ftc.teamcode.Sensor_Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by FTCRubies on 11/8/2015.
 */
@Autonomous (name="ODS_Test")
public class ODS_Test extends OpMode {
    OpticalDistanceSensor Optic;

    @Override
    public void init() {
        Optic = hardwareMap.opticalDistanceSensor.get("Optic");

    }

    @Override
    public void loop() {
        double reflectance = Optic.getLightDetected();
        telemetry.addData("Reflectance", reflectance);

    }
}
