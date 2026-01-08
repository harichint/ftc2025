//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//
//@TeleOp(name="Distance Sensor Example", group="Sensor")
//public class DistanceSensorExample extends LinearOpMode {
//
//    DistanceSensor test_distance;
//
//    @Override
//    public void runOpMode() {
//// Map the distance sensor
//        test_distance = hardwareMap.get(DistanceSensor.class, "test_distance");
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//// Measure distance in centimeters
//            double distance = test_distance.getDistance(DistanceUnit.CM);
//
//// Send distance to telemetry
//            telemetry.addData("Distance (cm)", distance);
//            telemetry.update();
//        }
//    }
//}
