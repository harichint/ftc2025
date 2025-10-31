package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name = "Distance Adjust", group = "Linear OpMode")
public class DistanceAdjustOpMode extends LinearOpMode {

    DistanceSensor distanceSensor;
    DcMotor leftMotor;
    DcMotor rightMotor;

    // Set your desired target distance (cm) to maintain
    static final double TARGET_DISTANCE_CM = 30.0;
    static final double THRESHOLD_CM = 2.0; // Allowed error

    @Override
    public void runOpMode() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "test_distance");
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
//        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        waitForStart();

        while (opModeIsActive()) {
            double currentDistance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance (cm)", currentDistance);

            if (currentDistance > TARGET_DISTANCE_CM + THRESHOLD_CM) {
// Too far, move forward
                telemetry.addData("Moving forward", currentDistance);
                leftMotor.setPower(0.3);
//                rightMotor.setPower(0.3);
            } else if (currentDistance < TARGET_DISTANCE_CM - THRESHOLD_CM) {
// Too close, move backward
                telemetry.addData("Moving backward", currentDistance);
                leftMotor.setPower(-0.3);
//                rightMotor.setPower(-0.3);
            } else {
// Within range, stop motors
                telemetry.addData("Stopping motors", currentDistance);
                leftMotor.setPower(0.0);
//                rightMotor.setPower(0.0);
            }

            telemetry.update();
        }
    }
}
