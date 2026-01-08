//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//
//// this turns 45degrees once the tag is read.
//@TeleOp(name = "Distance Adjust", group = "Sensor")
//public class DistanceAdjustOpMode extends LinearOpMode {
//
//    // --- Hardware Devices ---
//    DistanceSensor distanceSensor;
//    HuskyLens huskyLens;
//    DcMotor leftMotor;
//    DcMotor rightMotor; // Added the right motor back
//
//    // --- Constants for Distance Adjustment ---
//    static final double TARGET_DISTANCE_CM = 30.0;
//    static final double THRESHOLD_CM = 2.0; // Allowed error
//
//    // --- Constants for Robot Turning ---
//    // YOU MUST TUNE THESE VALUES FOR YOUR ROBOT
//    static final double COUNTS_PER_MOTOR_REV = 537.7;    // Example: REV HD Hex Motor
//    static final double WHEEL_DIAMETER_CM = 9.6;         // Example: Standard FTC wheel
//    static final double ROBOT_TRACK_WIDTH_CM = 35.5;     // <<< IMPORTANT: Measure the distance between the center of your left and right wheels
//
//    @Override
//    public void runOpMode() {
//        // --- INITIALIZATION ---
//        try {
//            // Retrieve devices from hardwareMap
//            distanceSensor = hardwareMap.get(DistanceSensor.class, "test_distance");
//            huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");
//            leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
////            rightMotor = hardwareMap.get(DcMotor.class, "right_motor"); // Initialize right motor
//
//            // Set motor directions (one must be reversed for a differential drive)
//            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD); // This might need to be FORWARD
////            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // This might need to be REVERSE
//
//            // Reset encoders and set to run with encoders
//            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            // --- HuskyLens Initialization ---
//            if (!huskyLens.knock()) {
//                telemetry.addData("FATAL", "HuskyLens not responding!");
//                telemetry.update();
//                while(!isStopRequested()) { sleep(50); }
//                return;
//            }
//            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
//
//        } catch (IllegalArgumentException e) {
//            telemetry.addData("FATAL", "Hardware device not found. Check config.");
//            telemetry.addData("Error", e.getMessage());
//            telemetry.update();
//            while(!isStopRequested()) { sleep(50); }
//            return;
//        }
//
//        telemetry.addLine("Initialization Complete. Ready to start.");
//        telemetry.update();
//
//        waitForStart();
//
//        // --- EXECUTION LOOP ---
//        while (opModeIsActive()) {
//            // --- AprilTag Detection ---
//            HuskyLens.Block[] blocks = huskyLens.blocks();
//            telemetry.addData("Tags Visible", blocks.length);
//
//            if (blocks.length > 0) {
//                // An AprilTag is visible!
//                int detectedID = blocks[0].id;
//                telemetry.addData("AprilTag DETECTED!", "ID: " + detectedID);
//                telemetry.addLine("Turning ROBOT 45 degrees...");
//                telemetry.update();
//
//                // Call the new method to turn the entire robot
//                turnRobot(45, 0.4);
//
//                // Stop the OpMode after the turn is complete
//                sleep(2000); // Pause to show the turn is done
//                requestOpModeStop();
//                break; // Exit the loop
//            }
//
//            // --- Distance Adjustment Logic (only runs if no tag is seen) ---
//            double currentDistance = distanceSensor.getDistance(DistanceUnit.CM);
//            telemetry.addData("Distance (cm)", "%.2f", currentDistance);
//
//            if (currentDistance < TARGET_DISTANCE_CM + THRESHOLD_CM) {
//                // Too far, move forward
//                telemetry.addData("Status", "Moving forward");
//                leftMotor.setPower(0.3);
////                rightMotor.setPower(0.3);
//            } else if (currentDistance > TARGET_DISTANCE_CM - THRESHOLD_CM) {
//                // Too close, move backward
//                telemetry.addData("Status", "Moving backward");
//                leftMotor.setPower(-0.3);
////                rightMotor.setPower(-0.3);
//            } else {
//                // Within range, stop motors
//                telemetry.addData("Status", "Holding position");
//                leftMotor.setPower(0.0);
////                rightMotor.setPower(0.0);
//            }
//
//            telemetry.update();
//        }
//    }
//
//    /**
//     * Turns the entire robot by a specific angle using encoders.
//     * @param angle The angle in degrees to turn the robot. Positive is a left turn, negative is a right turn.
//     * @param power The power to use for the turn (0.0 to 1.0).
//     */
//    private void turnRobot(double angle, double power) {
//        if (!opModeIsActive()) return;
//
//        // Reset encoders to ensure a clean start
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // Calculate the distance each wheel needs to travel for the turn
//        double wheelCircumference = Math.PI * WHEEL_DIAMETER_CM;
//        double robotTurnRadius = ROBOT_TRACK_WIDTH_CM / 2.0;
//        double turnDistance = (angle / 360.0) * (2.0 * Math.PI * robotTurnRadius);
//
//        // Calculate the number of encoder counts needed for that distance
//        double countsPerCm = COUNTS_PER_MOTOR_REV / wheelCircumference;
//        int targetTicks = (int) (turnDistance * countsPerCm);
//
//        // For a left turn (positive angle), left motor goes backward, right motor goes forward
//        leftMotor.setTargetPosition(targetTicks);
////        rightMotor.setTargetPosition(-targetTicks);
//
//        // Switch the motors to RUN_TO_POSITION mode
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // Set the power and start the movement
//        leftMotor.setPower(power);
////        rightMotor.setPower(power);
//
//        // Loop until both motors reach their target
//        while (opModeIsActive() && (leftMotor.isBusy() )) {//|| rightMotor.isBusy())) {
//            telemetry.addData("Turning Robot", "Angle: " + angle);
//            telemetry.addData("L Target/Actual", "%d / %d", -targetTicks, leftMotor.getCurrentPosition());
////            telemetry.addData("R Target/Actual", "%d / %d", targetTicks, rightMotor.getCurrentPosition());
//            telemetry.update();
//        }
//
//        // Stop the motors and return to normal encoder mode
//        leftMotor.setPower(0);
////        rightMotor.setPower(0);
//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//}
