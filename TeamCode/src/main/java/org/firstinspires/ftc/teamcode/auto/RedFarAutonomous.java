//package org.firstinspires.ftc.teamcode.auto;
//
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import java.util.Arrays;
//
//@Autonomous(name = "Red Far Autonomous")
//public class RedFarAutonomous extends LinearOpMode {
//
//    // --- HARDWARE ---
//    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
//    private HuskyLens huskyLens;
//    private NormalizedColorSensor colorSensor;
//    private CRServo intakeGate;
//    private DcMotor conveyorBelt;
//
//    // --- ROBOT CONSTANTS (Tune These!) ---
//    static final double COUNTS_PER_MOTOR_REV = 537.7;    // For goBILDA 5203-series motor
//    static final double WHEEL_DIAMETER_INCHES = 3.78;
//    static final double ROBOT_TRACK_WIDTH_INCHES = 14.0;   // Distance between left and right wheels
//    static final double LAUNCH_ANGLE_DEGREES = 40.0;
//
//    private static final double SHOOTER_CONVEYOR_POWER = 1.0;
//    private static final double GATE_SERVO_POWER = 1.0;
//
//    // --- SEQUENCE LOGIC ---
//    public enum BallColor { GREEN, PURPLE, UNKNOWN }
//    private final BallColor[] SEQ_1 = {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
//    private final BallColor[] SEQ_2 = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
//    private final BallColor[] SEQ_3 = {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
//    private final BallColor[] SEQ_UNKNOWN = {BallColor.UNKNOWN};
//
//    @Override
//    public void runOpMode() {
//        initializeHardware();
//
//        telemetry.addLine("--- Ready to Run Red Far Autonomous ---");
//        telemetry.update();
//        waitForStart();
//
//        // --- AUTONOMOUS SEQUENCE ---
//
//        // Step 1: Drive forward until we see the white line or a sequence tag.
//        telemetry.addLine("Step 1: Driving to line or tag...");
//        telemetry.update();
//        BallColor[] detectedSequence = driveUntilLineOrTag(0.7);
//
//        // Step 2: If we didn't find a tag while driving, do a stationary scan.
//        if (detectedSequence == null) {
//            telemetry.addLine("Step 2: Reached line, now scanning...");
//            telemetry.update();
//            detectedSequence = readSequenceFromObelisk(3.0); // Scan for 2 seconds
//        }
//
//        telemetry.addData("Sequence Found", Arrays.toString(detectedSequence));
//        telemetry.update();
//        sleep(1000);
//
//        // --- Conditional Logic: Only proceed if a sequence was found ---
//        if (detectedSequence[0] != BallColor.UNKNOWN) {
//            // Step 3: Turn right until the goal tag is visible.
//            telemetry.addLine("Step 3: Turning to find Red Goal...");
//            telemetry.update();
//            turnUntilTagVisible(new int[]{20, 24}, -0.3); // Turn right
//
//            // Step 4: Verify with the color sensor.
//            telemetry.addLine("Step 4: Verifying goal color...");
//            telemetry.update();
//            sleep(500);
//            if (isSeeingRed()) {
//                telemetry.addLine("SUCCESS: Confirmed Red Goal.");
//            } else {
//                telemetry.addLine("WARNING: Red Goal not detected!");
//            }
//            sleep(1500);
//
//            // Step 5: Shoot.
//            runShooter();
//            telemetry.addLine("Step 5: Shooting...");
//            telemetry.update();
//
//            // Step 6: Reposition after shooting.
//            telemetry.addLine("Step 6: Repositioning...");
//            telemetry.update();
//            turnRobot(LAUNCH_ANGLE_DEGREES, 0.5);      // Turn left
//            driveDistance(-10, 0.8);  // Move backward
//        } else {
//            // If the obelisk scan failed, stop here.
//            telemetry.addLine("ERROR: No sequence found. Stopping.");
//            telemetry.update();
//        }
//
//        sleep(3000); // End of OpMode
//    }
//
//    /**
//     * Activates the shooter-specific components (conveyor and gate servo).
//     */
//    public void runShooter() {
//        conveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
//        intakeGate.setPower(GATE_SERVO_POWER);
//    }
//
//    //----------------------------------------------------------------------------------------------
//    // INITIALIZATION & HELPER METHODS
//    //----------------------------------------------------------------------------------------------
//
//    /** Initializes all hardware and sets motor directions. */
//    private void initializeHardware() {
//        // Drivetrain
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
//
//        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        intakeGate = hardwareMap.get(CRServo.class, "intake_gate");
//        conveyorBelt = hardwareMap.get(DcMotor.class, "conveyor_belt");
//
//        // --- Set Motor & Servo Directions ---
//        intakeGate.setDirection(DcMotorSimple.Direction.FORWARD);   // Spins to help feed conveyor
//        conveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD); // Spins to push balls up/out
//
//        // Sensors
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
//        colorSensor.setGain(10f);
//
//        huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");
//        if (!huskyLens.knock()) {
//            telemetry.addData("FATAL", "HuskyLens not responding!");
//        }
//        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
//    }
//
//    /** Drives forward while simultaneously checking for a white line or a sequence tag. */
//    private BallColor[] driveUntilLineOrTag(double power) {
//        setDrivePower(power);
//        while (opModeIsActive()) {
//            // Check for sequence tags first
//            HuskyLens.Block[] blocks = huskyLens.blocks();
//            if (blocks.length > 0) {
//                for (HuskyLens.Block tag : blocks) {
//                    if (tag.id == 1) { setDrivePower(0); return SEQ_1; }
//                    if (tag.id == 2) { setDrivePower(0); return SEQ_2; }
//                    if (tag.id == 3) { setDrivePower(0); return SEQ_3; }
//                }
//            }
//            // If no tag, check for the white line
//            if (isSeeingWhite()) {
//                setDrivePower(0);
//                return null; // Stopped at line, no tag found
//            }
//            telemetry.addData("Status", "Driving until line or tag...");
//            telemetry.update();
//        }
//        setDrivePower(0);
//        return null;
//    }
//
//    /** Performs a stationary scan for a sequence tag for a given duration. */
//    private BallColor[] readSequenceFromObelisk(double scanSeconds) {
//        ElapsedTime scanTimer = new ElapsedTime();
//        while(opModeIsActive() && scanTimer.seconds() < scanSeconds) {
//            HuskyLens.Block[] blocks = huskyLens.blocks();
//            if (blocks.length > 0) {
//                for (HuskyLens.Block tag : blocks) {
//                    if (tag.id == 1) return SEQ_1;
//                    if (tag.id == 2) return SEQ_2;
//                    if (tag.id == 3) return SEQ_3;
//                }
//            }
//            sleep(50);
//        }
//        return SEQ_UNKNOWN;
//    }
//
//    /** Turns the robot until a target AprilTag ID is visible or a timeout occurs. */
//    private void turnUntilTagVisible(int[] targetIds, double power) {
//        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftFrontDrive.setPower(power);
//        leftBackDrive.setPower(power);
//        rightFrontDrive.setPower(-power);
//        rightBackDrive.setPower(-power);
//
//        ElapsedTime timeoutTimer = new ElapsedTime();
//        while (opModeIsActive() && timeoutTimer.seconds() < 5.0) {
//            for (HuskyLens.Block tag : huskyLens.blocks()) {
//                for (int targetId : targetIds) {
//                    if (tag.id == targetId) {
//                        setDrivePower(0);
//                        telemetry.addData("SUCCESS", "Found Tag ID: " + tag.id);
//                        return; // Exit method
//                    }
//                }
//            }
//            sleep(20);
//        }
//        setDrivePower(0); // Stop if timed out
//        telemetry.addLine("WARNING: Turn timed out without finding tag.");
//    }
//
//    /** Drives the robot a specific distance using encoders. */
//    private void driveDistance(double distanceInches, double power) {
//        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
//        int targetTicks = (int) (distanceInches * countsPerInch);
//
//        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setTargetPosition(targetTicks, targetTicks, targetTicks, targetTicks);
//        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        setDrivePower(power);
//        while (opModeIsActive() && leftFrontDrive.isBusy()) {
//            telemetry.addData("Driving", "%.2f inches", ticksToInches(leftFrontDrive.getCurrentPosition()));
//            telemetry.update();
//        }
//        setDrivePower(0);
//    }
//
//    /** Turns the robot a specific angle using encoders. */
//    private void turnRobot(double angle, double power) {
//        double turnDistance = (angle / 360.0) * (Math.PI * ROBOT_TRACK_WIDTH_INCHES);
//        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
//        int targetTicks = (int) (turnDistance * countsPerInch);
//
//        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setTargetPosition(targetTicks, -targetTicks, targetTicks, -targetTicks);
//        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        setDrivePower(power);
//        while (opModeIsActive() && leftFrontDrive.isBusy()) {
//            telemetry.addData("Turning", "%.1f degrees", ticksToInches(leftFrontDrive.getCurrentPosition()) / (Math.PI * ROBOT_TRACK_WIDTH_INCHES) * 360.0);
//            telemetry.update();
//        }
//        setDrivePower(0);
//    }
//
//    /** Checks for a strong red signal from the color sensor. */
//    private boolean isSeeingRed() {
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//        return (colors.alpha > 0.05f) && (colors.red > colors.blue * 1.5) && (colors.red > colors.green * 1.5);
//    }
//
//    /** Checks for a bright white signal from the color sensor. */
//    private boolean isSeeingWhite() {
//        return colorSensor.getNormalizedColors().alpha > 0.6f; // Tune this brightness threshold
//    }
//
//    // --- LOW-LEVEL MOTOR CONTROL ---
//    private void setDrivePower(double power) {
//        leftFrontDrive.setPower(power);
//        rightFrontDrive.setPower(power);
//        leftBackDrive.setPower(power);
//        rightBackDrive.setPower(power);
//    }
//    private void setDriveRunMode(DcMotor.RunMode mode) {
//        leftFrontDrive.setMode(mode);
//        rightFrontDrive.setMode(mode);
//        leftBackDrive.setMode(mode);
//        rightBackDrive.setMode(mode);
//    }
//    private void setTargetPosition(int lf, int rf, int lb, int rb) {
//        leftFrontDrive.setTargetPosition(lf);
//        rightFrontDrive.setTargetPosition(rf);
//        leftBackDrive.setTargetPosition(lb);
//        rightBackDrive.setTargetPosition(rb);
//    }
//    private double ticksToInches(int ticks) {
//        return (ticks / COUNTS_PER_MOTOR_REV) * (Math.PI * WHEEL_DIAMETER_INCHES);
//    }
//
//}
