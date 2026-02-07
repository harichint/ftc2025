package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name = "Far Right No Further Auto")
public class FarRightNoFurtherAuto extends LinearOpMode {

    // --- HARDWARE ---
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
//    private HuskyLens huskyLens;
    private NormalizedColorSensor colorSensor;
     private DcMotor intakeRoller;
    // Channel 1
    private CRServo rightIntakeGate; //servo
    private DcMotor rightConveyorBelt;//shooter
    //Channel 2
    private CRServo leftIntakeGate;
    private DcMotor leftConveyorBelt;

    // --- ROBOT CONSTANTS (Tune These!) ---
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // For goBILDA 5203-series motor
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double ROBOT_TRACK_WIDTH_INCHES = 14.0;   // Distance between left and right wheels
    static final double LAUNCH_ANGLE_DEGREES = 60.0;
    private static final double SHOOTER_CONVEYOR_POWER = 1.0;
    private static final double GATE_SERVO_POWER = 1.0;

    // --- ALLIANCE SELECTION ---
    private enum GoalDirection { LEFT, RIGHT }
    private GoalDirection selectedAlliance = GoalDirection.RIGHT; // Default to Right side goal

    // --- SEQUENCE LOGIC ---
    public enum BallColor { GREEN, PURPLE, UNKNOWN }
    private final BallColor[] SEQ_1 = {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
    private final BallColor[] SEQ_2 = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
    private final BallColor[] SEQ_3 = {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    private final BallColor[] SEQ_UNKNOWN = {BallColor.UNKNOWN};

    @Override
    public void runOpMode() {
        initializeHardware();

        // =============================== ALLIANCE SELECTION LOOP ===============================
        // This loop runs during the INIT phase, before the driver presses START.
        boolean selectionMade = false;

        selectedAlliance = GoalDirection.RIGHT;
        selectionMade = true;
       
        // =====================================================================================

        waitForStart();

        // Final check before running the sequence
        if (opModeIsActive() && selectionMade) {
            telemetry.addData("Executing for", selectedAlliance + " Goal");
            telemetry.update();

            // Step 1: Now that we have arrived, scan for the sequence tag obelisk.
            telemetry.addLine("Step 2: Arrived, now scanning obelisk...");
            telemetry.update();
            BallColor[] detectedSequence = readSequenceFromObelisk(1.0); // Scan for 2 seconds

            telemetry.addData("Sequence Found", Arrays.toString(detectedSequence));
            telemetry.update();

            // --- Conditional Logic: Only proceed if a sequence was found ---
            if (detectedSequence[0] != BallColor.UNKNOWN) {

                // Step 2: shooting based on ball sequence
                runShootingSequence(detectedSequence);

                telemetry.addLine("Step 2: Shoot 3 balls...");
                telemetry.update();
                sleep(500); // Run shooter for 1.5 seconds

                // 1. Move forward 1 foot (12 inches) to clear the shooting area
                driveDistance(12, 0.5);


            } else {
                // If the obelisk scan failed, stop here.
                telemetry.addLine("ERROR: No sequence found. Stopping.");
                telemetry.update();
            }

            sleep(500);
        }// End of OpMode
    }

    public void runShootingSequence(BallColor[] detectedSequence ) {
        // Step 4: Selective shooting based on sequence
        telemetry.addLine("Step 4: Executing shooting sequence..." + Arrays.toString(detectedSequence));
        telemetry.update();

        for (BallColor color : detectedSequence) {
            if (color == BallColor.GREEN) {
                // Green is held in the Left Intake
                telemetry.addData("Shooting", "GREEN (Left)");
                telemetry.update();
                runLeftShooter(detectedSequence);
                sleep(800); // Time to clear one ball
                stopLeftShooter();
            } else if (color == BallColor.PURPLE) {
                // Purple is held in the Right Intake
                telemetry.addData("Shooting", "PURPLE (Right)");
                telemetry.update();
                runRightShooter(detectedSequence);
                sleep(800); // Time to clear one ball
                stopRightShooter();
            }
            // Small pause between individual ball launches to allow shooter recovery
            sleep(50);
        }

        // Ensure everything is off after the loop finishes
        stopAllMechanisms();
    }

    /**
     * Stops the right shooter components.
     */
    public void stopRightShooter() {
        rightConveyorBelt.setPower(0);
        rightIntakeGate.setPower(0);
    }

    /**
     * Stops the left shooter components.
     */
    public void stopLeftShooter() {
        leftConveyorBelt.setPower(0);
        leftIntakeGate.setPower(0);
    }

    /**
     * Stops all conveyor and gate mechanisms.
     */
    public void stopAllMechanisms() {
        stopRightShooter();
        stopLeftShooter();
        sleep(1000);
        intakeRoller.setPower(0);
    }

    /**
     * Activates the shooter-specific components (conveyor and gate servo).
     */
    public void runRightShooter(BallColor[] seq) {

        if (Arrays.equals(seq, SEQ_1) || Arrays.equals(seq, SEQ_3)) {
            rightConveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
            sleep(800);
            rightIntakeGate.setPower(GATE_SERVO_POWER);
            sleep(1000);
            intakeRoller.setPower(SHOOTER_CONVEYOR_POWER);
        } else if (Arrays.equals(seq, SEQ_2)) {
            rightConveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
            sleep(800);
            rightIntakeGate.setPower(GATE_SERVO_POWER);
        }
    }


    public void runLeftShooter(BallColor[]seq) { //Green
        if (Arrays.equals(seq, SEQ_1) || Arrays.equals(seq, SEQ_3)) {
            leftConveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
            sleep(200);
            leftIntakeGate.setPower(GATE_SERVO_POWER);
        } else if (Arrays.equals(seq, SEQ_2)) {
            leftConveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
            sleep(150);
            leftIntakeGate.setPower(GATE_SERVO_POWER);
             sleep(1000);
             intakeRoller.setPower(SHOOTER_CONVEYOR_POWER);
        }

    }


    //----------------------------------------------------------------------------------------------
    // INITIALIZATION & HELPER METHODS
    //----------------------------------------------------------------------------------------------

    /** Initializes all hardware and sets motor directions. */
    private void initializeHardware() {
        // Drivetrain
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeRoller = hardwareMap.get(DcMotor.class, "intake_roller");
        rightIntakeGate = hardwareMap.get(CRServo.class, "right_intake_gate");
        rightConveyorBelt = hardwareMap.get(DcMotor.class, "right_conveyor_belt");
        leftIntakeGate = hardwareMap.get(CRServo.class, "left_intake_gate");
        leftConveyorBelt = hardwareMap.get(DcMotor.class, "left_conveyor_belt");

        // --- Set Motor & Servo Directions ---
        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeGate.setDirection(DcMotorSimple.Direction.REVERSE);   // Spins to help feed conveyor
        rightConveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD); // Spins to push balls up/out
        leftIntakeGate.setDirection(DcMotorSimple.Direction.FORWARD);   // Spins to help feed conveyor
        leftConveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE); // Spins to push balls up/out

        // Sensors
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
//        colorSensor.setGain(10f);

//        huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");
//        if (!huskyLens.knock()) {
//            telemetry.addData("FATAL", "HuskyLens not responding!");
//        }
//        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    /** Performs a stationary scan for a sequence tag for a given duration. */
    private BallColor[] readSequenceFromObelisk(double scanSeconds) {
        ElapsedTime scanTimer = new ElapsedTime();
//        while(opModeIsActive() && scanTimer.seconds() < scanSeconds) {
//            HuskyLens.Block[] blocks = huskyLens.blocks();
//            if (blocks.length > 0) {
//                for (HuskyLens.Block tag : blocks) {
//                    if (tag.id == 1) return SEQ_1;
//                    if (tag.id == 2) return SEQ_2;
//                    if (tag.id == 3) return SEQ_3;
//                }
//            }

            sleep(50);
//        }
        telemetry.addLine("ERROR: No sequence tag found. Default sequence loaded.");
        return SEQ_3;
    }

    /** Drives the robot a specific distance in inches using encoders. */
    private void driveDistance(double distanceInches, double power) {
        if (!opModeIsActive()) return;
        driveMecanum(distanceInches, 0, power); // Use the new method for straight movement
    }

    /** Drives the robot diagonally using forward and strafe distances. */
    private void driveMecanum(double forwardInches, double strafeInches, double power) {
        if (!opModeIsActive()) return;

        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

        int lfTicks = (int)((forwardInches + strafeInches) * countsPerInch);
        int rfTicks = (int)((forwardInches - strafeInches) * countsPerInch);
        int lbTicks = (int)((forwardInches - strafeInches) * countsPerInch);
        int rbTicks = (int)((forwardInches + strafeInches) * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPosition(lfTicks, rfTicks, lbTicks, rbTicks);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power);

        // --- FIXED: Uncommented this block ---
        while (opModeIsActive() && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {
            telemetry.addData("Status", "Driving to target...");
            telemetry.addData("Ticks", "LF: %d, RF: %d",
                    leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
            telemetry.update();
        }

        setDrivePower(0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /** Turns the robot a specific angle using encoders. */
    private void turnRobot(double angle, double power) {
        double turnDistance = (angle / 360.0) * (Math.PI * ROBOT_TRACK_WIDTH_INCHES);
        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
        int targetTicks = (int) (turnDistance * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // For a left turn (positive angle), left wheels go forward, right wheels go backward.
        setTargetPosition(targetTicks, -targetTicks, targetTicks, -targetTicks);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power);
        while (opModeIsActive() && leftFrontDrive.isBusy()) {
            // Optional telemetry
        }
        setDrivePower(0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- LOW-LEVEL MOTOR CONTROL ---
    private void setDrivePower(double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
    private void setDriveRunMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }
    private void setTargetPosition(int lf, int rf, int lb, int rb) {
        leftFrontDrive.setTargetPosition(lf);
        rightFrontDrive.setTargetPosition(rf);
        leftBackDrive.setTargetPosition(lb);
        rightBackDrive.setTargetPosition(rb);
    }
}
