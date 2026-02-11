package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name = "Near Left Auto")

public class NearLeftAutonomous extends LinearOpMode {

    // --- ROBOT CONSTANTS (Tune These!) ---
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // For goBILDA 5203-series motor
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double ROBOT_TRACK_WIDTH_INCHES = 14.0;   // Distance between left and right wheels
    static final double OBELISK_SCAN_ANGLE_DEGREES = 60.0;
    private static final double SHOOTER_CONVEYOR_POWER = 1.0;
    private static final double GATE_SERVO_POWER = 1.0;
    private static final double INTAKE_ROLLER_POWER = 1.0;
    private static final double SLOWER_POWER = 0.8;


    private final BallColor[] SEQ_1 = {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
    private final BallColor[] SEQ_2 = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
    private final BallColor[] SEQ_3 = {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    private BallColor[] detectedSequence = {BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN};

    // --- HARDWARE ---
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private HuskyLens huskyLens;
    private NormalizedColorSensor colorSensor;
    private DcMotor intakeRoller;

    //        Channel 1
    private CRServo rightIntakeGate;
    private DcMotorEx rightConveyorBelt;
    //      Channel 2
    private CRServo leftIntakeGate;
    private DcMotorEx leftConveyorBelt;
    private enum GoalDirection { LEFT, RIGHT;}

    private GoalDirection selectedAlliance = GoalDirection.LEFT; // Default to Right side goal

    public enum BallColor { GREEN, PURPLE, UNKNOWN;}

    @Override
    public void runOpMode() {
        initializeHardware();

        // =============================== ALLIANCE SELECTION LOOP ===============================

        selectedAlliance = GoalDirection.LEFT;


        // =====================================================================================

        telemetry.addData("Ready to Run for", selectedAlliance + " Alliance");
        telemetry.update();

        waitForStart();

        // --- AUTONOMOUS SEQUENCE ---

        // Step 1: Drive diagonally - 68 inches backward.
        telemetry.addLine("Step 1: Driving 40 inches backward.");
        telemetry.update();
        //  driveMecanum(-58, 0, 0.7); // Drive 70 forward, 35 right, at 70% power
        driveMecanum(-40, 0, 0.70, false); //Drive 34 inches backward

        detectedSequence = SEQ_1;
        sleep(100);
        //Step2 : turn left to scan the sequence
//        turnRobot(110.0, 0.7);  // Turn RIGHT: 45degrees
////        driveMecanum(5, 0, 0.7);
//        sleep(100);
        // Step 3: Now that we have arrived, scan for the sequence tag obelisk.
//        telemetry.addLine("Step 3: Arrived, now scanning obelisk...");
//        telemetry.update();
        detectedSequence = SEQ_1;//readSequenceFromObelisk(3.0); // Scan for 2 seconds
//        driveMecanum(-5, 0, 0.7);
        sleep(100);

        telemetry.addData("Sequence Found", Arrays.toString(detectedSequence));
        telemetry.update();
        //  sleep(500);

        // --- Conditional Logic: Only proceed if a sequence was found ---
        if (detectedSequence[0] != BallColor.UNKNOWN) {

            // Step 3: Turn towards the correct goal based on the selected alliance and the same angle as the steps 2.
//            telemetry.addLine("Step 4: Turning towards goal...");
//            telemetry.update();
//            turnRobot(-140.0, 0.7); // Turn LEFT for 45 degrees towards the goal post

            // Step 4: shooting. // based on sequence shoot
            telemetry.addLine("Step 5: Shoot 3 balls...");
            telemetry.update();
            runShootingSequence(2.5, 1250, 0.90, detectedSequence);

           // first line
            pickUpLineShoot(113.0, 101, 39, detectedSequence);

                // second line
//            pickUpLineShoot(134.0, 124, 66, detectedSequence);
//
//            // third line
//            pickUpLineShoot(158.0, 148, 90, detectedSequence);

            turnRobot(OBELISK_SCAN_ANGLE_DEGREES, 0.7); // Turn Right for Left side Alliance

            driveDistance(-30, 0.4, false);  // Move backward
        } else {
            // If the obelisk scan failed, stop here.
            telemetry.addLine("ERROR: No sequence found. Stopping.");
            telemetry.update();
        }

//        sleep(500); // End of OpMode
    }

    public void pickUpLineShoot(double angleBack, double angleForward, double backwardInches, BallColor[] detectedSequence)  {
        telemetry.addLine("Step 6a: turning left to collect balls...");
        telemetry.update();
        turnRobot(-angleBack, 0.7);// assuming the ball collector will pull all the balls within its path
        telemetry.addLine("Step 6b: drive towards the balls..");
        telemetry.update();
        driveMecanum(backwardInches, 0, 0.3, true); //Drive 36 inches fwd // for next set of balls  add + 24 and for the next one  + 24
        driveMecanum(-backwardInches, 0, 0.7, false); //Drive 36 inches back

        turnRobot(angleForward, 0.7);// turn towards goalpost //make it +90 if rotation is not correct.
        telemetry.addLine("Step 10: Shoot 3 balls...");
        telemetry.update();
        runShootingSequence(3.0, 1250, 0.90, detectedSequence);

    }


    /**
     * Looped shooting sequence to allow motors to reach target speed in Autonomous.
     */
    public void runShootingSequence(double durationSeconds, double targetVelocity, double thresholdPercent, BallColor[] seq) {
        ElapsedTime shootTimer = new ElapsedTime();
        while (opModeIsActive() && shootTimer.seconds() < durationSeconds) {
            runShootingActually(targetVelocity, thresholdPercent);
            showLiveStats(); // Updates telemetry every loop
            sleep(20);
        }
        stopAllMechanisms();
    }


    private void showLiveStats() {
        telemetry.addData("--- SENSORS ---", "");
        telemetry.addData("Husky ID", detectedSequence[0]);
        telemetry.addData("Shooter Vel (L/R)", "%.0f / %.0f",
                leftConveyorBelt.getVelocity(), rightConveyorBelt.getVelocity());
        telemetry.update();
    }

    public void runShootingActually(double targetVelocity, double powerToUse) {
        // 3. Apply Velocity
        leftConveyorBelt.setVelocity(targetVelocity);
        rightConveyorBelt.setVelocity(targetVelocity);

        // 4. Trigger Gates ONLY when belts reach 90% of target velocity
        // We lowered this from 95% to 90% to help the motors fire more reliably at lower powers
        boolean readyToShootLeft =  Math.abs(leftConveyorBelt.getVelocity()) >= (targetVelocity * powerToUse);
        boolean readyToShootRight = Math.abs(rightConveyorBelt.getVelocity()) >= (targetVelocity * powerToUse) + 100;

        // Combined check: Both must be at speed, and the target must be valid
        // Logic for Left side
// We use 0.90 to trigger opening, but we could stay open as long as it's > 0.85
// to prevent flickering when the pixel puts load on the motor.
        if (readyToShootLeft && targetVelocity > 100) {
            leftIntakeGate.setPower(GATE_SERVO_POWER);
        } else {
            // Only close the gate if it drops significantly below target
            if (Math.abs(leftConveyorBelt.getVelocity()) < (targetVelocity * (powerToUse - 0.05))) {
                leftIntakeGate.setPower(0);
            }
        }

// Logic for Right side
        if (readyToShootRight && targetVelocity > 100) {
            rightIntakeGate.setPower(GATE_SERVO_POWER);
        } else {
            if (Math.abs(rightConveyorBelt.getVelocity()) < (targetVelocity * (powerToUse - 0.05))) {
                rightIntakeGate.setPower(0);
            }
        }

        // Intake Roller logic (only run if at least one side is ready to feed)
        if ((readyToShootLeft || readyToShootRight) && targetVelocity > 100) {
            intakeRoller.setPower(INTAKE_ROLLER_POWER);
        } else {
            intakeRoller.setPower(0);
        }
        // Telemetry to help the driver see why it may not be shooting yet
        telemetry.addData("Target Velo", "%.0f", targetVelocity);
        telemetry.addData("L-Vel Actual", "%.0f", leftConveyorBelt.getVelocity());
        telemetry.addData("R-Vel Actual", "%.0f", rightConveyorBelt.getVelocity());
        telemetry.addData("Ready to Shoot", (readyToShootLeft && readyToShootRight) ? "YES" : "SPINNING UP...");

    }

    /** Performs a stationary scan for a sequence tag for a given duration. */
    private BallColor[] readSequenceFromObelisk(double scanSeconds) {
        ElapsedTime scanTimer = new ElapsedTime();
        while(opModeIsActive() && scanTimer.seconds() <= scanSeconds) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks.length > 0) {
                for (HuskyLens.Block tag : blocks) {
                    if (tag.id == 1) return SEQ_1;
                    if (tag.id == 2) return SEQ_2;
                    if (tag.id == 3) return SEQ_3;
                }
            }
            sleep(100);
        }
        telemetry.addLine("ERROR: No sequence found. Default Sequence loaded");
        telemetry.update();
        return SEQ_1;
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
        intakeRoller.setPower(0);
    }


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
        rightConveyorBelt = hardwareMap.get(DcMotorEx.class, "right_conveyor_belt");

        leftIntakeGate = hardwareMap.get(CRServo.class, "left_intake_gate");
        leftConveyorBelt = hardwareMap.get(DcMotorEx.class, "left_conveyor_belt");

        // --- Set Motor & Servo Directions ---
        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeGate.setDirection(DcMotorSimple.Direction.REVERSE);   // Spins to help feed conveyor
        rightConveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD); // Spins to push balls up/out
        leftIntakeGate.setDirection(DcMotorSimple.Direction.FORWARD);   // Spins to help feed conveyor
        leftConveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE); // Spins to push balls up/out

        // CRITICAL: Ensure motors are in RUN_USING_ENCODER for setVelocity to work
        leftConveyorBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightConveyorBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Sensors
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(10f);

        huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");
        if (!huskyLens.knock()) {
            telemetry.addData("FATAL", "HuskyLens not responding!");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    /** Drives the robot a specific distance in inches using encoders. */
    private void driveDistance(double distanceInches, double power, boolean intake) {
        if (!opModeIsActive()) return;
        driveMecanum(distanceInches, 0, power, intake); // Use the new method for straight movement
    }


    /** Drives the robot diagonally using backward and strafe distances. */
    private void driveMecanum(double backwardInches, double strafeInches, double power, boolean intake) {
        if (!opModeIsActive()) return;
        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

        int lfTicks = (int)((backwardInches + strafeInches) * countsPerInch);
        int rfTicks = (int)((backwardInches - strafeInches) * countsPerInch);
        int lbTicks = (int)((backwardInches - strafeInches) * countsPerInch);
        int rbTicks = (int)((backwardInches + strafeInches) * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPosition(lfTicks, rfTicks, lbTicks, rbTicks);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power);

        // Turn on intake and conveyors if requested
        if (intake) {
            intakeRoller.setPower(INTAKE_ROLLER_POWER);
            rightIntakeGate.setPower(SLOWER_POWER);
            leftIntakeGate.setPower(SLOWER_POWER);
            leftConveyorBelt.setPower(-SHOOTER_CONVEYOR_POWER);  // Run slow to pull balls in
            rightConveyorBelt.setPower(-SHOOTER_CONVEYOR_POWER);
        }

        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy())) {
            showLiveStats(); // <--- SHOWS DISTANCE ALL THE TIME
        }

        setDrivePower(0);
        if (intake) {
            stopAllMechanisms();
        }
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

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
}
