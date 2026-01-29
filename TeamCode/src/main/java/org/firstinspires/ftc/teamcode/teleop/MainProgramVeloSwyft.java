package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * This is the main TeleOp program that combines the Drivetrain, Intake, and Shooter systems.
 * Gamepad 1 controls the robot's movement (Mecanum drive).
 * Gamepad 2 controls the Intake and Shooter mechanisms with an advanced stateful toggle system.
 */
@TeleOp(name = "Main Program Velo Swyft DS", group = "Main")
public class MainProgramVeloSwyft extends OpMode {

    // --- DRIVETRAIN HARDWARE ---
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // --- INTAKE & SHOOTER HARDWARE ---
    private DcMotor intakeRoller;
    private CRServo leftIntakeGate;
    private DcMotorEx leftConveyorBelt;

    private CRServo rightIntakeGate;
    private DcMotorEx rightConveyorBelt;
    private static final double MAX_VELOCITY = 2000;
    private static final double REVERSE_VELOCITY = -1000;

    // --- MECHANISM STATE MACHINE ---
    private enum SystemState {
        STOPPED,
        LEFT_REVERSED,
        RIGHT_REVERSED,
        INTAKE_ONLY,
        SHOOTING_SEQUENCE,
//        SHOOTER_ONLY,// Add this
        REVERSE_OR_EMPTY_RIGHT_CHANNEL,

        REVERSE_OR_EMPTY_LEFT_CHANNEL,
        MANUAL_NEAR_SHOOTING,
        MANUAL_FAR_SHOOTING,
        MANUAL_MEDIUM_SHOOTING
    }
    private SystemState mechanismState = SystemState.STOPPED;

    // --- Button Edge Detection (for smart toggling) ---
    private boolean a2_was_pressed = false;
    private boolean b2_was_pressed = false;
    private boolean y2_was_pressed = false;
    private boolean x2_was_pressed = false;

    private boolean dpad_left_was_pressed = false;
    private boolean dpad_up_was_pressed = false;
    private boolean dpad_down_was_pressed = false; //manual shooting
    private boolean dpad_right_was_pressed = false;
    private boolean left_trigger_was_pressed = false;

    // --- MECHANISM CONSTANTS ---
    private static final double INTAKE_ROLLER_POWER = 1.0;

    private static final double GATE_SERVO_POWER = 1.0;
    // --- NEW SENSOR HARDWARE ---
    private AnalogInput distanceSensor;


    /**
     * Code to run ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        // ---- Initialize Sensors and Camera ---
        distanceSensor = hardwareMap.get(AnalogInput.class, "ranger");

        // --- Initialize Drivetrain ---
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBack = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set Drivetrain motor directions (TUNE THIS FOR YOUR ROBOT)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeRoller = hardwareMap.get(DcMotor.class, "intake_roller");
        leftIntakeGate = hardwareMap.get(CRServo.class, "left_intake_gate");
        leftConveyorBelt = hardwareMap.get(DcMotorEx.class, "left_conveyor_belt");
        rightIntakeGate = hardwareMap.get(CRServo.class, "right_intake_gate");
        rightConveyorBelt = hardwareMap.get(DcMotorEx.class, "right_conveyor_belt");

        leftConveyorBelt.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightConveyorBelt.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // --- ADD THIS TO STOP INSTANTLY ---
        leftConveyorBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightConveyorBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        // Increase the "F" (Feedforward) and "P" values so the motor can reach higher RPMs
//        // These values are standard for GoBILDA 5202 motors to reach ~2500 ticks/s
//        leftConveyorBelt.setVelocityPIDFCoefficients(12, 3, 0, 12);
//        rightConveyorBelt.setVelocityPIDFCoefficients(12, 3, 0, 12);
        // Lowered Integral (3.0 -> 0.1) to prevent runaway speed
        // Lowered Feedforward (12.0 -> 11.0) to prevent overshoot
        leftConveyorBelt.setVelocityPIDFCoefficients(11.0, 0.3, 0.0, 11.0);
        rightConveyorBelt.setVelocityPIDFCoefficients(11.0, 0.3, 0.0, 11.0);
        // Set Intake/Shooter motor directions (TUNE THIS FOR YOUR ROBOT)
        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        leftConveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftIntakeGate.setPower(GATE_SERVO_POWER);

        rightConveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeGate.setDirection(CRServo.Direction.REVERSE);
//        rightIntakeGate.setPower(GATE_SERVO_POWER);

        // Ensure all mechanisms are stopped on initialization
        leftIntakeGate.setPower(0);
        rightIntakeGate.setPower(0);

        stopAllMechanisms();

        // --- Telemetry ---
        telemetry.addLine("Main Program Initialized");
        telemetry.addLine("Gamepad 1: Drivetrain");
        telemetry.addLine("Gamepad 2: Mechanisms (Press same button to stop)");
        telemetry.update();
    }

    /**
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        // --- Gamepad 1: Drivetrain Control ---
        handleDrivetrain();

        // --- Gamepad 2: Intake and Shooter Control ---
        handleMechanisms();

        // --- Update Telemetry ---
        updateTelemetry();
    }

    /**
     * This method contains the joystick logic for the Mecanum drivetrain.
     */
    private void handleDrivetrain() {
        // The '-' sign on left_stick_y is because the joystick's Y-axis is inverted
        double forward = -gamepad1.left_stick_y;  // Controls forward and backward
        double strafe  =  gamepad1.left_stick_x;  // Controls left and right strafing
        double rotate  =  gamepad1.right_stick_x;  // Controls robot rotation

        // Mecanum Drive Formulas
        double leftFrontPower  = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower   = forward - strafe + rotate;
        double rightBackPower  = forward + strafe - rotate;

        // Normalization to prevent motor powers from exceeding 1.0
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        leftFront.setPower(leftFrontPower / denominator);
        rightFront.setPower(rightFrontPower / denominator);
        leftBack.setPower(leftBackPower / denominator);
        rightBack.setPower(rightBackPower / denominator);
    }

    /**
     * This method contains the advanced state machine logic for the intake and shooter.
     * Corrected handleMechanisms with proper edge detection
     */
    private void handleMechanisms() {
        // 1. Safety Check: If Gamepad 2 is not detected, stop everything and alert driver
        if (gamepad2.id == -1) { // -1 usually indicates no gamepad is assigned
            telemetry.addData("WARNING", "Gamepad 2 NOT DETECTED!");
            stopAllMechanisms();
            return;
        }

        // --- State Selection Logic (Edge Detection) ---

        // Check for 'A' button (Intake)
        if (gamepad2.a && !a2_was_pressed) {
            mechanismState = (mechanismState == SystemState.INTAKE_ONLY) ? SystemState.STOPPED : SystemState.INTAKE_ONLY;
        }

        // Check for Dpad Up (Manual Shooting from 60 inches Only)
        if (gamepad2.dpad_up && !dpad_up_was_pressed) {
            mechanismState = (mechanismState == SystemState.MANUAL_MEDIUM_SHOOTING) ? SystemState.STOPPED : SystemState.MANUAL_MEDIUM_SHOOTING;
        }
        dpad_up_was_pressed = gamepad2.dpad_up; // Update edge detection

        // Check for Dpad Down (Manual Shooting)
        if (gamepad2.dpad_down && !dpad_down_was_pressed) {
            mechanismState = (mechanismState == SystemState.MANUAL_NEAR_SHOOTING) ? SystemState.STOPPED : SystemState.MANUAL_NEAR_SHOOTING;
        }
        dpad_down_was_pressed = gamepad2.dpad_down; // Update edge detection

        if (gamepad2.left_trigger > 0.5 && !left_trigger_was_pressed) {
            mechanismState = (mechanismState == SystemState.MANUAL_FAR_SHOOTING) ? SystemState.STOPPED : SystemState.MANUAL_FAR_SHOOTING;
        }
        left_trigger_was_pressed = (gamepad2.left_trigger > 0.5); // Update edge detection

        if (gamepad2.dpad_right && !dpad_right_was_pressed) {
            mechanismState = (mechanismState == SystemState.REVERSE_OR_EMPTY_RIGHT_CHANNEL) ? SystemState.STOPPED : SystemState.REVERSE_OR_EMPTY_RIGHT_CHANNEL;
        }
        dpad_right_was_pressed = gamepad2.dpad_right; // Update edge detection

        if (gamepad2.dpad_left && !dpad_left_was_pressed) {
            mechanismState = (mechanismState == SystemState.REVERSE_OR_EMPTY_LEFT_CHANNEL) ? SystemState.STOPPED : SystemState.REVERSE_OR_EMPTY_LEFT_CHANNEL;
        }
        dpad_left_was_pressed = gamepad2.dpad_left; // Update edge detection


        // Check for 'Y' button (Shooting Sequence)
        if (gamepad2.y && !y2_was_pressed) {
            if (mechanismState != SystemState.SHOOTING_SEQUENCE) {
                mechanismState = SystemState.SHOOTING_SEQUENCE;
            } else {
                mechanismState = SystemState.STOPPED;
            }
        }

        // Check for 'X' button (Left Reverse)
        if (gamepad2.x && !x2_was_pressed) {
            mechanismState = (mechanismState == SystemState.LEFT_REVERSED) ? SystemState.STOPPED : SystemState.LEFT_REVERSED;
        }

        // Check for 'B' button (Right Reverse)
        if (gamepad2.b && !b2_was_pressed) {
            mechanismState = (mechanismState == SystemState.RIGHT_REVERSED) ? SystemState.STOPPED : SystemState.RIGHT_REVERSED;
        }

        // --- Debugging: Show which button is currently physically held down ---
        telemetry.addData("G2 Buttons", "A:%b B:%b X:%b Y:%b",
                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y);

        // Update edge detection variables for next loop
        a2_was_pressed = gamepad2.a;
        x2_was_pressed = gamepad2.x;
        y2_was_pressed = gamepad2.y;
        b2_was_pressed = gamepad2.b;

        // --- State Execution Logic ---
        switch (mechanismState) {
            case INTAKE_ONLY:
                runIntake();
                break;
            case REVERSE_OR_EMPTY_RIGHT_CHANNEL:
                intakeRoller.setPower(-INTAKE_ROLLER_POWER);
                rightIntakeGate.setPower(-GATE_SERVO_POWER);
                rightConveyorBelt.setVelocity(REVERSE_VELOCITY);
                break;
            case REVERSE_OR_EMPTY_LEFT_CHANNEL:
                intakeRoller.setPower(-INTAKE_ROLLER_POWER);
                leftIntakeGate.setPower(-GATE_SERVO_POWER);
                leftConveyorBelt.setVelocity(REVERSE_VELOCITY);
                break;
            case SHOOTING_SEQUENCE:
                runShootingSequence();
                break;
            case MANUAL_FAR_SHOOTING: // left_trigger
                runShootingActually(1650, 0.90, true); //80 inches from goal
                break;
            case MANUAL_MEDIUM_SHOOTING: // dpad_up
                runShootingActually(1500, 0.90, true); //60 inches from goal
                break;
            case MANUAL_NEAR_SHOOTING: // dpad_down
                runShootingActually(1450, 0.90, true); //40 inches from goal
                break;
            case LEFT_REVERSED:
                leftConveyorBelt.setVelocity(REVERSE_VELOCITY);
                leftIntakeGate.setPower(-1.0);
                break;
            case RIGHT_REVERSED:
                rightConveyorBelt.setVelocity(REVERSE_VELOCITY);
                rightIntakeGate.setPower(-1.0);
                break;
            case STOPPED:
            default:
                applyStopPowers();
                break;
        }
    }

    public void runShootingActually(double targetVelocity, double powerToUse, boolean manual) {
        // 3. Apply Velocity
        leftConveyorBelt.setVelocity(targetVelocity);
        rightConveyorBelt.setVelocity(targetVelocity);

        // 4. Trigger Gates ONLY when belts reach 90% of target velocity
        // We lowered this from 95% to 90% to help the motors fire more reliably at lower powers
        boolean readyToShootLeft = (manual)? Math.abs(leftConveyorBelt.getVelocity()) >= (targetVelocity * powerToUse) - 150
                : Math.abs(leftConveyorBelt.getVelocity()) >= (targetVelocity * powerToUse);
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

    public void applyStopPowers() {
        // Explicitly set velocity to 0
        leftConveyorBelt.setVelocity(0);
        rightConveyorBelt.setVelocity(0);

        // Force power to 0 to ensure the Brake behavior engages immediately
        leftConveyorBelt.setPower(0);
        rightConveyorBelt.setPower(0);

        leftIntakeGate.setPower(0);
        rightIntakeGate.setPower(0);
        intakeRoller.setPower(0);
    }

    public void runShootingSequence() {
        double distInches = (distanceSensor.getVoltage()*48.7) - 4.9;
        double targetVelocity, powerToUse, powerRatio;
        powerToUse = 0.85;
        if (distInches > 80 || Double.isNaN(distInches)) {
            powerRatio = 0.90; // Default for out of range

        } else if (distInches < 5) {
            powerRatio = 0;    // Safety: don't shoot if touching wall
            powerToUse = 0;
        } else if (distInches < 50) {
            // CLOSE RANGE LOGIC:
            // This formula ensures that even at 10 inches, the power stays around 0.68
            powerRatio = (distInches * 0.0015) + 0.67;
        } else {
            // LONG RANGE LOGIC:
            powerRatio = (distInches * 0.00238) + 0.643;
        }

        // 2. Calculate Final Velocity
        targetVelocity = Math.min(powerRatio * MAX_VELOCITY, MAX_VELOCITY);
        runShootingActually(targetVelocity, powerToUse, false);
    }
    /**
     * Updates all telemetry data in one place.
     */
    private void updateTelemetry() {
        telemetry.addData("--- Drivetrain ---", "");
        // Get voltage from the hardwareMap
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        telemetry.addData("Battery", "%.2fV", voltage);

        // 2. Sensor Distance Calculation (Analog to Inches)
        // Formula used in runShootingSequence: (voltage * 48.7) - 4.9
        double sensorVoltage = distanceSensor.getVoltage();
        double distanceInches = (sensorVoltage * 48.7) - 4.9;

        telemetry.addData("Raw Voltage", "%.3fV", sensorVoltage);
        telemetry.addData("Dist", "%.1f in", distanceInches);

        telemetry.addData("--- Mechanisms ---", "");
        telemetry.addData("State", mechanismState);

        // This ensures velocities are ALWAYS printed to the screen
        telemetry.addData("L-Velo", "%.0f ticks/s", leftConveyorBelt.getVelocity());
        telemetry.addData("R-Velo", "%.0f", rightConveyorBelt.getVelocity());

        // If we are shooting, show the target as well
        if (mechanismState == SystemState.SHOOTING_SEQUENCE) {
            telemetry.addLine("!! SHOOTING ACTIVE !!");
        }

        telemetry.update(); // Only ONE update call at the very end
    }

    // --- Helper Methods for Mechanisms ---

    public void runIntake() {
        intakeRoller.setPower(INTAKE_ROLLER_POWER);
    }

    /**
     * Stops all conveyor and gate mechanisms.
     */
    public void stopAllMechanisms() {
        applyStopPowers();
        mechanismState = SystemState.STOPPED;
    }



}

