//package org.firstinspires.ftc.teamcode.teleop;
//
//
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//
///**
// * This is the main TeleOp program that combines the Drivetrain, Intake, and Shooter systems.
// * Gamepad 1 controls the robot's movement (Mecanum drive).
// * Gamepad 2 controls the Intake and Shooter mechanisms with an advanced stateful toggle system.
// */
//@TeleOp(name = "Main Program No Sep Shooter (Drive + Mechanisms)", group = "Main")
//public class MainProgram extends OpMode {
//
//    // --- DRIVETRAIN HARDWARE ---
//    private DcMotor leftFront;
//    private DcMotor rightFront;
//    private DcMotor leftBack;
//    private DcMotor rightBack;
//
//    // --- INTAKE & SHOOTER HARDWARE ---
//    private DcMotor intakeRoller;
//    private CRServo leftIntakeGate;
//    private DcMotor leftConveyorBelt;
//
//    private CRServo rightIntakeGate;
//    private DcMotor rightConveyorBelt;
//    private ElapsedTime mechanismTimer = new ElapsedTime();
//    private boolean lagActive = false; // Tracks if we are currently in a waiting period
//
//
//    // --- MECHANISM STATE MACHINE ---
//    private enum SystemState {
//        STOPPED,
//        LEFT_REVERSED,
//        RIGHT_REVERSED,
//        INTAKE_ONLY,
//        SHOOTING_SEQUENCE,
//        SHOOTER_ONLY,// Add this
//        REVERSE_OR_EMPTY_RIGHT_CHANNEL,
//
//        REVERSE_OR_EMPTY_LEFT_CHANNEL
//    }
//    private SystemState mechanismState = SystemState.STOPPED;
//
//    // --- Button Edge Detection (for smart toggling) ---
//    private boolean a2_was_pressed = false;
//    private boolean b2_was_pressed = false;
//    private boolean y2_was_pressed = false;
//    private boolean x2_was_pressed = false;
//
//    private boolean dpad_left_was_pressed = false;
//    private boolean dpad_up_was_pressed = false;
//    private boolean dpad_right_was_pressed = false;
//
//    // --- MECHANISM CONSTANTS ---
//    private static final double INTAKE_ROLLER_POWER = 1.0;
//
//    private static final double LEFT_SHOOTER_CONVEYOR_POWER = 0.7    ;
//    private static final double RIGHT_SHOOTER_CONVEYOR_POWER = 1.0;
//
//    private static final double GATE_SERVO_POWER = 1.0;
//    // --- NEW SENSOR HARDWARE ---
//    private DistanceSensor distanceSensor;
//    private double dynamicShootingPower = 0.8; // Default fallback
//    private static final double MAX_SHOOT_DISTANCE_INCHES = 108.0; // 9 feet
//
//
//
//    /**
//     * Code to run ONCE when the driver hits INIT.
//     */
//    @Override
//    public void init() {
//        // ---- Initialize Sensors and Camera ---
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");
//        // --- ADD THIS TO INCREASE RANGE TO 2M ---
//        // Note: This works if using the standard REV 2M Distance Sensor
//        // It sets the sensor to be more sensitive for longer distances
//        if (distanceSensor instanceof com.qualcomm.hardware.rev.Rev2mDistanceSensor) {
//            Rev2mDistanceSensor revSensor =
//                    (Rev2mDistanceSensor) distanceSensor;
//        }
//
//        // --- Initialize Drivetrain ---
//        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
//        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
//        leftBack = hardwareMap.get(DcMotor.class, "left_back_drive");
//        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");
//
//        // Set Drivetrain motor directions (TUNE THIS FOR YOUR ROBOT)
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        intakeRoller = hardwareMap.get(DcMotor.class, "intake_roller");
//        leftIntakeGate = hardwareMap.get(CRServo.class, "left_intake_gate");
//        leftConveyorBelt = hardwareMap.get(DcMotor.class, "left_conveyor_belt");
//        rightIntakeGate = hardwareMap.get(CRServo.class, "right_intake_gate");
//        rightConveyorBelt = hardwareMap.get(DcMotor.class, "right_conveyor_belt");
//
//        // --- ADD THIS TO STOP INSTANTLY ---
//        leftConveyorBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightConveyorBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intakeRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Set Intake/Shooter motor directions (TUNE THIS FOR YOUR ROBOT)
//        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftConveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);
////        leftIntakeGate.setPower(GATE_SERVO_POWER);
//
//        rightConveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightIntakeGate.setDirection(CRServo.Direction.REVERSE);
////        rightIntakeGate.setPower(GATE_SERVO_POWER);
//
//        // Ensure all mechanisms are stopped on initialization
//        leftIntakeGate.setPower(0);
//        rightIntakeGate.setPower(0);
//
//        stopAllMechanisms();
//
//        // --- Telemetry ---
//        telemetry.addLine("Main Program Initialized");
//        telemetry.addLine("Gamepad 1: Drivetrain");
//        telemetry.addLine("Gamepad 2: Mechanisms (Press same button to stop)");
//        telemetry.update();
//    }
//
//    /**
//     * Code to run REPEATEDLY after the driver hits START but before they hit STOP.
//     */
//    @Override
//    public void loop() {
//        // --- Gamepad 1: Drivetrain Control ---
//        handleDrivetrain();
//
//        // --- Gamepad 2: Intake and Shooter Control ---
//        handleMechanisms();
//
//        // --- Update Telemetry ---
//        updateTelemetry();
//    }
//
//    /**
//     * This method contains the joystick logic for the Mecanum drivetrain.
//     */
//    private void handleDrivetrain() {
//        // The '-' sign on left_stick_y is because the joystick's Y-axis is inverted
//        double forward = -gamepad1.left_stick_y;  // Controls forward and backward
//        double strafe  =  gamepad1.left_stick_x;  // Controls left and right strafing
//        double rotate  =  gamepad1.right_stick_x;  // Controls robot rotation
//
//        // Mecanum Drive Formulas
//        double leftFrontPower  = forward + strafe + rotate;
//        double rightFrontPower = forward - strafe - rotate;
//        double leftBackPower   = forward - strafe + rotate;
//        double rightBackPower  = forward + strafe - rotate;
//
//        // Normalization to prevent motor powers from exceeding 1.0
//        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
//        leftFront.setPower(leftFrontPower / denominator);
//        rightFront.setPower(rightFrontPower / denominator);
//        leftBack.setPower(leftBackPower / denominator);
//        rightBack.setPower(rightBackPower / denominator);
//    }
//
//    /**
//     * This method contains the advanced state machine logic for the intake and shooter.
//     */
//    /**
//     * Corrected handleMechanisms with proper edge detection
//     */
//    private void handleMechanisms() {
//        // 1. Safety Check: If Gamepad 2 is not detected, stop everything and alert driver
//        if (gamepad2.id == -1) { // -1 usually indicates no gamepad is assigned
//            telemetry.addData("WARNING", "Gamepad 2 NOT DETECTED!");
//            stopAllMechanisms();
//            return;
//        }
//
//        // --- State Selection Logic (Edge Detection) ---
//
//        // Check for 'A' button (Intake)
//        if (gamepad2.a && !a2_was_pressed) {
//            mechanismState = (mechanismState == SystemState.INTAKE_ONLY) ? SystemState.STOPPED : SystemState.INTAKE_ONLY;
//        }
//
//        // Check for Dpad Up (Shooter Only)
//        if (gamepad2.dpad_up && !dpad_up_was_pressed) {
//            mechanismState = (mechanismState == SystemState.SHOOTER_ONLY) ? SystemState.STOPPED : SystemState.SHOOTER_ONLY;
//        }
//        dpad_up_was_pressed = gamepad2.dpad_up; // Update edge detection
//
//        if (gamepad2.dpad_right && !dpad_right_was_pressed) {
//            mechanismState = (mechanismState == SystemState.REVERSE_OR_EMPTY_RIGHT_CHANNEL) ? SystemState.STOPPED : SystemState.REVERSE_OR_EMPTY_RIGHT_CHANNEL;
//        }
//        dpad_right_was_pressed = gamepad2.dpad_right; // Update edge detection
//
//        if (gamepad2.dpad_left && !dpad_left_was_pressed) {
//            mechanismState = (mechanismState == SystemState.REVERSE_OR_EMPTY_LEFT_CHANNEL) ? SystemState.STOPPED : SystemState.REVERSE_OR_EMPTY_LEFT_CHANNEL;
//        }
//        dpad_left_was_pressed = gamepad2.dpad_left; // Update edge detection
//
//
//        // Check for 'Y' button (Shooting Sequence)
//        if (gamepad2.y && !y2_was_pressed) {
//            if (mechanismState != SystemState.SHOOTING_SEQUENCE) {
//                mechanismState = SystemState.SHOOTING_SEQUENCE;
//                mechanismTimer.reset();
//            } else {
//                mechanismState = SystemState.STOPPED;
//            }
//        }
//
//        // Check for 'X' button (Left Reverse)
//        if (gamepad2.x && !x2_was_pressed) {
//            mechanismState = (mechanismState == SystemState.LEFT_REVERSED) ? SystemState.STOPPED : SystemState.LEFT_REVERSED;
//        }
//
//        // Check for 'B' button (Right Reverse)
//        if (gamepad2.b && !b2_was_pressed) {
//            mechanismState = (mechanismState == SystemState.RIGHT_REVERSED) ? SystemState.STOPPED : SystemState.RIGHT_REVERSED;
//        }
//
//        // --- Debugging: Show which button is currently physically held down ---
//        telemetry.addData("G2 Buttons", "A:%b B:%b X:%b Y:%b",
//                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y);
//
//        // Update edge detection variables for next loop
//        a2_was_pressed = gamepad2.a;
//        x2_was_pressed = gamepad2.x;
//        y2_was_pressed = gamepad2.y;
//        b2_was_pressed = gamepad2.b;
//
//        // --- State Execution Logic ---
//        switch (mechanismState) {
//            case INTAKE_ONLY:
//                runIntake();
//                break;
//            case REVERSE_OR_EMPTY_RIGHT_CHANNEL:
//                runEmptyRightChannel();
//                break;
//            case REVERSE_OR_EMPTY_LEFT_CHANNEL:
//                runEmptyLefttChannel();
//                break;
//            case SHOOTING_SEQUENCE:
//                runShootingSequence();
//                break;
//            case SHOOTER_ONLY: // Add this case
//                runShooterOnly();
//                break;
//            case LEFT_REVERSED:
//                leftConveyorBelt.setPower(-0.8);
//                leftIntakeGate.setPower(-1.0);
//                break;
//            case RIGHT_REVERSED:
//                rightConveyorBelt.setPower(-0.8);
//                rightIntakeGate.setPower(-1.0);
//                break;
//            case STOPPED:
//            default:
//                applyStopPowers();
//                break;
//        }
//    }
//
//    public void runEmptyRightChannel() {
//        intakeRoller.setPower(-INTAKE_ROLLER_POWER);
//        rightIntakeGate.setPower(-GATE_SERVO_POWER);
//        rightConveyorBelt.setPower(-RIGHT_SHOOTER_CONVEYOR_POWER);
//
//    }
//    public void runEmptyLefttChannel() {
//        intakeRoller.setPower(-INTAKE_ROLLER_POWER);
//        leftIntakeGate.setPower(-GATE_SERVO_POWER);
//        leftConveyorBelt.setPower(-LEFT_SHOOTER_CONVEYOR_POWER);
//
//    }
//    public void runShooterOnly() {
//        leftConveyorBelt.setPower(LEFT_SHOOTER_CONVEYOR_POWER);
//        rightConveyorBelt.setPower(RIGHT_SHOOTER_CONVEYOR_POWER);
//    }
//
//    // Split stopping into two methods to prevent Timer Reset loops
//    public void applyStopPowers() {
//        leftConveyorBelt.setPower(0);
//        rightConveyorBelt.setPower(0);
//        leftIntakeGate.setPower(0);
//        rightIntakeGate.setPower(0);
//        intakeRoller.setPower(0);
//    }
//
//    public void runShootingSequence() {
//        double elapsed = mechanismTimer.milliseconds();
//
//        // 1. Get current distance
//        double distInches = distanceSensor.getDistance(DistanceUnit.INCH);
//
//        // 2. Calculate Dynamic Power (y = 0.0028x + 0.60)
//        double calculatedPower;
//        // Check if out of range (> 2m) or flaky
//        if (distInches > 80 || Double.isNaN(distInches)) {
//            calculatedPower = 0.90;
//        } else {
//            calculatedPower = (distInches * 0.0028) + 0.60;
//        }
//
//        // 3. Calculate dynamicDelay (The Momentum/Wait Time)
//        double dynamicDelay;
//        if (distInches >= 105) {
//            dynamicDelay = 850; // Far range calibration
//        } else if (distInches >= 43 && distInches <= 46) {
//            dynamicDelay = 800; // Mid range calibration
//        } else {
//            // Apply the linear equation for any distance
//            // Clamping results between 750ms and 1000ms for safety
//            dynamicDelay = 900 - (distInches * 0.7936);
//            dynamicDelay = Math.max(750, Math.min(dynamicDelay, 1000));
//        }
//
//        // 4. Safety Clamps for Power
//        if (distInches < 5) calculatedPower = 0;
//        double finalPower = Math.max(0.4, Math.min(calculatedPower, 1.0));
//
//        // 5. Apply Power to Conveyors
//        leftConveyorBelt.setPower(finalPower);
//        rightConveyorBelt.setPower(finalPower);
//
//        // 6. Execute Gate Logic based on dynamicDelay
//        // The gate now opens after exactly the right amount of momentum is built
//        if (elapsed > dynamicDelay) {
//            leftIntakeGate.setPower(GATE_SERVO_POWER);
//            rightIntakeGate.setPower(GATE_SERVO_POWER);
//            intakeRoller.setPower(INTAKE_ROLLER_POWER);
//        } else {
//            leftIntakeGate.setPower(0);
//            rightIntakeGate.setPower(0);
//            intakeRoller.setPower(0);
//        }
//
//        // Telemetry for live debugging
//        telemetry.addData("Dist", "%.1f in", distInches);
//        telemetry.addData("Power", "%.2f", finalPower);
//        telemetry.addData("Target Delay", "%.0f ms", dynamicDelay);
//        telemetry.addData("Current Elapsed", "%.0f ms", elapsed);
//    }
//
//    /**
//     * Updates all telemetry data in one place.
//     */
//    private void updateTelemetry() {
//        telemetry.addData("--- Drivetrain ---", "");
//        telemetry.addData("Forward", "%.2f", -gamepad1.left_stick_y);
//        telemetry.addData("Strafe", "%.2f", gamepad1.left_stick_x);
//        telemetry.addData("Rotate", "%.2f", gamepad1.right_stick_x);
//        // Get voltage from the hardwareMap (usually the first expansion hub)
//        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//        telemetry.addData("Battery Voltage", "%.2fV", voltage);
//
//        if (voltage < 11.0) {
//            telemetry.addLine("!!! LOW BATTERY - CHANGE NOW !!!");
//        }
//
//        double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
//
//        // Filter and display distance
//        // 80 inches is approx 2.03 meters
//        if (distanceInches > 0.5 && distanceInches < 80) {
//            telemetry.addData("Alliance Dist", "%.1f in (%.1f cm)",
//                    distanceInches, distanceInches * 2.54);
//        } else {
//            telemetry.addData("Alliance Dist", "OUT OF RANGE (> 2m)");
//        }
//
//        telemetry.addData("--- Mechanisms ---", "");
//        telemetry.addData("Mechanism State", mechanismState);
//
//        telemetry.update();
//    }
//
//    // --- Helper Methods for Mechanisms ---
//
//    public void runIntake() {
//        intakeRoller.setPower(INTAKE_ROLLER_POWER);
//    }
//
//    /**
//     * Stops all conveyor and gate mechanisms.
//     */
//    public void stopAllMechanisms() {
//        applyStopPowers();
//        mechanismTimer.reset();
//        mechanismState = SystemState.STOPPED;
//    }
//
//}
//
