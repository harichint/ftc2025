package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is the main TeleOp program that combines the Drivetrain, Intake, and Shooter systems.
 * Gamepad 1 controls the robot's movement (Mecanum drive).
 * Gamepad 2 controls the Intake and Shooter mechanisms with an advanced stateful toggle system.
 */
@TeleOp(name = "Main Program (Drive + Mechanisms)", group = "Main")
public class MainProgram extends OpMode {

    // --- DRIVETRAIN HARDWARE ---
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // --- INTAKE & SHOOTER HARDWARE ---
    private DcMotor intakeRoller;
    private CRServo leftintakeGate;
    private DcMotor leftconveyorBelt;

    private CRServo rightintakeGate;
    private DcMotor rightconveyorBelt;


    // --- MECHANISM STATE MACHINE ---
    private enum SystemState {
        STOPPED,
        REVERSED,
        INTAKE_ONLY,
        LEFT_SHOOTER_ONLY,
        LEFT_SERVO,
        RIGHT_SHOOTER_ONLY,
        RIGHT_SERVO

        //INTAKE_AND_SHOOTER
    }
    private SystemState mechanismState = SystemState.STOPPED;

    // --- Button Edge Detection (for smart toggling) ---
    private boolean a2_was_pressed = false;
    private boolean b2_was_pressed = false;
    private boolean y2_was_pressed = false;
    private boolean x2_was_pressed = false;

    private boolean dpad_left_was_pressed = false;
    private boolean dpad_up_was_pressed = false;
    private boolean dpad_down_was_pressed = false;

    // --- MECHANISM CONSTANTS ---
    private static final double INTAKE_ROLLER_POWER = 1.0;
    private static final double SHOOTER_CONVEYOR_POWER = 0.8;
    private static final double GATE_SERVO_POWER = 1.0;
    private static final double STOP_POWER = 0.0;


    /**
     * Code to run ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
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

        // --- Initialize Intake & Shooter ---
        intakeRoller = hardwareMap.get(DcMotor.class, "intake_roller");
        leftintakeGate = hardwareMap.get(CRServo.class, "left_intake_gate");
        leftconveyorBelt = hardwareMap.get(DcMotor.class, "left_conveyor_belt");
        rightintakeGate = hardwareMap.get(CRServo.class, "right_intake_gate");
        rightconveyorBelt = hardwareMap.get(DcMotor.class, "right_conveyor_belt");

        // Set Intake/Shooter motor directions (TUNE THIS FOR YOUR ROBOT)
        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        leftconveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftintakeGate.setDirection(CRServo.Direction.FORWARD);
        leftintakeGate.setPower(GATE_SERVO_POWER);

        rightconveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightintakeGate.setDirection(CRServo.Direction.FORWARD);
        rightintakeGate.setPower(GATE_SERVO_POWER);

        // Ensure all mechanisms are stopped on initialization
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
     */
    private void handleMechanisms() {
        // This section updates the desired state based on single button presses ("edges").

        // Check for 'A' button press
        if (gamepad2.a && !a2_was_pressed) {
            mechanismState = (mechanismState == SystemState.INTAKE_ONLY) ? SystemState.STOPPED : SystemState.INTAKE_ONLY;
        }

        // Check for 'B' button press
        if (gamepad2.b && !b2_was_pressed) {
            mechanismState = (mechanismState == SystemState.LEFT_SHOOTER_ONLY) ? SystemState.STOPPED : SystemState.LEFT_SHOOTER_ONLY;
        }

        // Check for 'Y' button press
        if (gamepad2.y && !y2_was_pressed) {
            mechanismState = (mechanismState == SystemState.LEFT_SERVO) ? SystemState.STOPPED : SystemState.LEFT_SERVO;
        }

//         Check for 'X' button press (Outtake/Reverse)
        if (gamepad2.x && !x2_was_pressed) {
            mechanismState = (mechanismState == SystemState.REVERSED) ? SystemState.STOPPED : SystemState.REVERSED;
        }

        // Check for 'dpad_left' button press
        if (gamepad2.dpad_left && !dpad_left_was_pressed) {
            mechanismState = (mechanismState == SystemState.RIGHT_SHOOTER_ONLY) ? SystemState.STOPPED : SystemState.RIGHT_SHOOTER_ONLY;
        }

        // Check for 'dpad_up' button press
        if (gamepad2.dpad_up && !dpad_up_was_pressed) {
            mechanismState = (mechanismState == SystemState.RIGHT_SERVO) ? SystemState.STOPPED : SystemState.RIGHT_SERVO;
        }


        // Update the 'was_pressed' state for the next loop cycle
        a2_was_pressed = gamepad2.a;
        b2_was_pressed = gamepad2.b;
        y2_was_pressed = gamepad2.y;
        x2_was_pressed = gamepad2.x;
        dpad_left_was_pressed = gamepad2.dpad_left;
        dpad_up_was_pressed = gamepad2.dpad_up;


        // This switch statement continuously executes the action based on the current state.
        switch (mechanismState) {
            case INTAKE_ONLY:
                runIntake();
                stopleftServo();
                stopleftShooter();
                break;
            case LEFT_SHOOTER_ONLY:
                runleftShooter();
                stopleftServo();
                stopIntake();
                break;
            case LEFT_SERVO:
                leftintakeGate.setPower(1.0); // Force full power
                intakeRoller.setPower(0);
                leftconveyorBelt.setPower(0);
                break;
            case RIGHT_SHOOTER_ONLY:
                runrightShooter();
                stoprightServo();
                stopIntake();
                break;
            case RIGHT_SERVO:
                rightintakeGate.setPower(1.0); // Force full power
                intakeRoller.setPower(0);
                rightconveyorBelt.setPower(0);
                break;
            case REVERSED:
                runOuttake();
                stopleftShooter();
                stopIntake();
                stopleftServo();
                break;
            case STOPPED:
            default:
                stopAllMechanisms();
                break;
        }
    }

    /**
     * Updates all telemetry data in one place.
     */
    private void updateTelemetry() {
        telemetry.addData("--- Drivetrain ---", "");
        telemetry.addData("Forward", "%.2f", -gamepad1.left_stick_y);
        telemetry.addData("Strafe", "%.2f", gamepad1.left_stick_x);
        telemetry.addData("Rotate", "%.2f", gamepad1.right_stick_x);
        telemetry.addData("--- Mechanisms ---", "");
        telemetry.addData("Mechanism State", mechanismState);
        telemetry.update();
    }

    // --- Helper Methods for Mechanisms ---

    public void runIntake() {
        intakeRoller.setPower(INTAKE_ROLLER_POWER);
    }

    public void runServo() {
        // Ensure the power is explicitly set
        leftintakeGate.setPower(GATE_SERVO_POWER);
    }

    public void stopleftServo() {
        leftintakeGate.setPower(STOP_POWER);
    }

    public void stoprightServo() {
        rightintakeGate.setPower(STOP_POWER);
    }

    public void runleftShooter() {
        leftconveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
        leftintakeGate.setPower(GATE_SERVO_POWER);
    }

    public void runrightShooter() {
        rightconveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
        rightintakeGate.setPower(GATE_SERVO_POWER);
    }

    public void runOuttake() {
        // For outtake, all components run in reverse
        intakeRoller.setPower(-INTAKE_ROLLER_POWER);
        leftintakeGate.setPower(-1.0);
    }

    public void stopIntake() {
        intakeRoller.setPower(STOP_POWER);
    }

    public void stopleftShooter() {
        leftconveyorBelt.setPower(STOP_POWER);
        leftintakeGate.setPower(STOP_POWER);
    }
    public void stoprightShooter() {
        rightconveyorBelt.setPower(STOP_POWER);
        rightintakeGate.setPower(STOP_POWER);
    }

    public void stopAllMechanisms() {
        stopIntake();
        stopleftShooter();
        stopleftServo();
        stoprightShooter();
        stoprightServo();
    }
}
