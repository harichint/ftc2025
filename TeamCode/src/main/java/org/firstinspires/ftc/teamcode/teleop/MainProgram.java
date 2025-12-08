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
    private CRServo intakeGate;
    private DcMotor conveyorBelt;

    // --- MECHANISM STATE MACHINE ---
    private enum SystemState {
        STOPPED,
        REVERSED,
        INTAKE_ONLY,
        SHOOTER_ONLY,
        INTAKE_AND_SHOOTER
    }
    private SystemState mechanismState = SystemState.STOPPED;

    // --- Button Edge Detection (for smart toggling) ---
    private boolean a2_was_pressed = false;
    private boolean b2_was_pressed = false;
    private boolean y2_was_pressed = false;
    private boolean x2_was_pressed = false;

    // --- MECHANISM CONSTANTS ---
    private static final double INTAKE_ROLLER_POWER = 1.0;
    private static final double SHOOTER_CONVEYOR_POWER = 1.0;
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
        intakeGate = hardwareMap.get(CRServo.class, "intake_gate");
        conveyorBelt = hardwareMap.get(DcMotor.class, "conveyor_belt");

        // Set Intake/Shooter motor directions (TUNE THIS FOR YOUR ROBOT)
        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeGate.setDirection(DcMotorSimple.Direction.FORWARD);

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
            mechanismState = (mechanismState == SystemState.SHOOTER_ONLY) ? SystemState.STOPPED : SystemState.SHOOTER_ONLY;
        }

        // Check for 'Y' button press
        if (gamepad2.y && !y2_was_pressed) {
            mechanismState = (mechanismState == SystemState.INTAKE_AND_SHOOTER) ? SystemState.STOPPED : SystemState.INTAKE_AND_SHOOTER;
        }

        // Check for 'X' button press (Outtake/Reverse)
        if (gamepad2.x && !x2_was_pressed) {
            mechanismState = (mechanismState == SystemState.REVERSED) ? SystemState.STOPPED : SystemState.REVERSED;
        }

        // Update the 'was_pressed' state for the next loop cycle
        a2_was_pressed = gamepad2.a;
        b2_was_pressed = gamepad2.b;
        y2_was_pressed = gamepad2.y;
        x2_was_pressed = gamepad2.x;


        // This switch statement continuously executes the action based on the current state.
        switch (mechanismState) {
            case INTAKE_ONLY:
                runIntake();
                stopShooter();
                break;
            case SHOOTER_ONLY:
                runShooter();
                stopIntake();
                break;
            case INTAKE_AND_SHOOTER:
                runIntake();
                runShooter();
                break;
            case REVERSED:
                runOuttake();
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

    public void runShooter() {
        conveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
        intakeGate.setPower(GATE_SERVO_POWER);
    }

    public void runOuttake() {
        // For outtake, all components run in reverse
        intakeRoller.setPower(-INTAKE_ROLLER_POWER);
        conveyorBelt.setPower(-SHOOTER_CONVEYOR_POWER);
        intakeGate.setPower(-GATE_SERVO_POWER);
    }

    public void stopIntake() {
        intakeRoller.setPower(STOP_POWER);
    }

    public void stopShooter() {
        conveyorBelt.setPower(STOP_POWER);
        intakeGate.setPower(STOP_POWER);
    }

    public void stopAllMechanisms() {
        stopIntake();
        stopShooter();
    }
}
