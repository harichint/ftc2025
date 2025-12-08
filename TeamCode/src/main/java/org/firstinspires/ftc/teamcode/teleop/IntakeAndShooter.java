package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Intake and Shooter Combined", group = "Main Teleop")
public class IntakeAndShooter extends OpMode {

    // --- HARDWARE MEMBERS ---
    private DcMotor intakeRoller;
    private CRServo intakeGate;
    private DcMotor conveyorBelt;

    // --- STATE MACHINE FOR CONTROL ---
    private enum SystemState {
        STOPPED,
        INTAKE_ONLY,
        SHOOTER_ONLY,
        INTAKE_AND_SHOOTER
    }
    private SystemState currentState = SystemState.STOPPED;

    // --- CONSTANTS ---
    private static final double INTAKE_ROLLER_POWER = 1.0;
    private static final double SHOOTER_CONVEYOR_POWER = 1.0;
    private static final double GATE_SERVO_POWER = 1.0;
    private static final double STOP_POWER = 0.0;

    @Override
    public void init() {
        // Initialize all hardware components
        intakeRoller = hardwareMap.get(DcMotor.class, "intake_roller");
        intakeGate = hardwareMap.get(CRServo.class, "intake_gate");
        conveyorBelt = hardwareMap.get(DcMotor.class, "conveyor_belt");

        // --- Set Motor & Servo Directions ---
        intakeRoller.setDirection(DcMotorSimple.Direction.FORWARD); // Spins to pull balls in
        intakeGate.setDirection(DcMotorSimple.Direction.FORWARD);   // Spins to help feed conveyor
        conveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD); // Spins to push balls up/out

        // Ensure everything is stopped on initialization
        stopAll();

        telemetry.addLine("Robot Initialized: Intake & Shooter");
        telemetry.addLine("Press a button to select a mode:");
        telemetry.addLine("A = Intake | B = Shooter | Y = Both | X = Stop");
        telemetry.update();
    }

    @Override
    public void loop() {
        // =============================== THE FIX ===============================
        // This section now only updates the desired state based on button presses.
        // The action happens in the switch statement below.
        if (gamepad2.x) {
            currentState = SystemState.STOPPED;
        } else if (gamepad2.y) {
            currentState = SystemState.INTAKE_AND_SHOOTER;
        } else if (gamepad2.a) {
            currentState = SystemState.INTAKE_ONLY;
        } else if (gamepad2.b) {
            currentState = SystemState.SHOOTER_ONLY;
        }

        // This switch statement continuously executes the action based on the current state.
        switch (currentState) {
            case INTAKE_ONLY:
                runIntake();
                stopShooter(); // Ensure shooter is off
                telemetry.addData("Mode", "INTAKE ONLY (Latching)");
                break;
            case SHOOTER_ONLY:
                runShooter();
                stopIntake(); // Ensure intake is off
                telemetry.addData("Mode", "SHOOTER ONLY (Latching)");
                break;
            case INTAKE_AND_SHOOTER:
                runIntake();
                runShooter();
                telemetry.addData("Mode", "INTAKE + SHOOTER (Latching)");
                break;
            case STOPPED:
            default:
                stopAll();
                telemetry.addData("Mode", "STOPPED");
                break;
        }
        // =======================================================================

        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure everything is stopped when the OpMode is terminated.
        stopAll();
    }

    // --- Helper Methods for Controlling Mechanisms ---

    /**
     * Activates the intake-specific components (the front roller).
     */
    public void runIntake() {
        intakeRoller.setPower(INTAKE_ROLLER_POWER);
    }

    /**
     * Activates the shooter-specific components (conveyor and gate servo).
     */
    public void runShooter() {
        conveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
        intakeGate.setPower(GATE_SERVO_POWER);
    }

    /**
     * Deactivates only the intake components.
     */
    public void stopIntake() {
        intakeRoller.setPower(STOP_POWER);
    }

    /**
     * Deactivates only the shooter components.
     */
    public void stopShooter() {
        conveyorBelt.setPower(STOP_POWER);
        intakeGate.setPower(STOP_POWER);
    }

    /**
     * Stops all motors and servos related to intake and shooting.
     */
    public void stopAll() {
        stopIntake();
        stopShooter();
    }
}
