package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SequenceReaderOpMode;

import java.util.Arrays;

@Autonomous(name = "Main Autonomous")
public class MainAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Access the single static 'result' object from SequenceReaderOpMode
        SequenceReaderOpMode.BallColor detectedAlliance = SequenceReaderOpMode.result.alliance;
        SequenceReaderOpMode.BallColor[] detectedSequence = SequenceReaderOpMode.result.sequence;

        // Display the plan during the Init phase of this OpMode
        telemetry.addLine("--- Ready to Run Autonomous ---");
        telemetry.addData("Executing for Alliance", detectedAlliance);
        telemetry.addData("Using Detected Sequence", Arrays.toString(detectedSequence));
        telemetry.update();

        waitForStart();

        // Now, use the variables to decide your autonomous path
        if (detectedAlliance == SequenceReaderOpMode.BallColor.BLUE) {
            // Execute blue alliance path...or use blue goal post
        } else if (detectedAlliance == SequenceReaderOpMode.BallColor.RED) {
            // Execute red alliance path...ie use red goal post
        }

        // Check the sequence if it was found
        if (detectedSequence.length > 0) {
            // Example: check the first element of the sequence
            if (detectedSequence[0] == SequenceReaderOpMode.BallColor.PURPLE) {
                // Do something for purple...
            }
        } else {
            // Default path if no sequence tag was seen
        }

        // ... rest of your autonomous code ...
    }
}
