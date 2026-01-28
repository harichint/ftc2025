//package org.firstinspires.ftc.teamcode.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//public abstract class BaseAuto extends LinearOpMode {
//    protected DriveTrain drive;
//    protected ScoringSystem scoring;
//    protected FarHardCodedAutonomous.BallColor[] sequence;
//
//    public void setup() {
//        drive = new DriveTrain(hardwareMap);
//        scoring = new ScoringSystem(hardwareMap);
//        // ... Alliance selection loop logic ...
//    }
//
//    // This method is the same for Far and Near
//    protected void performScoring(FarHardCodedAutonomous.BallColor[] detected) throws InterruptedException {
//        for (FarHardCodedAutonomous.BallColor color : detected) {
//            if (color == FarHardCodedAutonomous.BallColor.GREEN) scoring.runLeft(1.0);
//            else scoring.runRight(1.0);
//            Thread.sleep(800);
//            scoring.stop();
//        }
//    }
//}
