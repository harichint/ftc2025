package org.firstinspires.ftc.teamcode.auto;

public interface ShootStrategy {
    void execute(ScoringSystem scoring);
}

// Example for SEQ_2 logic
//public class Seq2Strategy implements ShootStrategy {
//    public void execute(ScoringSystem scoring) {
//        // Custom logic: Shoot Green, then run roller for purple
//        scoring.runLeft(1.0);
//        // ... sleeps ...
//        scoring.setRoller(1.0);
//    }
//}
