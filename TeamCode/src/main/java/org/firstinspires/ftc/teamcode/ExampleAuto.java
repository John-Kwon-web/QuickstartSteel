package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto", group = "Examples")
public class ExampleAuto extends OpMode {

    /* ==================== PEDRO OBJECTS ==================== */

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    /* ==================== FIELD POSES ==================== */

    private final Pose startPose   = new Pose(28.5, 128, Math.toRadians(180));

    private final Pose scorePose   = new Pose(60, 85, Math.toRadians(135));

    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));

    /* ==================== PATHS ==================== */

    private Path scorePreload;

    private PathChain grabPickup1, scorePickup1;
    private PathChain grabPickup2, scorePickup2;
    private PathChain grabPickup3, scorePickup3;

    /* ==================== BUILD PATHS ==================== */

    public void buildPaths() {

        // Score preload (straight line)
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(
                startPose.getHeading(),
                scorePose.getHeading()
        );

        // Cycle 1: Grab Pickup 1
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(
                        scorePose.getHeading(),
                        pickup1Pose.getHeading()
                )
                .build();

        // Cycle 1: Score Pickup 1
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(
                        pickup1Pose.getHeading(),
                        scorePose.getHeading()
                )
                .build();

        // Cycle 2: Grab Pickup 2
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(
                        scorePose.getHeading(),
                        pickup2Pose.getHeading()
                )
                .build();

        // Cycle 2: Score Pickup 2
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(
                        pickup2Pose.getHeading(),
                        scorePose.getHeading()
                )
                .build();

        // Cycle 3: Grab Pickup 3
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(
                        scorePose.getHeading(),
                        pickup3Pose.getHeading()
                )
                .build();

        // Cycle 3: Score Pickup 3
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(
                        pickup3Pose.getHeading(),
                        scorePose.getHeading()
                )
                .build();
    }

    /* ==================== FSM PATH STATE MACHINE ==================== */

    public void autonomousPathUpdate() {

        switch (pathState) {

            // ---- SCORE PRELOAD ----
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            // ---- GO PICKUP 1 ----
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;

            // ---- SCORE PICKUP 1 ----
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;

            // ---- GO PICKUP 2 ----
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;

            // ---- SCORE PICKUP 2 ----
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;

            // ---- GO PICKUP 3 ----
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;

            // ---- SCORE PICKUP 3 ----
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;

            // ---- DONE ----
            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1); // stop running
                }
                break;
        }
    }

    /* ==================== STATE SETTER ==================== */

    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    /* ==================== MAIN LOOP ==================== */

    @Override
    public void loop() {

        // REQUIRED: update Pedro follower every loop
        follower.update();

        // Run FSM state machine
        autonomousPathUpdate();

        // Telemetry Debug
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /* ==================== INIT ==================== */

    @Override
    public void init() {

        // Timers
        pathTimer = new Timer();
        actionTimer = new Timer();   // ✅ FIXED
        opmodeTimer = new Timer();

        // Create follower
        follower = Constants.createFollower(hardwareMap);

        // Build all paths BEFORE starting
        buildPaths();

        // Set starting pose
        follower.setStartingPose(startPose);

        telemetry.addLine("Example Auto Ready!");
        telemetry.update();
    }

    /* ==================== START ==================== */

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0); // begin auto
    }

    /* ==================== STOP ==================== */

    @Override
    public void stop() {
        // Auto ends automatically
    }
}
