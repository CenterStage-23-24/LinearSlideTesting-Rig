package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name  = "Slide Testing Op-Mode V0.0.1")
public class TestingOpMode extends OpMode{
    private DcMotorEx slideMotor;
    private PIDController slideController;
    private Timing.Timer uptimeTimer = new Timing.Timer(999999999, TimeUnit.MINUTES);
    private Timing.Timer stateTimer = new Timing.Timer(999999999, TimeUnit.MINUTES);
    private final double p = 0.02, i = 0, d= 0.00015, f =0.05 ;
    private final double  maxTicks = 850;
    private int atPosCounter= 0;

    private double prevTarget = 0;
    private double target = 0;
    private int positionsReached = 0;

    private double totalDistanceTicks;

    public enum State {
        active,
        resting
    };

    State state = State.active;



    public void init(){
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideController = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void start() {
        uptimeTimer.start();
        stateTimer.start();
    }

    public void loop() {
        State startState = state;
        switch (state) {
            case active:
                uptimeTimer.resume();
                double slidePos = slideMotor.getCurrentPosition();

                if (atPos(slidePos, target)) {
                    atPosCounter++;
                } else {
                    atPosCounter = 0;
                }

                if (atPosCounter >= 15){
                    totalDistanceTicks += Math.abs(target - prevTarget);
                    prevTarget = target;
                    target = Math.random() * maxTicks;
                    positionsReached++;
                }
                double power = slideController.calculate(slidePos, target) + f;
                slideMotor.setPower(power);

                telemetry.addData("Target Pos: ", target);
                telemetry.addData("Current Pos: ", slidePos);
                telemetry.addData("Power: ", power);
                telemetry.addData("Ticks Traveled: ", totalDistanceTicks);
                telemetry.addData("Positions Reached: ", positionsReached);
                telemetry.addData("Uptime: ", uptimeTimer.elapsedTime());
                telemetry.addData("State Time: ", stateTimer.elapsedTime());

                if (stateTimer.elapsedTime() >= 5) {
                    state = State.resting;
                }
                break;

            case resting:
                uptimeTimer.pause();
                telemetry.addData("Resting Time: ", stateTimer.elapsedTime());
                slideMotor.setPower(0);

                if (stateTimer.elapsedTime() >= 5) {
                    state = State.active;
                }
                break;
        }

        if (state != startState) {
            stateTimer.start();
        }
    }

    public boolean atPos(double current, double target){
        return (Math.abs(target - current) < 10);
    }
}
