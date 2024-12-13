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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name  = "Slide Timing Op-Mode V0.0.1")
public class TimingOpmode extends OpMode{
    private DcMotorEx slideMotor;
    private PIDController slideController;
    private Timing.Timer timer = new Timing.Timer(999999999, TimeUnit.MILLISECONDS);
    private double p = TuningOpMode.p, i = TuningOpMode.i , d = TuningOpMode.d, f = TuningOpMode.f ;
    private final double  maxTicks = 850;
    private int slidePos = 0;

    public static double target = 0;

    private double prevTarget = 0;


    public void init(){
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideController = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void start() {
        timer.start();
    }

    public void loop() {
                p = TuningOpMode.p;
                i = TuningOpMode.i;
                d = TuningOpMode.d;
                f = TuningOpMode.f;
                target = TuningOpMode.target;
                slidePos = slideMotor.getCurrentPosition();


                if (target != prevTarget) {
                    timer.start();
                }

                if (atPos(slidePos, target)) {
                    timer.pause();
                }

                double power = slideController.calculate(slidePos, target) + f;
                slideMotor.setPower(power);

                telemetry.addData("Target Pos: ", target);
                telemetry.addData("Current Pos: ", slidePos);
                telemetry.addData("Applied Power: ", power);
                telemetry.addData("Amps: ", slideMotor.getCurrent(CurrentUnit.MILLIAMPS) / 1000);
                telemetry.addData("4.5 Amp Target: ", 4.5);
                telemetry.addData("4.0 Amp Target: ", 4.0);
                telemetry.addData("3.5 Amp Target: ", 3.5);
                telemetry.addData("3.0 Amp Target: ", 3.0);
                telemetry.addData("Elapsed Time (ms): ", timer.elapsedTime());
                telemetry.addData("Velocity, m/s: ", (slideMotor.getVelocity() / 145.1) * (30.0/35.0) * (2.0 * 0.028 * Math.PI));

                prevTarget = target;
    }

    public boolean atPos(double current, double target){
        return (Math.abs(target - current) < 10);
    }
}
