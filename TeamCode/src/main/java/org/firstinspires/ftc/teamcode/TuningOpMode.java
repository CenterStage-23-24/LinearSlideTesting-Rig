package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name  = "Tuning Op-Mode")
public class TuningOpMode extends OpMode{
    private DcMotorEx slideMotor;
    private PIDController slideController;
    //Drum Only Constants
    //public static double p = 0.01, i = 0, d= 0.0001, f =0 ;
    public static double p = 0.00, i = 0, d= 0.0000, f =0 ;


    public static double target = 0;


    public void init(){
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideController = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void loop() {
        slideController.setPID(p, i, d);
        double slidePos = slideMotor.getCurrentPosition();

        double power = slideController.calculate(slidePos, target) + f;
        slideMotor.setPower(power);

        telemetry.addData("Target Pos: ", target);
        telemetry.addData("Current Pos: ", slidePos);
        telemetry.addData("Power: ", power);
    }
}
