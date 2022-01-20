package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="DBDist")
public class Linear extends LinearOpMode {
    DcMotor fl,fr,br,bl;
    DistanceSensor ds;

    @Override
    public void runOpMode()
    {

        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        ds = hardwareMap.get(DistanceSensor.class, "ds");

        while(!isStarted()&&!isStopRequested())
        {

        }
        if(opModeIsActive())
        {
            driveByDistance(ds.getDistance(DistanceUnit.CM),2.7,1);
            setPowerZero();
        }
    }

    public boolean threshold(double val, double otherval, double range)
    {
        return (val>( otherval + range)||val<( otherval - range));
    }

    public void setPowerAll(double power)
    {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    public void setPowerZero()
    {
        setPowerAll(0);
    }

    public void driveByDistance(double initial, double desired, double multiplier)
    {
        double divisor = desired - initial;
        double power = .06;
        while(opModeIsActive()&&threshold(ds.getDistance(DistanceUnit.CM),desired, .3) && Math.abs(power)>.055)
        {
            power = ((ds.getDistance(DistanceUnit.CM)-desired)/divisor)*multiplier;
            setPowerAll(power);


        }
        setPowerZero();
    }
}
