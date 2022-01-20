package org.firstinspires.ftc.teamcode;/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "TestBot", group = "Example")
//@Disabled
public class TestBot extends OpMode {

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    DcMotor frMotor, flMotor, brMotor, blMotor;
    DistanceSensor ds;


    @Override
    public void init() {
        frMotor = hardwareMap.dcMotor.get("fr");
        flMotor = hardwareMap.dcMotor.get("fl");
        brMotor = hardwareMap.dcMotor.get("br");
        blMotor = hardwareMap.dcMotor.get("bl");

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ds = hardwareMap.get(DistanceSensor.class, "ds");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void init_loop() {
        telemetry.addData("distance cm: ", ds.getDistance(DistanceUnit.CM));
        telemetry.update();
    }


    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {
        flMotor.setPower(gamepad1.left_stick_y);
        blMotor.setPower(gamepad1.left_stick_y);
        frMotor.setPower(gamepad1.right_stick_y);
        brMotor.setPower(gamepad1.right_stick_y);
        telemetry.addData("distance cm: ", ds.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
    @Override
    public void stop() {

    }


}
