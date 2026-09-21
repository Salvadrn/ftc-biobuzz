package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;

@TeleOp(name = "TeleOp ROJO", group = "1 Competencia")
public class TeleOpRed extends BiobuzzTeleOp {
    @Override
    protected Alliance alliance() { return Alliance.RED; }
}
