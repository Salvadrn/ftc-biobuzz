package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Break-beam Adafruit 2168 (receptor con salida de colector abierto) o cualquier otra
 * entrada digital; el sentido se configura: activeHigh = true si "activo" se lee como true.
 *
 * Break-beam con pull-up: haz libre = el receptor jala a tierra = false; haz cortado
 * (hay pelota) = true.
 */
public class BeamSensor {
    private final DigitalChannel channel;
    private final boolean activeHigh;

    public BeamSensor(HardwareMap hardwareMap, String name, boolean activeHigh) {
        this(hardwareMap.get(DigitalChannel.class, name), activeHigh);
    }

    public BeamSensor(DigitalChannel channel, boolean activeHigh) {
        this.channel = channel;
        this.activeHigh = activeHigh;
        channel.setMode(DigitalChannel.Mode.INPUT);
    }

    /** true = haz cortado (break-beam) o imán presente (switch magnético). */
    public boolean isActive() {
        return channel.getState() == activeHigh;
    }
}
