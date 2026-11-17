package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Properties;

/**
 * StateStore en un archivo de texto del Control Hub (FIRST/biobuzz_estado.properties).
 * Si el archivo no existe o falla, regresa el valor por omisión: nunca truena el OpMode.
 */
public class FileStateStore implements StateStore {
    private final File file;
    private final Properties props = new Properties();

    public FileStateStore() {
        this(new File(AppUtil.FIRST_FOLDER, "biobuzz_estado.properties"));
    }

    public FileStateStore(File file) {
        this.file = file;
        try (InputStream in = new FileInputStream(file)) {
            props.load(in);
        } catch (IOException ignored) {
            // primera vez o archivo dañado: se empieza vacío
        }
    }

    @Override
    public double getDouble(String key, double fallback) {
        String v = props.getProperty(key);
        if (v == null) return fallback;
        try {
            return Double.parseDouble(v);
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    @Override
    public void putDouble(String key, double value) {
        props.setProperty(key, Double.toString(value));
        try (OutputStream out = new FileOutputStream(file)) {
            props.store(out, "BIOBUZZ: estado que sobrevive entre OpModes");
        } catch (IOException ignored) {
            // sin archivo, la próxima vez se supone la posición de casa
        }
    }
}
