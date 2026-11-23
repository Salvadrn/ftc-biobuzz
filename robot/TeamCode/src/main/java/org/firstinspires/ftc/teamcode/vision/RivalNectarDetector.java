package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Config.IntakeCameraCfg;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

/**
 * Cámara Logitech C270 en la esquina +X del intake (CAD v116-v127; la visera salió con F2 en
 * la v106), en el USB 2.0 de la Control Hub (R708.A la permite). Busca NECTAR del color
 * rival frente al intake para no jalarla (G408). Su pose del CAD es provisional (ver
 * Config.IntakeCameraCfg): con la mitad de abajo de la imagen, cerca del robot solo cubre
 * la mitad +X de la boca. El respaldo es el V3 de la garganta.
 *
 * Usa el ColorBlobLocatorProcessor del SDK con filtros de área y redondez: una NECTAR
 * cerca del robot es un círculo grande en la mitad de abajo de la imagen; las cintas del
 * piso son alargadas y lo que está lejos sale chico.
 *
 * Si la cámara no está en la configuración, simplemente no hace nada: el intake se
 * queda con el sensor de color de la garganta. Si deja de mandar cuadros (estado distinto
 * de STREAMING), tampoco: la última lista de manchas se quedaría congelada.
 *
 * No se cierra al terminar: el SDK cierra el VisionPortal solo al parar el OpMode, y
 * close() espera sin poder interrumpirse a que la cámara acabe de abrir.
 *
 * Los rangos ColorRange.BLUE/RED del SDK son genéricos (YCrCb), no calibrados para NECTAR:
 * revisar con "Test: Sensores" y la vista en vivo (IntakeCameraCfg.LIVE_VIEW).
 */
public class RivalNectarDetector {
    private final VisionPortal portal;
    private final ColorBlobLocatorProcessor blobs;
    private int lastCount = 0;
    private double lastArea = 0;

    public RivalNectarDetector(HardwareMap hardwareMap, Alliance alliance) {
        WebcamName cam = hardwareMap.tryGet(WebcamName.class, Names.INTAKE_CAMERA);
        if (cam == null) {
            portal = null;
            blobs = null;
            return;
        }
        blobs = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(alliance == Alliance.RED ? ColorRange.BLUE : ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, IntakeCameraCfg.ROI_TOP, 1, -1))
                .setDrawContours(true)
                .setBlurSize(5)
                .build();
        blobs.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                IntakeCameraCfg.MIN_AREA_PX, IntakeCameraCfg.MAX_AREA_PX));
        blobs.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                IntakeCameraCfg.MIN_CIRCULARITY, 1.0));
        portal = new VisionPortal.Builder()
                .setCamera(cam)
                .setCameraResolution(new Size(IntakeCameraCfg.WIDTH_PX, IntakeCameraCfg.HEIGHT_PX))
                .addProcessor(blobs)
                .enableLiveView(IntakeCameraCfg.LIVE_VIEW)
                .build();
    }

    public boolean isPresent() {
        return portal != null;
    }

    /** true si hay una NECTAR rival frente al intake en el último cuadro. */
    public boolean rivalAhead() {
        if (blobs == null || portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            lastCount = 0;
            lastArea = 0;
            return false;
        }
        List<ColorBlobLocatorProcessor.Blob> found = blobs.getBlobs();
        lastCount = found.size();
        lastArea = found.isEmpty() ? 0 : found.get(0).getContourArea();
        return !found.isEmpty();
    }

    public String describe() {
        if (portal == null) return "sin cámara";
        return String.format("%s  manchas %d  área %.0f px", portal.getCameraState(), lastCount, lastArea);
    }

    /** Solo si se quiere soltar la cámara a media prueba. No llamarlo al parar. */
    public void close() {
        if (portal != null) portal.close();
    }
}
