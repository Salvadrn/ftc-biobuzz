package com.qualcomm.hardware.limelightvision;

import org.json.JSONObject;

/**
 * Solo para la simulación: arma un LLResult REAL del SDK a partir de un JSON, como si
 * hubiera llegado del Limelight hace stalenessMs milisegundos. Vive en el paquete del SDK
 * porque parse() y setControlHubTimeStamp() no son públicos.
 */
public final class LLFactory {
    private LLFactory() {}

    public static LLResult make(JSONObject json, long stalenessMs) {
        LLResult r = LLResult.parse(json);
        if (r != null) r.setControlHubTimeStamp(System.currentTimeMillis() - stalenessMs);
        return r;
    }
}
