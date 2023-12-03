package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;

public final class revUltis {
    private revUltis() {}

    public static void checkNeoError(REVLibError error, String message) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
        }
    }
}
