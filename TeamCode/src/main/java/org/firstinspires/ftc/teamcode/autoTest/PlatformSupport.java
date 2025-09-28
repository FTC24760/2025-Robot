package org.firstinspires.ftc.teamcode.autoTest;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class PlatformSupport {

    public static long getCurrentTimeMillis() {
        return System.currentTimeMillis();
    }

    public static void waitForSimulatorTimeStep() {
        //does nothing in real world robot
    }

    public static boolean isSimulator() {
        return false;
    }

    public static File getSettingsFile(String filename) {
        return AppUtil.getInstance().getSettingsFile(filename);
    }

    public static String readFile(File file) {
        return ReadWriteFile.readFile(file);
    }
}
