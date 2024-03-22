package prime.utilities;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Scanner;
import org.littletonrobotics.junction.LogFileUtil;

public class PrimeLogFileUtil {

  private static final String environmentVariable = "AKIT_LOG_PATH";
  private static final String advantageScopeFileName = "akit-log-path.txt";

  private PrimeLogFileUtil() {}

  public static String addPathSuffix(String path, String suffix) {
    return LogFileUtil.addPathSuffix(path, suffix);
  }

  public static String findReplayLog() {
    // Read environment variables
    String envPath = System.getenv(environmentVariable);
    if (envPath != null) {
      System.out.println("Using log from " + environmentVariable + " environment variable - \"" + envPath + "\"");
      return envPath;
    }

    // Read file from AdvantageScope
    Path advantageScopeTempPath = Paths.get(System.getProperty("java.io.tmpdir"), advantageScopeFileName);
    String advantageScopeLogPath = null;
    try (Scanner fileScanner = new Scanner(advantageScopeTempPath)) {
      advantageScopeLogPath = fileScanner.nextLine();
    } catch (IOException e) {
      System.out.println(e.getMessage());
      System.out.println("Exception occured looking for AdvantageScope log file.");
    }
    if (advantageScopeLogPath != null) {
      System.out.println("Using log from AdvantageScope - \"" + advantageScopeLogPath + "\"");
      return advantageScopeLogPath;
    }

    // Prompt on stdin
    System.out.print(
      "No log provided with the " + environmentVariable + " environment variable or through AdvantageScope."
    );
    return "";
  }
}
