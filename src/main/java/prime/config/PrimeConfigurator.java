package prime.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FilenameFilter;

/**
 * A utility class providing static methods for configuring the robot from a JSON file
 */
public class PrimeConfigurator {

  /**
   * Retreives a list of configuration files in the /deploy folder
   * @return
   */
  public static String[] getAvailableConfigsInDeploy() {
    FilenameFilter jsonFilter = (dir, name) ->
      name.toLowerCase().endsWith(".json");

    return Filesystem.getDeployDirectory().list(jsonFilter);
  }

  /**
   * Uses an ObjectMapper to bind properties from a JSON configuration file into an instance of a given class
   * @param <T> The class type to bind to
   * @param type The class type to bind to
   * @param pathToFile The JSON file to bind properties from
   * @return
   */
  public static <T> T mapConfigFromJsonFile(Class<T> type, String pathToFile) {
    try {
      var mapper = new ObjectMapper();
      var file = new File(pathToFile);
      if (!file.exists()) throw new Exception(
        "File does not exist at path: \"" + pathToFile + "\""
      );

      T config = mapper.readValue(file, type);

      return config;
    } catch (Exception e) {
      DriverStation.reportError(
        "[ERROR] >> Failed to map config: " + e.getMessage(),
        e.getStackTrace()
      );
      return null;
    }
  }
}
