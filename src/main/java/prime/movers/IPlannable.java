package prime.movers;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;

public interface IPlannable extends AutoCloseable {
  /**
   * Returns a list of named commands that can be registered to PathPlanner to be triggered in Paths/Autos
   */
  public Map<String, Command> getNamedCommands();
}
