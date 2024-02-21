package prime.movers;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;

public interface IPlannable extends AutoCloseable {
  public Map<String, Command> getNamedCommands();
}
