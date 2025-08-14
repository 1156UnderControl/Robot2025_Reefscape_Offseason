package frc.Java_Is_UnderControl.Util;

import com.fasterxml.jackson.databind.*;
import com.fasterxml.jackson.databind.node.MissingNode;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;

public final class JsonUtil {
  private static final ObjectMapper mapper = new ObjectMapper();

  private static JsonNode at(String variablePath, String jsonPathName) {
    JsonNode root;
    try {
      Path p = Filesystem.getDeployDirectory().toPath().resolve(jsonPathName);
      root = mapper.readTree(p.toFile());
    } catch (Exception e) {
      e.printStackTrace();
      root = MissingNode.getInstance();
    }
    
    return root.at(variablePath); // nunca lança NPE; retorna MissingNode se não existir
  }

  public static String getString(String variablePath, String jsonPath) {
    JsonNode n = at(variablePath, jsonPath);
    return n.asText();
  }

  public static int getInt(String variablePath, String jsonPath) {
    JsonNode n = at(variablePath, jsonPath);
    return n.asInt();
  }

  public static double getDouble(String variablePath, String jsonPath) {
    JsonNode n = at(variablePath, jsonPath);
    return n.asDouble();
  }

  public boolean getBool(String variablePath, String jsonPath) {
    JsonNode n = at(variablePath, jsonPath);
    return n.asBoolean();
  }
}