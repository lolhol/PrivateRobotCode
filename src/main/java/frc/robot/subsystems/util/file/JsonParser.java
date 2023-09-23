package frc.robot.subsystems.util.file;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.HashMap;
import java.util.Map;

public class JsonParser {

  Gson gson = new Gson();

  public JsonObject parseData(File file) {
    FileReader reader;
    try {
      reader = new FileReader(file);
    } catch (FileNotFoundException e) {
      return null;
    }

    return gson.fromJson(reader, JsonObject.class);
  }

  public Map<String, Integer> getMap(JsonObject obj, String name) {
    JsonArray nameArray = obj.getAsJsonArray(name);

    // Iterate through the JsonArray and print the contents of each JsonObject

    Map<String, Integer> returnMap = new HashMap<>();

    for (int i = 0; i < nameArray.size(); i++) {
      JsonObject nameObject = nameArray.get(i).getAsJsonObject();

      for (String key : nameObject.keySet()) {
        int value = nameObject.get(key).getAsInt();
        returnMap.put(key, value);
      }
    }

    return returnMap;
  }
}
