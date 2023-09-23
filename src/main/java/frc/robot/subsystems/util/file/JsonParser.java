package frc.robot.subsystems.util.file;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import frc.robot.config.util.enums.Axises;
import frc.robot.config.util.enums.Buttons;
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

    System.out.println(reader + "!!!!!!");

    return gson.fromJson(reader, JsonObject.class);
  }

  public Map<Axises, Integer> getMapAxis(
    JsonObject obj,
    String name,
    String subString
  ) {
    if (obj != null) {
      JsonArray nameArray = obj.getAsJsonObject(name).getAsJsonArray(subString);

      Map<Axises, Integer> returnMap = new HashMap<>();
      for (int i = 0; i < nameArray.size(); i++) {
        JsonObject nameObject = nameArray.get(i).getAsJsonObject();

        for (Map.Entry<String, JsonElement> entry : nameObject.entrySet()) {
          String axisName = entry.getKey();
          Axises axis = Axises.valueOf(axisName);

          int value = entry.getValue().getAsInt();
          returnMap.put(axis, value);
        }
      }

      return returnMap;
    } else {
      System.out.println("OBJECT = NULL!");
    }

    return null;
  }

  public Map<Buttons, Integer> getMapButton(
    JsonObject obj,
    String name,
    String subString
  ) {
    if (obj != null) {
      JsonArray nameArray = obj.getAsJsonObject(name).getAsJsonArray(subString);

      Map<Buttons, Integer> returnMap = new HashMap<>();
      for (int i = 0; i < nameArray.size(); i++) {
        JsonObject nameObject = nameArray.get(i).getAsJsonObject();

        for (Map.Entry<String, JsonElement> entry : nameObject.entrySet()) {
          String buttonName = entry.getKey();
          Buttons button = Buttons.valueOf(buttonName);

          int value = entry.getValue().getAsInt();
          returnMap.put(button, value);
        }
      }

      return returnMap;
    } else {
      System.out.println("OBJECT = NULL!");
    }

    return null;
  }
}
