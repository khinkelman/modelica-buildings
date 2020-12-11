within Buildings.Fluid.Boilers.Examples.BaseClasses.Examples;
model SteamCoil
  extends Modelica.Icons.Example;

  package MediumSte = Buildings.Media.Steam "Steam medium";
  package MediumWat = Buildings.Media.Water "Water medium";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SteamCoil;
