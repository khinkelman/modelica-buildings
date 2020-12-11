within Buildings.Fluid.Boilers.Examples.BaseClasses.Examples;
model SteamTrap
  extends Modelica.Icons.Example;

  package MediumWat = Buildings.Media.Water "Water medium";

  parameter Modelica.SIunits.Temperature TSat=273.15+110
     "Saturation temperature";
  parameter Modelica.SIunits.AbsolutePressure pSat=143380
     "Saturation pressure";

  parameter Modelica.SIunits.MassFlowRate m_flow_nominal=10
    "Nominal mass flow rate";
  Buildings.Fluid.Boilers.Examples.BaseClasses.SteamTrap steTra(redeclare
      package Medium = MediumWat,
    m_flow_nominal=m_flow_nominal,
    show_T=true)                  "Steam trap"
    annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
  Sources.Boundary_pT sou(
    redeclare package Medium = MediumWat,
    p=pSat,
    T=TSat,
    nPorts=1)
            "Source"
    annotation (Placement(transformation(extent={{-80,0},{-60,20}})));
  Sources.Boundary_pT sin(redeclare package Medium = MediumWat, nPorts=1)
                                                                "Sink"
    annotation (Placement(transformation(extent={{60,0},{40,20}})));
  Modelica.Blocks.Sources.Ramp ram(
    height=m_flow_nominal,
    duration(displayUnit="min") = 60,
    startTime(displayUnit="min") = 60)
                                   "Ramp"
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  Movers.FlowControlled_m_flow pum(redeclare package Medium = MediumWat,
      m_flow_nominal=m_flow_nominal,
    nominalValuesDefineDefaultPressureCurve=true)
                                     "Pump"
    annotation (Placement(transformation(extent={{0,0},{20,20}})));
equation
  connect(sou.ports[1], steTra.port_a)
    annotation (Line(points={{-60,10},{-40,10}}, color={0,127,255}));
  connect(steTra.port_b, pum.port_a)
    annotation (Line(points={{-20,10},{0,10}}, color={0,127,255}));
  connect(pum.port_b, sin.ports[1])
    annotation (Line(points={{20,10},{40,10}}, color={0,127,255}));
  connect(ram.y, pum.m_flow_in)
    annotation (Line(points={{-59,70},{10,70},{10,22}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
  experiment(Tolerance=1e-6, StopTime=180.0),
__Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/BaseClasses/Examples/SteamTrap.mos"
        "Simulate and plot"));
end SteamTrap;
