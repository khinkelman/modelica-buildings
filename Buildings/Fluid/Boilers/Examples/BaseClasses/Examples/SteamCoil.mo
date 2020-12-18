within Buildings.Fluid.Boilers.Examples.BaseClasses.Examples;
model SteamCoil
  extends Modelica.Icons.Example;

  package MediumSte = Buildings.Media.Steam (
    p_default=143380) "Steam medium";
  package MediumWat = Buildings.Media.Water "Water medium";

  parameter Modelica.SIunits.MassFlowRate m_flow_nominal = 1
    "Nominal mass flow rate";

  parameter Modelica.SIunits.AbsolutePressure pSte=143380
    "Nominal steam pressure";
  parameter Modelica.SIunits.Temperature TSte=
    MediumSte.saturationTemperature(pSte)
    "Steam temperature";
  parameter Modelica.SIunits.Temperature TSatLow=273.15+100
     "High pressure saturation temperature";
  parameter Modelica.SIunits.PressureDifference dp=pSte-101325
    "Prescribed pressure change";
  Buildings.Fluid.Boilers.Examples.BaseClasses.SteamCoil coi(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=m_flow_nominal,
    show_T=true,
    TSatHig=TSte,
    pSat=pSte,
    TSatLow=TSatLow,
    dpCoi_nominal=dp)                                        "Steam coil"
    annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  Sources.Boundary_pT sou(
    redeclare package Medium = MediumSte,
    p=pSte,
    T=TSte,
    nPorts=1)
            "Source"
    annotation (Placement(transformation(extent={{-60,0},{-40,20}})));
  Modelica.Blocks.Sources.Ramp ram(
    height=m_flow_nominal - 0.01,
    duration(displayUnit="min") = 60,
    offset=0.01,
    startTime(displayUnit="min") = 60)
                                   "Ramp"
    annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
  Movers.FlowControlled_m_flow pum(
    redeclare package Medium = MediumWat,
    m_flow_nominal=m_flow_nominal,
    nominalValuesDefineDefaultPressureCurve=true)
                                     "Pump"
    annotation (Placement(transformation(extent={{20,0},{40,20}})));
  Sources.Boundary_pT sin(redeclare package Medium = MediumWat,
    p=pSte - dp,                                                nPorts=1)
    "Sink" annotation (Placement(transformation(extent={{80,0},{60,20}})));
equation
  connect(sou.ports[1], coi.port_a)
    annotation (Line(points={{-40,10},{-20,10}}, color={0,127,255}));
  connect(coi.port_b, pum.port_a)
    annotation (Line(points={{0,10},{20,10}}, color={0,127,255}));
  connect(pum.port_b, sin.ports[1])
    annotation (Line(points={{40,10},{60,10}}, color={0,127,255}));
  connect(ram.y, pum.m_flow_in)
    annotation (Line(points={{-39,70},{30,70},{30,22}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
  experiment(Tolerance=1e-6, StopTime=180.0),
__Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/BaseClasses/Examples/SteamCoil.mos"
        "Simulate and plot"));
end SteamCoil;
