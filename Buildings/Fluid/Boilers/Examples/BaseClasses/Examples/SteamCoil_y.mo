within Buildings.Fluid.Boilers.Examples.BaseClasses.Examples;
model SteamCoil_y
  extends Modelica.Icons.Example;

  package MediumSte = Buildings.Media.Steam (
    p_default=143380) "Steam medium";
  package MediumWat = Buildings.Media.Water "Water medium";

  parameter Modelica.SIunits.MassFlowRate m_flow_nominal = 1
    "Nominal mass flow rate";
  parameter Modelica.SIunits.PressureDifference dp_nominal = 500
    "Nominal pressure difference";

  parameter Modelica.SIunits.AbsolutePressure pSte=143380
    "Nominal steam pressure";
  parameter Modelica.SIunits.Temperature TSte=
    MediumSte.saturationTemperature(pSte)
    "Steam temperature";
  Buildings.Fluid.Boilers.Examples.BaseClasses.SteamCoil coi(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=m_flow_nominal,
    show_T=true,
    TSat=TSte,
    pSat=pSte,
    dp_nominal=dp_nominal/2)                                 "Steam coil"
    annotation (Placement(transformation(extent={{-50,0},{-30,20}})));
  Sources.Boundary_pT sou(
    redeclare package Medium = MediumSte,
    p=pSte,
    T=TSte,
    nPorts=1)
            "Source"
    annotation (Placement(transformation(extent={{-80,0},{-60,20}})));
  Modelica.Blocks.Sources.Ramp ram(
    height=m_flow_nominal - 0.01,
    duration(displayUnit="min") = 60,
    offset=0.01,
    startTime(displayUnit="min") = 60)
                                   "Ramp"
    annotation (Placement(transformation(extent={{-92,-70},{-72,-50}})));
  Sources.Boundary_pT sin(redeclare package Medium = MediumWat,
    p(displayUnit="Pa") = 101325,
    nPorts=1)
    "Sink" annotation (Placement(transformation(extent={{90,0},{70,20}})));
  Movers.SpeedControlled_y                 pum(
    redeclare package Medium = MediumWat,
    per(pressure(V_flow={0,m_flow_nominal,2*m_flow_nominal}/1000, dp={
            dp_nominal*2,dp_nominal,0})),
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Pump"
    annotation (Placement(transformation(extent={{40,0},{60,20}})));
  Sensors.MassFlowRate                 senMasFlo(redeclare package Medium =
        MediumWat)
    annotation (Placement(transformation(extent={{10,0},{30,20}})));
  Modelica.Blocks.Math.Gain gain1(k=1/m_flow_nominal)
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
  Controls.Continuous.LimPID           conPID(
    Td=1,
    k=0.5,
    Ti=15) annotation (Placement(transformation(extent={{10,60},{30,80}})));
  Modelica.Blocks.Sources.Pulse y(
    offset=0.25,
    startTime=0,
    amplitude=0.5,
    period=15*60) "Input signal"
                 annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  FixedResistances.PressureDrop dp(
    redeclare package Medium = MediumWat,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=dp_nominal/2)
                           "Pressure drop"
    annotation (Placement(transformation(extent={{-20,0},{0,20}})));
equation
  connect(sou.ports[1], coi.port_a)
    annotation (Line(points={{-60,10},{-50,10}}, color={0,127,255}));
  connect(y.y, gain1.u)
    annotation (Line(points={{-59,70},{-42,70}}, color={0,0,127}));
  connect(gain1.y, conPID.u_s)
    annotation (Line(points={{-19,70},{8,70}}, color={0,0,127}));
  connect(conPID.u_m, senMasFlo.m_flow)
    annotation (Line(points={{20,58},{20,21}}, color={0,0,127}));
  connect(conPID.y, pum.y)
    annotation (Line(points={{31,70},{50,70},{50,22}}, color={0,0,127}));
  connect(sin.ports[1], pum.port_b)
    annotation (Line(points={{70,10},{60,10}}, color={0,127,255}));
  connect(pum.port_a, senMasFlo.port_b)
    annotation (Line(points={{40,10},{30,10}}, color={0,127,255}));
  connect(senMasFlo.port_a, dp.port_b)
    annotation (Line(points={{10,10},{0,10}}, color={0,127,255}));
  connect(dp.port_a, coi.port_b)
    annotation (Line(points={{-20,10},{-30,10}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
  experiment(Tolerance=1e-6, StopTime=180.0),
__Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/BaseClasses/Examples/SteamCoil.mos"
        "Simulate and plot"));
end SteamCoil_y;
