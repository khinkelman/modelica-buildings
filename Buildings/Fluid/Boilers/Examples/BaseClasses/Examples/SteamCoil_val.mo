within Buildings.Fluid.Boilers.Examples.BaseClasses.Examples;
model SteamCoil_val
  extends Modelica.Icons.Example;

  package MediumSte = Buildings.Media.Steam (
    p_default=143380,
    T_default=110+273.15) "Steam medium";
  package MediumWat = Buildings.Media.Water (
    T_default=90+273.15,
    p_default=101325) "Water medium";

  parameter Modelica.SIunits.MassFlowRate m_flow_nominal = 1
    "Nominal mass flow rate";
  parameter Modelica.SIunits.PressureDifference dp_nominal=6000
    "Pressure drop at nominal mass flow rate";

  parameter Modelica.SIunits.AbsolutePressure pSte=143380
    "Nominal steam pressure";
  parameter Modelica.SIunits.Temperature TSte=
    MediumSte.saturationTemperature(pSte)
    "Steam temperature";

  parameter MediumSte.ThermodynamicState staSte_default=MediumSte.setState_pTX(
      T=MediumSte.T_default, p=MediumSte.p_default, X=MediumSte.X_default);
  parameter Modelica.SIunits.Density rhoSte_default=MediumSte.density(staSte_default)
    "Density, used to compute rhoStd in val";

  Buildings.Fluid.Boilers.Examples.BaseClasses.SteamCoil coi(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=m_flow_nominal,
    show_T=true,
    TSatHig=TSte,
    pSat=pSte)                                 "Steam coil"
    annotation (Placement(transformation(extent={{-30,0},{-10,20}})));
  Sources.Boundary_pT sou(
    redeclare package Medium = MediumSte,
    p=pSte,
    T=TSte,
    nPorts=1)
            "Source"
    annotation (Placement(transformation(extent={{-90,0},{-70,20}})));
  Sources.Boundary_pT sin(redeclare package Medium = MediumWat,
    T=363.15,                                                   nPorts=1)
    "Sink" annotation (Placement(transformation(extent={{90,0},{70,20}})));
  Actuators.Valves.TwoWayLinear val(
    redeclare package Medium = MediumSte,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=dp_nominal/2,
    rhoStd=rhoSte_default,
    riseTime=60,
    dpFixed_nominal=dp_nominal/2)
    annotation (Placement(transformation(extent={{-60,0},{-40,20}})));
  Controls.Continuous.LimPID conCoi(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.5,
    Ti=15)
          "Coil flow controller"
    annotation (Placement(transformation(extent={{30,40},{50,60}})));
  Sensors.MassFlowRate senMasFlo(redeclare package Medium = MediumWat)
    annotation (Placement(transformation(extent={{30,0},{50,20}})));
  Modelica.Blocks.Sources.Ramp mSet_flow(
    height=m_flow_nominal,
    duration(displayUnit="min") = 1200,
    startTime(displayUnit="min") = 1200)
    annotation (Placement(transformation(extent={{-10,40},{10,60}})));
  FixedResistances.CheckValve cheVal(
    redeclare package Medium = MediumWat,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=1)
    annotation (Placement(transformation(extent={{0,0},{20,20}})));
equation
  connect(sou.ports[1], val.port_a)
    annotation (Line(points={{-70,10},{-60,10}}, color={0,127,255}));
  connect(val.port_b, coi.port_a)
    annotation (Line(points={{-40,10},{-30,10}}, color={0,127,255}));
  connect(senMasFlo.port_b, sin.ports[1])
    annotation (Line(points={{50,10},{70,10}}, color={0,127,255}));
  connect(senMasFlo.m_flow, conCoi.u_m)
    annotation (Line(points={{40,21},{40,38}}, color={0,0,127}));
  connect(mSet_flow.y, conCoi.u_s)
    annotation (Line(points={{11,50},{28,50}}, color={0,0,127}));
  connect(conCoi.y, val.y) annotation (Line(points={{51,50},{60,50},{60,70},{-50,
          70},{-50,22}}, color={0,0,127}));
  connect(coi.port_b, cheVal.port_a)
    annotation (Line(points={{-10,10},{0,10}}, color={0,127,255}));
  connect(cheVal.port_b, senMasFlo.port_a)
    annotation (Line(points={{20,10},{30,10}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
  experiment(StopTime=3600, Tolerance=1e-06),
__Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/BaseClasses/Examples/SteamCoil.mos"
        "Simulate and plot"));
end SteamCoil_val;
