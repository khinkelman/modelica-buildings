within Buildings.Fluid.Boilers.Examples;
model SteamBoilerSimple
  "Open loop example model for simple steam boiler"
  extends Modelica.Icons.Example;

  package MediumSte = Buildings.Media.Steam (
     T_default=110+273.15,
     p_default=143380) "Steam medium";
  package MediumWat = Buildings.Media.Water (
    T_default=20+273.15,
    p_default=101325) "Water medium";

  parameter Modelica.SIunits.AbsolutePressure pSat=143380
    "Nominal steam pressure";
  parameter Modelica.SIunits.Temperature TSat=
    MediumSte.saturationTemperature(pSat)
    "Steam temperature";
  parameter Modelica.SIunits.SpecificEnthalpy hfg=
    MediumSte.specificEnthalpy(state=
      MediumSte.setState_pTX(p=pSat, T=TSat, X={1})) -
    MediumWat.specificEnthalpy(state=
      MediumWat.setState_pTX(p=pSat, T=TSat, X={1}))
    "Enthalpy of vaporation";
  parameter Modelica.SIunits.Temperature TSubCoo=278.15
    "Degree of subcooling at the heating coil";

  parameter Modelica.SIunits.MassFlowRate mBoi_flow_nominal=QBoi_flow_nominal/hfg
    "Nominal mass flow rate of boiler";
  parameter Modelica.SIunits.HeatFlowRate QBoi_flow_nominal=57130.65
    "Nominal heat flow rate of boiler";
  parameter Modelica.SIunits.MassFlowRate mCoi_flow_nominal=mBoi_flow_nominal
    "Nominal mass flow rate of Coil";

  parameter Modelica.SIunits.PressureDifference dp_nominal=10000
    "Pressure drop at nominal mass flow rate in district network";
  .Buildings.Fluid.Boilers.SteamBoilerSimple boi(
    redeclare package Medium_a = MediumWat,
    redeclare package Medium_b = MediumSte,
    m_flow_nominal=mBoi_flow_nominal,
    show_T=true,
    TSat=TSat,
    pSat=pSat,
    Q_flow_nominal=QBoi_flow_nominal,
    effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
    a={0.8},
    fue=Data.Fuels.NaturalGasLowerHeatingValue()) "Boiler"
    annotation (Placement(transformation(extent={{40,20},{60,40}})));
  BaseClasses.SteamCoil coi(redeclare package Medium_a = MediumSte, redeclare
      package Medium_b = MediumWat,
    m_flow_nominal=mCoi_flow_nominal,
    show_T=true,
    TSat=TSat,
    pSat=pSat)                      "Steam coil"
    annotation (Placement(transformation(extent={{60,-80},{40,-60}})));
  Movers.FlowControlled_m_flow pum(redeclare package Medium = MediumWat,
      m_flow_nominal=mCoi_flow_nominal)
    "Condensate pump"
    annotation (Placement(transformation(extent={{20,-80},{0,-60}})));
  FixedResistances.PressureDrop preDro(
    redeclare package Medium = MediumWat,
    m_flow_nominal=mBoi_flow_nominal,
    dp_nominal=dp_nominal) "Flow resistance"
    annotation (Placement(transformation(extent={{0,-30},{20,-10}})));

  Modelica.Blocks.Math.Gain m_flow(k=QBoi_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
  Sources.Boundary_pT sou(redeclare package Medium = MediumWat, nPorts=1)
            "Source"
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  Sources.Boundary_pT sin1(redeclare package Medium = MediumWat, nPorts=1)
    "Sink" annotation (Placement(transformation(extent={{60,-30},{40,-10}})));
  Modelica.Blocks.Sources.TimeTable y(table=[0,0; 1800,1; 1800,0; 2400,0; 2400,1;
        3600,1])
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
equation
  connect(pum.port_b, preDro.port_a) annotation (Line(points={{0,-70},{-20,-70},
          {-20,-20},{0,-20}}, color={0,127,255}));
  connect(boi.port_b, coi.port_a) annotation (Line(points={{60,30},{80,30},{80,-70},
          {60,-70}},      color={0,127,255}));
  connect(coi.port_b, pum.port_a)
    annotation (Line(points={{40,-70},{20,-70}},  color={0,127,255}));
  connect(m_flow.y, pum.m_flow_in) annotation (Line(points={{-19,50},{-10,50},{-10,
          -50},{10,-50},{10,-58}},     color={0,0,127}));
  connect(sou.ports[1], boi.port_a)
    annotation (Line(points={{20,30},{40,30}}, color={0,127,255}));
  connect(preDro.port_b, sin1.ports[1])
    annotation (Line(points={{20,-20},{40,-20}}, color={0,127,255}));
  connect(y.y, boi.y) annotation (Line(points={{-59,70},{30,70},{30,38},{38,38}},
        color={0,0,127}));
  connect(y.y, m_flow.u) annotation (Line(points={{-59,70},{-50,70},{-50,50},{-42,
          50}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=3600, __Dymola_Algorithm="Dassl"));
end SteamBoilerSimple;
