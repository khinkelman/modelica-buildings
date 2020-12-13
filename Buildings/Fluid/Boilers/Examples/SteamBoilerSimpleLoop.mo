within Buildings.Fluid.Boilers.Examples;
model SteamBoilerSimpleLoop "Loop example based on EnergyPlus"
  extends Modelica.Icons.Example;

  package MediumSte = Buildings.Media.Steam (
     T_default=110+273.15,
     p_default=143380) "Steam medium";
  package MediumWat = Buildings.Media.Water "Water medium";

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
  parameter Modelica.SIunits.HeatFlowRate QBoi_flow_nominal=QCoi1_flow_nominal+QCoi2_flow_nominal
    "Nominal heat flow rate of boiler";
  parameter Modelica.SIunits.MassFlowRate mCoi1_flow_nominal=QCoi1_flow_nominal/hfg
    "Nominal mass flow rate of Coil";
  parameter Modelica.SIunits.MassFlowRate mCoi2_flow_nominal=QCoi2_flow_nominal/hfg
    "Nominal mass flow rate of Coil";
  parameter Modelica.SIunits.HeatFlowRate QCoi1_flow_nominal=20000
    "Nominal heat flow rate of boiler";
  parameter Modelica.SIunits.HeatFlowRate QCoi2_flow_nominal=50000
    "Nominal heat flow rate of boiler";

  parameter Modelica.SIunits.PressureDifference dp_nominal=10000
    "Pressure drop at nominal mass flow rate in district network";
  .Buildings.Fluid.Boilers.SteamBoilerSimple boi(
    redeclare package Medium_a = MediumWat,
    redeclare package Medium_b = MediumSte,
    m_flow_nominal=mBoi_flow_nominal,
    TSat=TSat,
    pSat=pSat,
    Q_flow_nominal=QBoi_flow_nominal,
    effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
    a={0.8},
    fue=Data.Fuels.NaturalGasLowerHeatingValue()) "Boiler"
    annotation (Placement(transformation(extent={{40,20},{60,40}})));
  BaseClasses.SteamCoil coi2(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=mCoi2_flow_nominal,
    TSat=TSat,
    pSat=pSat) "Steam coil"
    annotation (Placement(transformation(extent={{60,-30},{40,-10}})));
  Movers.FlowControlled_m_flow pum2(redeclare package Medium = MediumWat,
      m_flow_nominal=mCoi2_flow_nominal)
                                        "Condensate pump"
    annotation (Placement(transformation(extent={{20,-30},{0,-10}})));
  FixedResistances.PressureDrop preDro(
    redeclare package Medium = MediumWat,
    m_flow_nominal=mBoi_flow_nominal,
    dp_nominal=dp_nominal) "Flow resistance"
    annotation (Placement(transformation(extent={{0,20},{20,40}})));

  Modelica.Blocks.Sources.TimeTable y1(table=[0,0; 1800,1; 1800,0; 2400,0; 2400,
        1; 3600,1])
    annotation (Placement(transformation(extent={{-90,-50},{-70,-30}})));
  Modelica.Blocks.Math.Gain m2_flow(k=QCoi2_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-48,-10},{-28,10}})));
  Movers.FlowControlled_m_flow pum1(redeclare package Medium = MediumWat,
      m_flow_nominal=mCoi1_flow_nominal)
                                        "Condensate pump"
    annotation (Placement(transformation(extent={{20,-70},{0,-50}})));
  BaseClasses.SteamCoil coi1(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=mCoi1_flow_nominal,
    TSat=TSat,
    pSat=pSat) "Steam coil"
    annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
  Modelica.Blocks.Sources.TimeTable y2(table=[0,0; 1200,0.2; 1200,0.8; 3000,0.8;
        3000,0.3; 3600,0.3])
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  Modelica.Blocks.Math.Gain m1_flow(k=QCoi1_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-48,-50},{-28,-30}})));
  Modelica.Blocks.Math.Product pro "Product"
    annotation (Placement(transformation(extent={{-50,30},{-30,50}})));
equation
  connect(pum2.port_b, preDro.port_a) annotation (Line(points={{0,-20},{-10,-20},
          {-10,30},{0,30}}, color={0,127,255}));
  connect(preDro.port_b, boi.port_a)
    annotation (Line(points={{20,30},{40,30}}, color={0,127,255}));
  connect(boi.port_b, coi2.port_a) annotation (Line(points={{60,30},{70,30},{70,
          -20},{60,-20}}, color={0,127,255}));
  connect(coi2.port_b, pum2.port_a)
    annotation (Line(points={{40,-20},{20,-20}}, color={0,127,255}));
  connect(m2_flow.y, pum2.m_flow_in)
    annotation (Line(points={{-27,0},{10,0},{10,-8}}, color={0,0,127}));
  connect(boi.port_b, coi1.port_a) annotation (Line(points={{60,30},{70,30},{70,
          -60},{60,-60}}, color={0,127,255}));
  connect(pum1.port_b, preDro.port_a) annotation (Line(points={{0,-60},{-10,-60},
          {-10,30},{0,30}}, color={0,127,255}));
  connect(coi1.port_b, pum1.port_a)
    annotation (Line(points={{40,-60},{20,-60}}, color={0,127,255}));
  connect(y2.y, m2_flow.u)
    annotation (Line(points={{-69,0},{-50,0}}, color={0,0,127}));
  connect(y1.y, m1_flow.u) annotation (Line(points={{-69,-40},{-62,-40},{-62,-40},
          {-50,-40}}, color={0,0,127}));
  connect(m1_flow.y, pum1.m_flow_in)
    annotation (Line(points={{-27,-40},{10,-40},{10,-48}}, color={0,0,127}));
  connect(y2.y, pro.u1) annotation (Line(points={{-69,0},{-66,0},{-66,46},{-52,46}},
        color={0,0,127}));
  connect(y1.y, pro.u2) annotation (Line(points={{-69,-40},{-60,-40},{-60,34},{-52,
          34}}, color={0,0,127}));
  connect(pro.y, boi.y)
    annotation (Line(points={{-29,40},{38,40},{38,39}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(Tolerance=1e-6, StopTime=3600.0),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/SteamBoilerSimpleLoop.mos"
        "Simulate and plot"));
end SteamBoilerSimpleLoop;
