within Buildings.Fluid.Boilers.Examples;
model SteamBoilerSimpleLoop "Loop example based on EnergyPlus"
  extends Modelica.Icons.Example;

  package MediumSte = IBPSA.Media.Steam (
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
    show_T=true,
    TSat=TSat,
    pSat=pSat,
    Q_flow_nominal=QBoi_flow_nominal,
    effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
    a={0.8},
    fue=Data.Fuels.NaturalGasLowerHeatingValue()) "Boiler"
    annotation (Placement(transformation(extent={{30,20},{50,40}})));
  BaseClasses.SteamCoil coi2(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=mCoi2_flow_nominal,
    TSat=TSat,
    pSat=pSat) "Steam coil"
    annotation (Placement(transformation(extent={{50,-30},{30,-10}})));
  Movers.FlowControlled_m_flow pum2(redeclare package Medium = MediumWat,
      m_flow_nominal=mCoi2_flow_nominal)
                                        "Condensate pump"
    annotation (Placement(transformation(extent={{10,-30},{-10,-10}})));
  FixedResistances.PressureDrop preDro(
    redeclare package Medium = MediumWat,
    m_flow_nominal=mBoi_flow_nominal,
    dp_nominal=dp_nominal) "Flow resistance"
    annotation (Placement(transformation(extent={{-10,20},{10,40}})));

  Modelica.Blocks.Sources.TimeTable y1(table=[0,0; 1800,1; 1800,0; 2400,0; 2400,
        1; 3600,1])
    annotation (Placement(transformation(extent={{-100,-50},{-80,-30}})));
  Modelica.Blocks.Math.Gain m2_flow(k=QCoi2_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  Movers.FlowControlled_m_flow pum1(redeclare package Medium = MediumWat,
      m_flow_nominal=mCoi1_flow_nominal)
                                        "Condensate pump"
    annotation (Placement(transformation(extent={{10,-70},{-10,-50}})));
  BaseClasses.SteamCoil coi1(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=mCoi1_flow_nominal,
    show_T=true,
    TSat=TSat,
    pSat=pSat) "Steam coil"
    annotation (Placement(transformation(extent={{50,-70},{30,-50}})));
  Modelica.Blocks.Sources.TimeTable y2(table=[0,0; 1200,0.2; 1200,0.8; 3000,0.8;
        3000,0.3; 3600,0.3])
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  Modelica.Blocks.Math.Gain m1_flow(k=QCoi1_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-58,-50},{-38,-30}})));
  Modelica.Blocks.Math.Product pro "Product"
    annotation (Placement(transformation(extent={{-60,28},{-40,48}})));
  HeatExchangers.Heater_T supHea(
    redeclare package Medium = MediumSte,
    m_flow_nominal=mBoi_flow_nominal,
    show_T=true,
    dp_nominal=10000) "Super heater"
    annotation (Placement(transformation(extent={{70,20},{90,40}})));
  Modelica.Blocks.Sources.Constant conTSet(k=130 + 273.15)
    "Constant temperature setpoint"
    annotation (Placement(transformation(extent={{30,60},{50,80}})));
equation
  connect(pum2.port_b, preDro.port_a) annotation (Line(points={{-10,-20},{-20,
          -20},{-20,30},{-10,30}},
                            color={0,127,255}));
  connect(preDro.port_b, boi.port_a)
    annotation (Line(points={{10,30},{30,30}}, color={0,127,255}));
  connect(coi2.port_b, pum2.port_a)
    annotation (Line(points={{30,-20},{10,-20}}, color={0,127,255}));
  connect(m2_flow.y, pum2.m_flow_in)
    annotation (Line(points={{-37,0},{0,0},{0,-8}},   color={0,0,127}));
  connect(pum1.port_b, preDro.port_a) annotation (Line(points={{-10,-60},{-20,
          -60},{-20,30},{-10,30}},
                            color={0,127,255}));
  connect(coi1.port_b, pum1.port_a)
    annotation (Line(points={{30,-60},{10,-60}}, color={0,127,255}));
  connect(y2.y, m2_flow.u)
    annotation (Line(points={{-79,0},{-60,0}}, color={0,0,127}));
  connect(y1.y, m1_flow.u) annotation (Line(points={{-79,-40},{-60,-40}},
                      color={0,0,127}));
  connect(m1_flow.y, pum1.m_flow_in)
    annotation (Line(points={{-37,-40},{0,-40},{0,-48}},   color={0,0,127}));
  connect(y2.y, pro.u1) annotation (Line(points={{-79,0},{-76,0},{-76,44},{-62,
          44}},
        color={0,0,127}));
  connect(y1.y, pro.u2) annotation (Line(points={{-79,-40},{-70,-40},{-70,32},{
          -62,32}},
                color={0,0,127}));
  connect(pro.y, boi.y)
    annotation (Line(points={{-39,38},{28,38}},         color={0,0,127}));
  connect(boi.port_b, supHea.port_a)
    annotation (Line(points={{50,30},{70,30},{70,30}}, color={0,127,255}));
  connect(supHea.port_b, coi2.port_a) annotation (Line(points={{90,30},{100,30},
          {100,-20},{50,-20}}, color={0,127,255}));
  connect(supHea.port_b, coi1.port_a) annotation (Line(points={{90,30},{100,30},
          {100,-60},{50,-60}}, color={0,127,255}));
  connect(conTSet.y, supHea.TSet) annotation (Line(points={{51,70},{60,70},{60,
          38},{68,38}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,
            -100},{120,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,
            100}})),
    experiment(Tolerance=1e-6, StopTime=3600.0),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/SteamBoilerSimpleLoop.mos"
        "Simulate and plot"));
end SteamBoilerSimpleLoop;
