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
  parameter Modelica.SIunits.HeatFlowRate QBoi_flow_nominal=57130.65
    "Nominal heat flow rate of boiler";
  parameter Modelica.SIunits.MassFlowRate mCoi_flow_nominal=mBoi_flow_nominal
    "Nominal mass flow rate of Coil";

  parameter Modelica.SIunits.PressureDifference dp_nominal=10000
    "Pressure drop at nominal mass flow rate in district network";
  SteamBoilerSimple boi(redeclare package Medium_a = MediumWat, redeclare
      package Medium_b = MediumSte,
    m_flow_nominal=mBoi_flow_nominal,
    TSat=TSat,
    pSat=pSat,
    Q_flow_nominal=QBoi_flow_nominal,
    effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
    fue=Data.Fuels.NaturalGasLowerHeatingValue())
                                    "Boiler"
    annotation (Placement(transformation(extent={{40,20},{60,40}})));
  BaseClasses.SteamCoil coi(redeclare package Medium_a = MediumSte, redeclare
      package Medium_b = MediumWat,
    m_flow_nominal=mCoi_flow_nominal,
    TSat=TSat,
    pSat=pSat)                      "Steam coil"
    annotation (Placement(transformation(extent={{60,-60},{40,-40}})));
  Movers.FlowControlled_m_flow pum(redeclare package Medium = MediumWat)
    "Condensate pump"
    annotation (Placement(transformation(extent={{20,-60},{0,-40}})));
  FixedResistances.PressureDrop preDro(
    redeclare package Medium = MediumWat,
    m_flow_nominal=mBoi_flow_nominal,
    dp_nominal=dp_nominal) "Flow resistance"
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  Modelica.Blocks.Sources.CombiTimeTable datRea(
    tableOnFile=true,
    fileName=ModelicaServices.ExternalReferences.loadResource("modelica://Buildings//Resources/Data/Fluid/Boilers/Examples/SteamBoilerSimpleLoop_EnergyPlus.dat"),
    verboseRead=false,
    columns=2:15,
    tableName="EnergyPlus",
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments)
    "Reader for \"CoolingTower_VariableSpeed_Merkel.idf\" energy plus example results"
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));

  Modelica.Blocks.Math.Gain m_flow(k=1/hfg) "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
  Modelica.Blocks.Math.Gain plr(k=1/QBoi_flow_nominal) "Gain to calculate PLR"
    annotation (Placement(transformation(extent={{0,60},{20,80}})));
equation
  connect(pum.port_b, preDro.port_a) annotation (Line(points={{0,-50},{-20,-50},
          {-20,30},{0,30}},   color={0,127,255}));
  connect(preDro.port_b, boi.port_a)
    annotation (Line(points={{20,30},{40,30}}, color={0,127,255}));
  connect(boi.port_b, coi.port_a) annotation (Line(points={{60,30},{80,30},{80,
          -50},{60,-50}}, color={0,127,255}));
  connect(coi.port_b, pum.port_a)
    annotation (Line(points={{40,-50},{20,-50}},  color={0,127,255}));
  connect(datRea.y[12], m_flow.u) annotation (Line(points={{-59,70},{-52,70},{
          -52,50},{-42,50}}, color={0,0,127}));
  connect(m_flow.y, pum.m_flow_in) annotation (Line(points={{-19,50},{-10,50},{
          -10,-30},{10,-30},{10,-38}}, color={0,0,127}));
  connect(datRea.y[12], plr.u)
    annotation (Line(points={{-59,70},{-2,70}}, color={0,0,127}));
  connect(plr.y, boi.y) annotation (Line(points={{21,70},{30,70},{30,38},{38,38}},
        color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SteamBoilerSimpleLoop;
