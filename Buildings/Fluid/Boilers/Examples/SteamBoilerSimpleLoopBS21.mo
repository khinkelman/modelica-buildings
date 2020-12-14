within Buildings.Fluid.Boilers.Examples;
model SteamBoilerSimpleLoopBS21 "Loop example based on EnergyPlus"
  extends Modelica.Icons.Example;

  package MediumSte = Buildings.Media.Steam (
     T_default=110+273.15,
     p_default=143380) "Steam medium";
  package MediumWat = Buildings.Media.Water (
     T_default=90+273.15,
     p_default=101325) "Water medium";

  parameter Modelica.SIunits.AbsolutePressure pSat=143380
    "Nominal steam pressure";
  parameter Modelica.SIunits.Temperature TSat=
    MediumSte.saturationTemperature(pSat)
    "Steam temperature";
  parameter Modelica.SIunits.Temperature TSte=130 + 273.15
    "Steam superheated temperature";
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

   // Table parameters
  parameter Boolean tableOnFile=true
    "= true, if table is defined on file or in function usertab";
  parameter String tableName="HeatingLoadProfiles"
    "Table name on file or in function usertab (see docu)";
  parameter Modelica.Blocks.Types.Smoothness smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments
    "Smoothness of table interpolation";
  parameter Modelica.SIunits.Time timeScale(
    min=Modelica.Constants.eps)=1 "Time scale of first table column";

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
    annotation (Placement(transformation(extent={{10,20},{30,40}})));
  BaseClasses.SteamCoil coi2(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=mCoi2_flow_nominal,
    TSat=TSat,
    pSat=pSat) "Steam coil"
    annotation (Placement(transformation(extent={{30,-30},{10,-10}})));
  Movers.FlowControlled_m_flow pum2(redeclare package Medium = MediumWat,
      m_flow_nominal=mCoi2_flow_nominal)
                                        "Condensate pump"
    annotation (Placement(transformation(extent={{-10,-30},{-30,-10}})));
  FixedResistances.PressureDrop preDro(
    redeclare package Medium = MediumWat,
    m_flow_nominal=mBoi_flow_nominal,
    dp_nominal=dp_nominal) "Flow resistance"
    annotation (Placement(transformation(extent={{-30,20},{-10,40}})));

  Modelica.Blocks.Sources.TimeTable y1(table=[0,0.01; 1800,1; 2000,1; 2400,0.2;
        2600,0.2; 3600,0.9])
    annotation (Placement(transformation(extent={{-150,-50},{-130,-30}})));
  Modelica.Blocks.Math.Gain m2_flow(k=QCoi2_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-108,-10},{-88,10}})));
  Movers.FlowControlled_m_flow pum1(redeclare package Medium = MediumWat,
      m_flow_nominal=mCoi1_flow_nominal)
                                        "Condensate pump"
    annotation (Placement(transformation(extent={{-10,-70},{-30,-50}})));
  BaseClasses.SteamCoil coi1(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=mCoi1_flow_nominal,
    show_T=true,
    TSat=TSat,
    pSat=pSat) "Steam coil"
    annotation (Placement(transformation(extent={{30,-70},{10,-50}})));
  Modelica.Blocks.Sources.TimeTable y2(table=[0,0.01; 1200,0.2; 1500,0.8; 2500,
        0.8; 3000,0.3; 3600,0.3])
    annotation (Placement(transformation(extent={{-150,-10},{-130,10}})));
  Modelica.Blocks.Math.Gain m1_flow(k=QCoi1_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-108,-50},{-88,-30}})));
  Modelica.Blocks.Math.Product pro "Product"
    annotation (Placement(transformation(extent={{-110,40},{-90,60}})));
  HeatExchangers.Heater_T supHea(
    redeclare package Medium = MediumSte,
    m_flow_nominal=mBoi_flow_nominal,
    show_T=true,
    dp_nominal=10000,
    T_start=TSat)     "Super heater"
    annotation (Placement(transformation(extent={{50,20},{70,40}})));
  Modelica.Blocks.Sources.Constant conTSet(k=TSte)
    "Constant temperature setpoint"
    annotation (Placement(transformation(extent={{10,60},{30,80}})));
  HeatTransfer.Sources.FixedTemperature           gnd(T=(55 + 273.15) + 273.15)
    "Ground temperature"
    annotation (Placement(transformation(extent={{60,60},{80,80}})));
  FixedResistances.PlugFlowPipe pipSte(
    redeclare package Medium = MediumSte,
    dh=0.1,
    length=20,
    dIns=0.05,
    kIns=0.028,
    m_flow_nominal=mBoi_flow_nominal,
    cPip=500,
    thickness=0.0032,
    rhoPip=8000,
    T_start_in=TSte,
    T_start_out=TSte,
    nPorts=1) "Steam pipe"
    annotation (Placement(transformation(extent={{80,20},{100,40}})));
  FixedResistances.Junction spl(
    redeclare package Medium = MediumSte,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    p_start=pSat,
    T_start=TSte,
    m_flow_nominal=mBoi_flow_nominal*{1,-1/2,-1/2},
    dp_nominal=6000*{1,-1,-1}) "Splitter"
    annotation (Placement(transformation(extent={{110,20},{130,40}})));
  FixedResistances.Junction jun(
    redeclare package Medium = MediumWat,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=mBoi_flow_nominal*{1,1/2,-1/2},
    dp_nominal=6000*{1,1,-1}) "Junction"
    annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
  Modelica.Blocks.Sources.CombiTimeTable QHea(
    tableOnFile=tableOnFile,
    tableName=tableName,
    fileName=ModelicaServices.ExternalReferences.loadResource(
        "modelica://Buildings/Resources/Data/Fluid/Boilers/Examples/SteamBoilerSimpleLoopBS21/HeatingLoadProfiles.csv"),

    columns={2,10},
    smoothness=smoothness,
    timeScale=timeScale) "Heating demand"
    annotation (Placement(transformation(extent={{-140,70},{-120,90}})));

equation
  connect(preDro.port_b, boi.port_a)
    annotation (Line(points={{-10,30},{10,30}},color={0,127,255}));
  connect(coi2.port_b, pum2.port_a)
    annotation (Line(points={{10,-20},{-10,-20}},color={0,127,255}));
  connect(m2_flow.y, pum2.m_flow_in)
    annotation (Line(points={{-87,0},{-20,0},{-20,-8}},
                                                      color={0,0,127}));
  connect(coi1.port_b, pum1.port_a)
    annotation (Line(points={{10,-60},{-10,-60}},color={0,127,255}));
  connect(y2.y, m2_flow.u)
    annotation (Line(points={{-129,0},{-110,0}},
                                               color={0,0,127}));
  connect(y1.y, m1_flow.u) annotation (Line(points={{-129,-40},{-110,-40}},
                      color={0,0,127}));
  connect(m1_flow.y, pum1.m_flow_in)
    annotation (Line(points={{-87,-40},{-20,-40},{-20,-48}},
                                                           color={0,0,127}));
  connect(y2.y, pro.u1) annotation (Line(points={{-129,0},{-126,0},{-126,56},{-112,
          56}},
        color={0,0,127}));
  connect(y1.y, pro.u2) annotation (Line(points={{-129,-40},{-120,-40},{-120,44},
          {-112,44}},
                color={0,0,127}));
  connect(pro.y, boi.y)
    annotation (Line(points={{-89,50},{0,50},{0,38},{8,38}},
                                                        color={0,0,127}));
  connect(boi.port_b, supHea.port_a)
    annotation (Line(points={{30,30},{50,30}},         color={0,127,255}));
  connect(conTSet.y, supHea.TSet) annotation (Line(points={{31,70},{40,70},{40,38},
          {48,38}},     color={0,0,127}));
  connect(supHea.port_b, pipSte.port_a)
    annotation (Line(points={{70,30},{80,30}}, color={0,127,255}));
  connect(gnd.port, pipSte.heatPort)
    annotation (Line(points={{80,70},{90,70},{90,40}}, color={191,0,0}));
  connect(pipSte.ports_b[1],spl. port_1)
    annotation (Line(points={{100,30},{110,30}}, color={0,127,255}));
  connect(spl.port_3, coi2.port_a)
    annotation (Line(points={{120,20},{120,-20},{30,-20}}, color={0,127,255}));
  connect(spl.port_2, coi1.port_a) annotation (Line(points={{130,30},{134,30},{134,
          -60},{30,-60}}, color={0,127,255}));
  connect(jun.port_2, preDro.port_a)
    annotation (Line(points={{-40,30},{-30,30}}, color={0,127,255}));
  connect(jun.port_3, pum2.port_b) annotation (Line(points={{-50,20},{-50,-20},{
          -30,-20}}, color={0,127,255}));
  connect(pum1.port_b, jun.port_1) annotation (Line(points={{-30,-60},{-70,-60},
          {-70,30},{-60,30}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{160,100}})),
    experiment(Tolerance=1e-6, StopTime=3600.0),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/SteamBoilerSimpleLoopBS21.mos"
        "Simulate and plot"));
end SteamBoilerSimpleLoopBS21;
