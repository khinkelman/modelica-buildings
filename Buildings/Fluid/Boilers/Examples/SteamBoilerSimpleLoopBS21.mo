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
  parameter Modelica.SIunits.HeatFlowRate QCoi1_flow_nominal=19347.2793
    "Nominal heat flow rate of boiler";
  parameter Modelica.SIunits.HeatFlowRate QCoi2_flow_nominal=19347.2793
    "Nominal heat flow rate of boiler";

  parameter Modelica.SIunits.PressureDifference dp_nominal=10000
    "Pressure drop at nominal mass flow rate in district network";

  parameter Buildings.Fluid.Movers.Data.Generic per(
   pressure(V_flow={0,mCoi1_flow_nominal,2*mCoi1_flow_nominal}/1000,
                   dp={2*dp_nominal,dp_nominal,0}))
    "Performance data for primary pumps";

//  per(pressure(V_flow={0,m_flow_nominal,2*m_flow_nominal}/1.2,
//                   dp={2*dp_nominal,dp_nominal,0}));

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
    fue=Data.Fuels.NaturalGasLowerHeatingValue(),
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
                                                  "Boiler"
    annotation (Placement(transformation(extent={{20,20},{40,40}})));
  BaseClasses.SteamCoil coi2(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=mCoi2_flow_nominal,
    TSat=TSat,
    pSat=pSat) "Steam coil"
    annotation (Placement(transformation(extent={{40,-30},{20,-10}})));
  Movers.FlowControlled_m_flow pum2(redeclare package Medium = MediumWat,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      m_flow_nominal=mCoi2_flow_nominal,
    per=per)                            "Condensate pump"
    annotation (Placement(transformation(extent={{0,-30},{-20,-10}})));
  FixedResistances.PressureDrop preDro(
    redeclare package Medium = MediumWat,
    m_flow_nominal=mBoi_flow_nominal,
    dp_nominal=dp_nominal) "Flow resistance"
    annotation (Placement(transformation(extent={{-20,20},{0,40}})));

  Modelica.Blocks.Sources.TimeTable y1(table=[0,0.01; 1800,1; 2000,1; 2400,0.2;
        2600,0.2; 3600,0.9], timeScale=24)
    annotation (Placement(transformation(extent={{-150,-50},{-130,-30}})));
  Modelica.Blocks.Math.Gain m2_flow(k=QCoi1_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Movers.FlowControlled_m_flow pum1(redeclare package Medium = MediumWat,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      m_flow_nominal=mCoi1_flow_nominal,
    per=per)                            "Condensate pump"
    annotation (Placement(transformation(extent={{0,-70},{-20,-50}})));
  BaseClasses.SteamCoil coi1(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=mCoi1_flow_nominal,
    show_T=true,
    TSat=TSat,
    pSat=pSat) "Steam coil"
    annotation (Placement(transformation(extent={{40,-70},{20,-50}})));
  Modelica.Blocks.Sources.TimeTable y2(table=[0,0.01; 1200,0.2; 1500,0.8; 2500,
        0.8; 3000,0.3; 3600,0.3], timeScale=24)
    annotation (Placement(transformation(extent={{-150,-10},{-130,10}})));
  Modelica.Blocks.Math.Gain m1_flow(k=QCoi2_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
  HeatExchangers.Heater_T supHea(
    redeclare package Medium = MediumSte,
    m_flow_nominal=mBoi_flow_nominal,
    show_T=true,
    dp_nominal=10000,
    T_start=TSat)     "Super heater"
    annotation (Placement(transformation(extent={{60,20},{80,40}})));
  Modelica.Blocks.Sources.Constant conTSet(k=TSte)
    "Constant temperature setpoint"
    annotation (Placement(transformation(extent={{20,60},{40,80}})));
  HeatTransfer.Sources.FixedTemperature           gnd(T=(55 + 273.15) + 273.15)
    "Ground temperature"
    annotation (Placement(transformation(extent={{70,60},{90,80}})));
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
    annotation (Placement(transformation(extent={{90,20},{110,40}})));
  FixedResistances.Junction spl(
    redeclare package Medium = MediumSte,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    p_start=pSat,
    T_start=TSte,
    m_flow_nominal=mBoi_flow_nominal*{1,-1/2,-1/2},
    dp_nominal=6000*{1,-1,-1}) "Splitter"
    annotation (Placement(transformation(extent={{120,20},{140,40}})));
  FixedResistances.Junction jun(
    redeclare package Medium = MediumWat,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=mBoi_flow_nominal*{1,1/2,-1/2},
    dp_nominal=6000*{1,1,-1}) "Junction"
    annotation (Placement(transformation(extent={{-50,20},{-30,40}})));
  Modelica.Blocks.Sources.CombiTimeTable QHea(
    tableOnFile=tableOnFile,
    tableName=tableName,
    fileName=ModelicaServices.ExternalReferences.loadResource(
        "modelica://Buildings/Resources/Data/Fluid/Boilers/Examples/SteamBoilerSimpleLoopBS21/HeatingLoadProfiles.csv"),
    columns=2:10,
    smoothness=smoothness,
    timeScale=timeScale,
    startTime(displayUnit="h") = 7200)
                         "Heating demand"
    annotation (Placement(transformation(extent={{-180,80},{-160,100}})));

  Modelica.Blocks.Math.Add add(k1=-1, k2=-1)
    annotation (Placement(transformation(extent={{-70,60},{-50,80}})));
  Modelica.Blocks.Math.Gain PLR(k=1/QBoi_flow_nominal) "Part load ratio"
    annotation (Placement(transformation(extent={{-30,60},{-10,80}})));
  Modelica.Blocks.Math.Gain m2_flow1(k=1/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-138,80},{-118,100}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=-1,
    duration(displayUnit="h") = 3600,
    offset=1,
    startTime(displayUnit="h") = 3600)
    annotation (Placement(transformation(extent={{-180,40},{-160,60}})));
  Modelica.Blocks.Math.Add add1
    annotation (Placement(transformation(extent={{-106,52},{-86,72}})));
equation
  connect(preDro.port_b, boi.port_a)
    annotation (Line(points={{0,30},{20,30}},  color={0,127,255}));
  connect(coi2.port_b, pum2.port_a)
    annotation (Line(points={{20,-20},{0,-20}},  color={0,127,255}));
  connect(coi1.port_b, pum1.port_a)
    annotation (Line(points={{20,-60},{0,-60}},  color={0,127,255}));
  connect(boi.port_b, supHea.port_a)
    annotation (Line(points={{40,30},{60,30}},         color={0,127,255}));
  connect(conTSet.y, supHea.TSet) annotation (Line(points={{41,70},{50,70},{50,38},
          {58,38}},     color={0,0,127}));
  connect(supHea.port_b, pipSte.port_a)
    annotation (Line(points={{80,30},{90,30}}, color={0,127,255}));
  connect(gnd.port, pipSte.heatPort)
    annotation (Line(points={{90,70},{100,70},{100,40}},
                                                       color={191,0,0}));
  connect(pipSte.ports_b[1],spl. port_1)
    annotation (Line(points={{110,30},{120,30}}, color={0,127,255}));
  connect(spl.port_3, coi2.port_a)
    annotation (Line(points={{130,20},{130,-20},{40,-20}}, color={0,127,255}));
  connect(spl.port_2, coi1.port_a) annotation (Line(points={{140,30},{144,30},{144,
          -60},{40,-60}}, color={0,127,255}));
  connect(jun.port_2, preDro.port_a)
    annotation (Line(points={{-30,30},{-20,30}}, color={0,127,255}));
  connect(jun.port_3, pum2.port_b) annotation (Line(points={{-40,20},{-40,-20},{
          -20,-20}}, color={0,127,255}));
  connect(pum1.port_b, jun.port_1) annotation (Line(points={{-20,-60},{-60,-60},
          {-60,30},{-50,30}}, color={0,127,255}));
  connect(coi2.QOut_flow, add.u2) annotation (Line(points={{19,-11},{14,-11},{14,
          12},{-76,12},{-76,64},{-72,64}},
                                       color={0,0,127}));
  connect(coi1.QOut_flow, add.u1) annotation (Line(points={{19,-51},{18,-51},{18,
          -52},{10,-52},{10,8},{-80,8},{-80,76},{-72,76}},
                                                    color={0,0,127}));
  connect(add.y, PLR.u)
    annotation (Line(points={{-49,70},{-32,70}}, color={0,0,127}));
  connect(PLR.y, boi.y) annotation (Line(points={{-9,70},{6,70},{6,38},{18,38}},
        color={0,0,127}));
  connect(y2.y, m2_flow.u)
    annotation (Line(points={{-129,0},{-112,0}}, color={0,0,127}));
  connect(y1.y, m1_flow.u)
    annotation (Line(points={{-129,-40},{-112,-40}}, color={0,0,127}));
  connect(m2_flow1.u, QHea.y[2])
    annotation (Line(points={{-140,90},{-159,90}}, color={0,0,127}));
  connect(pum2.m_flow_in, m2_flow.y)
    annotation (Line(points={{-10,-8},{-10,0},{-89,0}}, color={0,0,127}));
  connect(pum1.m_flow_in, m1_flow.y)
    annotation (Line(points={{-10,-48},{-10,-40},{-89,-40}}, color={0,0,127}));
  connect(m2_flow1.y, add1.u1) annotation (Line(points={{-117,90},{-117,80},{
          -108,80},{-108,68}}, color={0,0,127}));
  connect(ramp.y, add1.u2) annotation (Line(points={{-159,50},{-134,50},{-134,
          56},{-108,56}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{160,100}})),
    experiment(
      StopTime=86400,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/SteamBoilerSimpleLoopBS21.mos"
        "Simulate and plot"));
end SteamBoilerSimpleLoopBS21;
