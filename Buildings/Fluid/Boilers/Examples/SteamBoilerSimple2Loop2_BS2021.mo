within Buildings.Fluid.Boilers.Examples;
model SteamBoilerSimple2Loop2_BS2021
  "Closed loop example of steam boiler system with two parallel paths"
  extends Modelica.Icons.Example;

  package MediumSte = Buildings.Media.Steam (
     T_default=110+273.15,
     p_default=143380) "Steam medium";
  package MediumWat = Buildings.Media.Water (
     T_default=90+273.15,
     p_default=101325) "Water medium";

  parameter Modelica.SIunits.AbsolutePressure pAtm=101325
    "Atmospheric pressure";
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

  parameter Modelica.SIunits.MassFlowRate m_flow_nominal=Q_flow_nominal/hfg
    "Nominal mass flow rate";
  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal=19347.2793
    "Nominal heat flow rate";
  parameter MediumSte.ThermodynamicState staSte_default=MediumSte.setState_pTX(
      T=MediumSte.T_default, p=MediumSte.p_default, X=MediumSte.X_default);
  parameter Modelica.SIunits.Density rhoSte_default=MediumSte.density(staSte_default)
    "Density, used to compute rhoStd in val";

  parameter Modelica.SIunits.PressureDifference dp_nominal=60000
    "Pressure drop at nominal mass flow rate in district network";

  parameter Buildings.Fluid.Movers.Data.Generic per(
   pressure(V_flow=m_flow_nominal*6*{0,1,2}/1000,
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
    m_flow_nominal=m_flow_nominal*2,
    show_T=true,
    TSat=TSat,
    pSat=pSat,
    Q_flow_nominal=Q_flow_nominal*2,
    effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
    a={0.8},
    fue=Data.Fuels.NaturalGasLowerHeatingValue(),
    dpSte_nominal=pSat - 101325,
    dpVal_nominal=dp_nominal/6,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    k=3,
    Ti=3)                                         "Boiler"
    annotation (Placement(transformation(extent={{60,70},{80,90}})));

  Modelica.Blocks.Sources.TimeTable y(table=[0,0.01; 1800,1; 2000,1; 2400,0.2;
        2600,0.2; 3600,0.9])
    annotation (Placement(transformation(extent={{-170,30},{-150,50}})));
  Movers.SpeedControlled_y pum(
    redeclare package Medium = MediumWat,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    per=per,
    y_start=0.2)
    "Condensate pump"
    annotation (Placement(transformation(extent={{-20,70},{0,90}})));
  BaseClasses.SteamCoil coi(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=m_flow_nominal,
    show_T=true,
    TSat=TSat,
    pSat=pSat,
    dp_nominal=dp_nominal/6)
               "Steam coil"
    annotation (Placement(transformation(extent={{60,-40},{40,-20}})));
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
    annotation (Placement(transformation(extent={{-140,110},{-120,130}})));

  Controls.Continuous.LimPID conPum(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=3,
    Ti=3) "Pump controller"
    annotation (Placement(transformation(extent={{-50,100},{-30,120}})));
  Sensors.MassFlowRate senMasFlo(redeclare package Medium = MediumWat)
    annotation (Placement(transformation(extent={{0,-40},{-20,-20}})));
  Modelica.Blocks.Math.Gain mCal_flow1(k=Q_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-118,-10},{-98,10}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Modelica.Blocks.Math.Gain plr(k=1/Q_flow_nominal) "Part load ratio"
    annotation (Placement(transformation(extent={{20,100},{40,120}})));
  Modelica.Blocks.Math.Add add1
    annotation (Placement(transformation(extent={{40,-100},{60,-80}})));
  BaseClasses.SteamCoil coi1(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=m_flow_nominal,
    show_T=true,
    TSat=TSat,
    pSat=pSat,
    dp_nominal=dp_nominal/6)
               "Steam coil"
    annotation (Placement(transformation(extent={{60,-130},{40,-110}})));
  Sensors.MassFlowRate senMasFlo1(redeclare package Medium = MediumWat)
    annotation (Placement(transformation(extent={{0,-130},{-20,-110}})));
  FixedResistances.Junction spl(
    redeclare package Medium = MediumSte,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    p_start=pSat,
    T_start=TSat,
    m_flow_nominal=m_flow_nominal*{2,-1,-1},
    dp_nominal=dp_nominal/6*{1,0,0})
                               "Splitter"
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
  FixedResistances.Junction jun(
    redeclare package Medium = MediumWat,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    m_flow_nominal=m_flow_nominal*{2,1,-1},
    dp_nominal=dp_nominal/6*{1,1,0})
                              "Junction"
    annotation (Placement(transformation(extent={{-40,-110},{-60,-130}})));
  Modelica.Blocks.Math.Add add2(k1=-1, k2=-1)
    annotation (Placement(transformation(extent={{140,-16},{160,4}})));
  Modelica.Blocks.Sources.TimeTable y1(table=[0,0.01; 1200,0.2; 1500,0.8; 2500,
        0.8; 3000,0.3; 3600,0.3])
    annotation (Placement(transformation(extent={{-170,-10},{-150,10}})));
  FixedResistances.CheckValve cheVal(
    redeclare package Medium = MediumWat,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=dp_nominal/6,
    dpFixed_nominal=dp_nominal*5/6)
    annotation (Placement(transformation(extent={{30,-40},{10,-20}})));
  FixedResistances.CheckValve cheVal1(
    redeclare package Medium = MediumWat,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=dp_nominal/6,
    dpFixed_nominal=dp_nominal*5/6)
    annotation (Placement(transformation(extent={{30,-130},{10,-110}})));
  Actuators.Valves.TwoWayLinear val1(
    redeclare package Medium = MediumSte,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=dp_nominal/6,
    rhoStd=rhoSte_default,
    riseTime=60,
    dpFixed_nominal=dp_nominal*5/6)
    annotation (Placement(transformation(extent={{100,-130},{80,-110}})));
  Actuators.Valves.TwoWayLinear val(
    redeclare package Medium = MediumSte,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=dp_nominal/6,
    rhoStd=rhoSte_default,
    riseTime=60,
    dpFixed_nominal=dp_nominal*5/6)
    annotation (Placement(transformation(extent={{100,-40},{80,-20}})));
  Controls.Continuous.LimPID conCoi(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=3,
    Ti=3) "Coil flow controller"
    annotation (Placement(transformation(extent={{-20,10},{0,30}})));
  Controls.Continuous.LimPID conCoi1(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.5,
    Ti=15)
          "Coil flow controller"
    annotation (Placement(transformation(extent={{-20,-80},{0,-60}})));
  Sensors.MassFlowRate senMasFloMai(redeclare package Medium = MediumWat)
    "Main mass flow sensor"
    annotation (Placement(transformation(extent={{-50,70},{-30,90}})));
  Modelica.Blocks.Math.Gain mCal_flow(k=Q_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-120,30},{-100,50}})));
  Modelica.Blocks.Math.Add add3
    annotation (Placement(transformation(extent={{-80,100},{-60,120}})));
  Modelica.Blocks.Math.Gain QCoi_flow(k=-Q_flow_nominal) "Coil heat flow rate"
    annotation (Placement(transformation(extent={{-130,70},{-110,90}})));
  Modelica.Blocks.Math.Gain QCoi1_flow(k=-Q_flow_nominal) "Coil heat flow rate"
    annotation (Placement(transformation(extent={{-128,-50},{-108,-30}})));
equation
  connect(coi.QOut_flow, add.u2) annotation (Line(points={{39,-21},{34,-21},{34,
          -6},{38,-6}}, color={0,0,127}));
  connect(coi.QLos_flow, add.u1) annotation (Line(points={{39,-24},{32,-24},{32,
          6},{38,6}}, color={0,0,127}));
  connect(plr.y, boi.y) annotation (Line(points={{41,110},{50,110},{50,88},{58,88}},
        color={0,0,127}));
  connect(coi1.QOut_flow, add1.u2) annotation (Line(points={{39,-111},{34,-111},
          {34,-96},{38,-96}},color={0,0,127}));
  connect(coi1.QLos_flow, add1.u1) annotation (Line(points={{39,-114},{32,-114},
          {32,-84},{38,-84}},color={0,0,127}));
  connect(boi.port_b, spl.port_1)
    annotation (Line(points={{80,80},{100,80}},color={0,127,255}));
  connect(add.y, add2.u1)
    annotation (Line(points={{61,0},{138,0}}, color={0,0,127}));
  connect(add2.u2, add1.y) annotation (Line(points={{138,-12},{120,-12},{120,-90},
          {61,-90}},      color={0,0,127}));
  connect(add2.y, plr.u) annotation (Line(points={{161,-6},{172,-6},{172,130},{12,
          130},{12,110},{18,110}},    color={0,0,127}));
  connect(mCal_flow1.u, y1.y)
    annotation (Line(points={{-120,0},{-149,0}}, color={0,0,127}));
  connect(coi.port_b, cheVal.port_a)
    annotation (Line(points={{40,-30},{30,-30}}, color={0,127,255}));
  connect(coi1.port_b, cheVal1.port_a)
    annotation (Line(points={{40,-120},{30,-120}}, color={0,127,255}));
  connect(spl.port_3, val.port_a)
    annotation (Line(points={{110,70},{110,-30},{100,-30}},
                                                         color={0,127,255}));
  connect(spl.port_2, val1.port_a) annotation (Line(points={{120,80},{130,80},{130,
          -120},{100,-120}},color={0,127,255}));
  connect(val1.port_b, coi1.port_a)
    annotation (Line(points={{80,-120},{60,-120}}, color={0,127,255}));
  connect(val.port_b, coi.port_a)
    annotation (Line(points={{80,-30},{60,-30}}, color={0,127,255}));
  connect(cheVal.port_b, senMasFlo.port_a)
    annotation (Line(points={{10,-30},{0,-30}},    color={0,127,255}));
  connect(cheVal1.port_b, senMasFlo1.port_a)
    annotation (Line(points={{10,-120},{0,-120}},    color={0,127,255}));
  connect(senMasFlo1.port_b, jun.port_1)
    annotation (Line(points={{-20,-120},{-40,-120}}, color={0,127,255}));
  connect(jun.port_3, senMasFlo.port_b) annotation (Line(points={{-50,-110},{-50,
          -30},{-20,-30}}, color={0,127,255}));
  connect(pum.port_b, boi.port_a)
    annotation (Line(points={{0,80},{60,80}},   color={0,127,255}));
  connect(senMasFlo1.m_flow,conCoi1. u_m)
    annotation (Line(points={{-10,-109},{-10,-82}}, color={0,0,127}));
  connect(senMasFlo.m_flow, conCoi.u_m)
    annotation (Line(points={{-10,-19},{-10,8}}, color={0,0,127}));
  connect(conCoi.y, val.y)
    annotation (Line(points={{1,20},{90,20},{90,-18}},   color={0,0,127}));
  connect(conCoi1.y, val1.y)
    annotation (Line(points={{1,-70},{90,-70},{90,-108}},   color={0,0,127}));
  connect(jun.port_2, senMasFloMai.port_a) annotation (Line(points={{-60,-120},{
          -68,-120},{-68,80},{-50,80}}, color={0,127,255}));
  connect(senMasFloMai.port_b, pum.port_a)
    annotation (Line(points={{-30,80},{-20,80}}, color={0,127,255}));
  connect(senMasFloMai.m_flow, conPum.u_m) annotation (Line(points={{-40,91},{-40,
          98}},                       color={0,0,127}));
  connect(y.y, mCal_flow.u)
    annotation (Line(points={{-149,40},{-122,40}}, color={0,0,127}));
  connect(add3.y, conPum.u_s)
    annotation (Line(points={{-59,110},{-52,110}}, color={0,0,127}));
  connect(conPum.y, pum.y)
    annotation (Line(points={{-29,110},{-10,110},{-10,92}}, color={0,0,127}));
  connect(mCal_flow.y, add3.u1) annotation (Line(points={{-99,40},{-94,40},{-94,
          116},{-82,116}},  color={0,0,127}));
  connect(add3.u2, mCal_flow1.y) annotation (Line(points={{-82,104},{-90,104},{-90,
          0},{-97,0}},        color={0,0,127}));
  connect(mCal_flow.y, conCoi.u_s) annotation (Line(points={{-99,40},{-62,40},{-62,
          20},{-22,20}},     color={0,0,127}));
  connect(mCal_flow1.y, conCoi1.u_s) annotation (Line(points={{-97,0},{-60,0},{-60,
          -70},{-22,-70}},     color={0,0,127}));
  connect(y.y, QCoi_flow.u) annotation (Line(points={{-149,40},{-140,40},{-140,80},
          {-132,80}}, color={0,0,127}));
  connect(QCoi1_flow.u, y1.y) annotation (Line(points={{-130,-40},{-140,-40},{-140,
          0},{-149,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-180,-140},{180,140}})),
    experiment(StopTime=3600, __Dymola_Algorithm="Dassl"),
__Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/SteamBoilerSimple2Loop_BS2021.mos"
        "Simulate and plot"));
end SteamBoilerSimple2Loop2_BS2021;
