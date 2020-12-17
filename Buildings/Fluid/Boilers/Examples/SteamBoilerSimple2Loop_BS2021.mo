within Buildings.Fluid.Boilers.Examples;
model SteamBoilerSimple2Loop_BS2021
  "Closed loop example of steam boiler system with two parallel paths"
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

  parameter Modelica.SIunits.PressureDifference dp_nominal=6000
    "Pressure drop at nominal mass flow rate in district network";

  parameter Buildings.Fluid.Movers.Data.Generic per(
   pressure(V_flow={0,m_flow_nominal,2*m_flow_nominal}/1000,
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
    dpVal_nominal=dp_nominal/5,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    k=3,
    Ti=3)                                         "Boiler"
    annotation (Placement(transformation(extent={{40,50},{60,70}})));

  Modelica.Blocks.Sources.TimeTable y(table=[0,0.01; 1800,1; 2000,1; 2400,0.2;
        2600,0.2; 3600,0.9])
    annotation (Placement(transformation(extent={{-150,30},{-130,50}})));
  Movers.SpeedControlled_y pum(
    redeclare package Medium = MediumWat,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    per=per)
    "Condensate pump"
    annotation (Placement(transformation(extent={{10,-40},{-10,-20}})));
  BaseClasses.SteamCoil coi(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=m_flow_nominal,
    show_T=true,
    TSat=TSat,
    pSat=pSat,
    dp_nominal=dp_nominal/5)
               "Steam coil"
    annotation (Placement(transformation(extent={{82,-40},{62,-20}})));
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
    annotation (Placement(transformation(extent={{-150,70},{-130,90}})));

  Controls.Continuous.LimPID con(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=3,
    Ti=3)                        "Controller"
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Sensors.MassFlowRate senMasFlo(redeclare package Medium = MediumWat)
    annotation (Placement(transformation(extent={{-20,-40},{-40,-20}})));
  Modelica.Blocks.Math.Gain mCal_flow(k=Q_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Sources.Boundary_pT pRef(redeclare package Medium = MediumWat, nPorts=2)
    "Reference pressure"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-30,70})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{62,-10},{82,10}})));
  Modelica.Blocks.Math.Gain plr(k=1/Q_flow_nominal) "Part load ratio"
    annotation (Placement(transformation(extent={{0,80},{20,100}})));
  Modelica.Blocks.Math.Add add1
    annotation (Placement(transformation(extent={{62,-80},{82,-60}})));
  Movers.SpeedControlled_y pum1(
    redeclare package Medium = MediumWat,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    per=per)
    "Condensate pump"
    annotation (Placement(transformation(extent={{10,-110},{-10,-90}})));
  BaseClasses.SteamCoil coi1(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=m_flow_nominal,
    show_T=true,
    TSat=TSat,
    pSat=pSat,
    dp_nominal=dp_nominal/5)
               "Steam coil"
    annotation (Placement(transformation(extent={{82,-110},{62,-90}})));
  Controls.Continuous.LimPID con1(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=3,
    Ti=3)                        "Controller"
    annotation (Placement(transformation(extent={{-40,-80},{-20,-60}})));
  Sensors.MassFlowRate senMasFlo1(redeclare package Medium = MediumWat)
    annotation (Placement(transformation(extent={{-20,-110},{-40,-90}})));
  FixedResistances.Junction spl(
    redeclare package Medium = MediumSte,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    p_start=pSat,
    T_start=TSat,
    m_flow_nominal=m_flow_nominal*{2,-1,-1},
    dp_nominal=dp_nominal/5*{1,-1,-1})
                               "Splitter"
    annotation (Placement(transformation(extent={{80,50},{100,70}})));
  FixedResistances.Junction jun(
    redeclare package Medium = MediumWat,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    m_flow_nominal=m_flow_nominal*{2,1,-1},
    dp_nominal=dp_nominal/5*{1,1,-1})
                              "Junction"
    annotation (Placement(transformation(extent={{-70,50},{-50,70}})));
  Modelica.Blocks.Math.Add add2
    annotation (Placement(transformation(extent={{120,-16},{140,4}})));
  Modelica.Blocks.Sources.TimeTable y2(table=[0,0.01; 1200,0.2; 1500,0.8; 2500,
        0.8; 3000,0.3; 3600,0.3])
    annotation (Placement(transformation(extent={{-150,-10},{-130,10}})));
  FixedResistances.CheckValve cheVal(
    redeclare package Medium = MediumWat,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=dp_nominal/5,
    dpFixed_nominal=dp_nominal*4/5)
    annotation (Placement(transformation(extent={{46,-40},{26,-20}})));
  FixedResistances.CheckValve cheVal1(
    redeclare package Medium = MediumWat,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=dp_nominal/5,
    dpFixed_nominal=dp_nominal*4/5)
    annotation (Placement(transformation(extent={{46,-110},{26,-90}})));
equation
  connect(pum.port_b, senMasFlo.port_a)
    annotation (Line(points={{-10,-30},{-20,-30}},
                                                 color={0,127,255}));
  connect(mCal_flow.y, con.u_s)
    annotation (Line(points={{-89,0},{-42,0}},  color={0,0,127}));
  connect(con.u_m, senMasFlo.m_flow)
    annotation (Line(points={{-30,-12},{-30,-19}},
                                                color={0,0,127}));
  connect(con.y, pum.y)
    annotation (Line(points={{-19,0},{0,0},{0,-18}},    color={0,0,127}));
  connect(coi.QOut_flow, add.u2) annotation (Line(points={{61,-21},{56,-21},{56,
          -6},{60,-6}}, color={0,0,127}));
  connect(coi.QLos_flow, add.u1) annotation (Line(points={{61,-24},{54,-24},{54,
          6},{60,6}}, color={0,0,127}));
  connect(plr.y, boi.y) annotation (Line(points={{21,90},{30,90},{30,68},{38,68}},
        color={0,0,127}));
  connect(coi1.QOut_flow, add1.u2) annotation (Line(points={{61,-91},{56,-91},{
          56,-76},{60,-76}}, color={0,0,127}));
  connect(coi1.QLos_flow, add1.u1) annotation (Line(points={{61,-94},{54,-94},{
          54,-64},{60,-64}}, color={0,0,127}));
  connect(con1.y, pum1.y)
    annotation (Line(points={{-19,-70},{0,-70},{0,-88}}, color={0,0,127}));
  connect(con1.u_m, senMasFlo1.m_flow)
    annotation (Line(points={{-30,-82},{-30,-89}}, color={0,0,127}));
  connect(boi.port_b, spl.port_1)
    annotation (Line(points={{60,60},{80,60}}, color={0,127,255}));
  connect(spl.port_3, coi.port_a)
    annotation (Line(points={{90,50},{90,-30},{82,-30}}, color={0,127,255}));
  connect(spl.port_2, coi1.port_a) annotation (Line(points={{100,60},{110,60},{
          110,-100},{82,-100}}, color={0,127,255}));
  connect(pum1.port_b, senMasFlo1.port_a)
    annotation (Line(points={{-10,-100},{-20,-100}}, color={0,127,255}));
  connect(senMasFlo.port_b, jun.port_3) annotation (Line(points={{-40,-30},{-60,
          -30},{-60,50}}, color={0,127,255}));
  connect(senMasFlo1.port_b, jun.port_1) annotation (Line(points={{-40,-100},{
          -80,-100},{-80,60},{-70,60}}, color={0,127,255}));
  connect(jun.port_2, pRef.ports[1])
    annotation (Line(points={{-50,60},{-28,60}}, color={0,127,255}));
  connect(pRef.ports[2], boi.port_a)
    annotation (Line(points={{-32,60},{40,60}}, color={0,127,255}));
  connect(add.y, add2.u1)
    annotation (Line(points={{83,0},{118,0}}, color={0,0,127}));
  connect(add2.u2, add1.y) annotation (Line(points={{118,-12},{100,-12},{100,
          -70},{83,-70}}, color={0,0,127}));
  connect(add2.y, plr.u) annotation (Line(points={{141,-6},{150,-6},{150,110},{
          -10,110},{-10,90},{-2,90}}, color={0,0,127}));
  connect(mCal_flow.y, con1.u_s) annotation (Line(points={{-89,0},{-70,0},{-70,
          -70},{-42,-70}}, color={0,0,127}));
  connect(mCal_flow.u, y2.y)
    annotation (Line(points={{-112,0},{-129,0}}, color={0,0,127}));
  connect(coi.port_b, cheVal.port_a)
    annotation (Line(points={{62,-30},{46,-30}}, color={0,127,255}));
  connect(cheVal.port_b, pum.port_a)
    annotation (Line(points={{26,-30},{10,-30}}, color={0,127,255}));
  connect(coi1.port_b, cheVal1.port_a)
    annotation (Line(points={{62,-100},{46,-100}}, color={0,127,255}));
  connect(cheVal1.port_b, pum1.port_a)
    annotation (Line(points={{26,-100},{10,-100}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},{160,
            120}})),
    experiment(StopTime=31536000, __Dymola_Algorithm="Dassl"),
__Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/SteamBoilerSimple2Loop_BS2021.mos"
        "Simulate and plot"));
end SteamBoilerSimple2Loop_BS2021;
