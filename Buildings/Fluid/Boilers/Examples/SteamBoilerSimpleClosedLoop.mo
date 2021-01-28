within Buildings.Fluid.Boilers.Examples;
model SteamBoilerSimpleClosedLoop
  "Closed loop example of steam boiler system"
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
    m_flow_nominal=m_flow_nominal,
    show_T=true,
    TSat=TSat,
    pSat=pSat,
    Q_flow_nominal=Q_flow_nominal,
    effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
    a={0.8},
    fue=Data.Fuels.NaturalGasLowerHeatingValue(),
    dpSte_nominal=pSat - 101325,
    dpVal_nominal=dp_nominal/2,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    k=3,
    Ti=3)                                         "Boiler"
    annotation (Placement(transformation(extent={{100,40},{120,60}})));

  Modelica.Blocks.Sources.TimeTable y(table=[0,0.01; 1800,1; 2000,1; 2400,0.2;
        2600,0.2; 3600,0.9])
    annotation (Placement(transformation(extent={{-140,0},{-120,20}})));
  Movers.SpeedControlled_y pum(
    redeclare package Medium = MediumWat,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    per=per)
    "Condensate pump"
    annotation (Placement(transformation(extent={{80,-50},{60,-30}})));
  BaseClasses.SteamCoil coi(
    redeclare package Medium_a = MediumSte,
    redeclare package Medium_b = MediumWat,
    m_flow_nominal=m_flow_nominal,
    show_T=true,
    TSatHig=TSat,
    pSat=pSat) "Steam coil"
    annotation (Placement(transformation(extent={{120,-50},{100,-30}})));

  Controls.Continuous.LimPID con(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=3,
    Ti=3)                        "Controller"
    annotation (Placement(transformation(extent={{20,0},{40,20}})));
  Sensors.MassFlowRate senMasFlo(redeclare package Medium = MediumWat)
    annotation (Placement(transformation(extent={{40,-50},{20,-30}})));
  Modelica.Blocks.Math.Gain mCal_flow(k=Q_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-80,0},{-60,20}})));
  Sources.Boundary_pT pRef(redeclare package Medium = MediumWat, nPorts=2)
    "Reference pressure"
    annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
equation
  connect(boi.port_b, coi.port_a) annotation (Line(points={{120,50},{130,50},{130,
          -40},{120,-40}}, color={0,127,255}));
  connect(coi.port_b, pum.port_a)
    annotation (Line(points={{100,-40},{80,-40}}, color={0,127,255}));
  connect(pum.port_b, senMasFlo.port_a)
    annotation (Line(points={{60,-40},{40,-40}}, color={0,127,255}));
  connect(y.y, mCal_flow.u)
    annotation (Line(points={{-119,10},{-82,10}}, color={0,0,127}));
  connect(mCal_flow.y, con.u_s)
    annotation (Line(points={{-59,10},{18,10}}, color={0,0,127}));
  connect(con.u_m, senMasFlo.m_flow)
    annotation (Line(points={{30,-2},{30,-29}}, color={0,0,127}));
  connect(con.y, pum.y)
    annotation (Line(points={{41,10},{70,10},{70,-28}}, color={0,0,127}));
  connect(y.y, boi.y) annotation (Line(points={{-119,10},{-100,10},{-100,58},{98,
          58}}, color={0,0,127}));
  connect(pRef.ports[1], boi.port_a)
    annotation (Line(points={{-20,32},{-20,50},{100,50}}, color={0,127,255}));
  connect(senMasFlo.port_b, pRef.ports[2])
    annotation (Line(points={{20,-40},{-20,-40},{-20,28}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{160,100}})),
    experiment(StopTime=3600, __Dymola_Algorithm="Dassl"),
__Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/SteamBoilerSimpleClosedLoop.mos"
        "Simulate and plot"));
end SteamBoilerSimpleClosedLoop;
