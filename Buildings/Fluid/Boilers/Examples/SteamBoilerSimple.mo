within Buildings.Fluid.Boilers.Examples;
model SteamBoilerSimple
  "Open loop example model for simple steam boiler system"
  extends Modelica.Icons.Example;

  package MediumSte = Buildings.Media.Steam (
     T_default=110+273.15,
     p_default=143380) "Steam medium";
  package MediumWat = Buildings.Media.Water (
    T_default=20+273.15,
    p_default=101325) "Water medium";

  parameter Modelica.SIunits.AbsolutePressure pSat=143380
    "Nominal steam pressure";
  parameter Modelica.SIunits.AbsolutePressure pAtm=101325
    "Atmospheric pressure";
  parameter Modelica.SIunits.PressureDifference dpSte_nominal = pSat - pAtm
    "Nominal change in pressure in steam loop";
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

  parameter Buildings.Fluid.Movers.Data.Generic per(
   pressure(V_flow=mCoi_flow_nominal*{0,1,2}/1000,
                   dp=dpSte_nominal*{2,1,0}))
    "Performance data for primary pumps";
  parameter Modelica.SIunits.PressureDifference dpPip_nominal=10000
    "Pressure drop in pipe at nominal mass flow rate";

  parameter MediumSte.ThermodynamicState staSte_default=MediumSte.setState_pTX(
      T=MediumSte.T_default, p=MediumSte.p_default, X=MediumSte.X_default);
  parameter Modelica.SIunits.Density rhoSte_default=MediumSte.density(staSte_default)
    "Density, used to compute rhoStd in val";

  Buildings.Fluid.Boilers.SteamBoilerSimple boi(
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
    dpSte_nominal=dpSte_nominal,
    dpVal_nominal(displayUnit="Pa") = 1,
    k=3,
    Ti=3)                                         "Boiler"
    annotation (Placement(transformation(extent={{60,20},{80,40}})));
  BaseClasses.SteamCoil coi(redeclare package Medium_a = MediumSte, redeclare
      package Medium_b = MediumWat,
    m_flow_nominal=mCoi_flow_nominal,
    show_T=true,
    TSatHig=TSat,
    pSat=pSat,
    dpCoi_nominal(displayUnit="Pa") = 1,
    dpSte_nominal=pSat - pAtm + dpPip_nominal - 3)
                                       "Steam coil"
    annotation (Placement(transformation(extent={{40,-80},{20,-60}})));

  Modelica.Blocks.Math.Gain m_flow(k=QBoi_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
  Sources.Boundary_pT sou(redeclare package Medium = MediumWat,
    p=pAtm,
    T=363.15,
    nPorts=2)
            "Source"
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
  Sources.Boundary_pT sin(redeclare package Medium = MediumWat,
    p=pAtm - 5,
    T=363.15,
    nPorts=1)
    "Sink" annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  Modelica.Blocks.Sources.TimeTable y(table=[0,0; 1800,1; 1800,0; 2400,0; 2400,1;
        3600,1])
    annotation (Placement(transformation(extent={{-90,70},{-70,90}})));
  Controls.Continuous.LimPID conVal(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.5,
    Ti=30) "Valve controller"
    annotation (Placement(transformation(extent={{-20,-50},{0,-30}})));
  Sensors.MassFlowRate senMasFlo(redeclare package Medium = MediumWat)
    annotation (Placement(transformation(extent={{0,-80},{-20,-60}})));
  Actuators.Valves.TwoWayLinear val(
    redeclare package Medium = MediumSte,
    m_flow_nominal=mCoi_flow_nominal,
    dpValve_nominal=1,
    rhoStd=rhoSte_default,
    riseTime=30,
    init=Modelica.Blocks.Types.Init.InitialState,
    dpFixed_nominal=2)
    annotation (Placement(transformation(extent={{80,-80},{60,-60}})));
  Sensors.RelativePressure senRelPre
    annotation (Placement(transformation(extent={{40,10},{20,-10}})));
  Movers.SpeedControlled_y pum(redeclare package Medium = MediumWat, per=per,
    y_start=1)                                                       "Pump"
    annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
  Controls.Continuous.LimPID conVal1(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=3,
    Ti=3)  "Valve controller"
    annotation (Placement(transformation(extent={{20,40},{40,60}})));
  Modelica.Blocks.Sources.RealExpression dpSet(y=dpSte_nominal)
    "Relative pressure sepoint"
    annotation (Placement(transformation(extent={{-10,40},{10,60}})));
  FixedResistances.PressureDrop res(
    redeclare package Medium = MediumWat,
    m_flow_nominal=mCoi_flow_nominal,
    dp_nominal=dpPip_nominal)
    annotation (Placement(transformation(extent={{-30,-80},{-50,-60}})));
equation
  connect(y.y, boi.y) annotation (Line(points={{-69,80},{52,80},{52,38},{58,38}},
        color={0,0,127}));
  connect(y.y, m_flow.u) annotation (Line(points={{-69,80},{-60,80},{-60,60},{-90,
          60},{-90,-40},{-82,-40}},
                color={0,0,127}));
  connect(m_flow.y, conVal.u_s) annotation (Line(points={{-59,-40},{-22,-40}},
                      color={0,0,127}));
  connect(senMasFlo.m_flow, conVal.u_m)
    annotation (Line(points={{-10,-59},{-10,-52}}, color={0,0,127}));
  connect(dpSet.y, conVal1.u_s)
    annotation (Line(points={{11,50},{18,50}}, color={0,0,127}));
  connect(sou.ports[1], pum.port_a)
    annotation (Line(points={{-60,32},{-60,30},{-40,30}}, color={0,127,255}));
  connect(pum.port_b, boi.port_a)
    annotation (Line(points={{-20,30},{60,30}}, color={0,127,255}));
  connect(boi.port_b, val.port_a) annotation (Line(points={{80,30},{90,30},{90,-70},
          {80,-70}}, color={0,127,255}));
  connect(val.port_b, coi.port_a)
    annotation (Line(points={{60,-70},{40,-70}}, color={0,127,255}));
  connect(coi.port_b, senMasFlo.port_a)
    annotation (Line(points={{20,-70},{0,-70}}, color={0,127,255}));
  connect(senRelPre.port_a, val.port_a) annotation (Line(points={{40,0},{90,0},{
          90,-70},{80,-70}}, color={0,127,255}));
  connect(senRelPre.port_b, sou.ports[2]) annotation (Line(points={{20,0},{-50,0},
          {-50,28},{-60,28}}, color={0,127,255}));
  connect(senRelPre.p_rel, conVal1.u_m)
    annotation (Line(points={{30,9},{30,38}}, color={0,0,127}));
  connect(conVal1.y, pum.y) annotation (Line(points={{41,50},{46,50},{46,70},{-30,
          70},{-30,42}}, color={0,0,127}));
  connect(conVal.y, val.y)
    annotation (Line(points={{1,-40},{70,-40},{70,-58}}, color={0,0,127}));
  connect(senMasFlo.port_b, res.port_a)
    annotation (Line(points={{-20,-70},{-30,-70}}, color={0,127,255}));
  connect(res.port_b, sin.ports[1])
    annotation (Line(points={{-50,-70},{-60,-70}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=3600, __Dymola_Algorithm="Dassl"),
__Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/SteamBoilerSimple.mos"
        "Simulate and plot"));
end SteamBoilerSimple;
