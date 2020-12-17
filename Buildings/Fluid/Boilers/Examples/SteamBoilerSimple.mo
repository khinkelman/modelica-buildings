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
   pressure(V_flow={0,mCoi_flow_nominal,2*mCoi_flow_nominal}/1000,
                   dp={2*dp_nominal,dp_nominal,0}))
    "Performance data for primary pumps";
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
    fue=Data.Fuels.NaturalGasLowerHeatingValue(),
    dpVal_nominal=dp_nominal/2,
    k=3,
    Ti=3)                                         "Boiler"
    annotation (Placement(transformation(extent={{40,20},{60,40}})));
  BaseClasses.SteamCoil coi(redeclare package Medium_a = MediumSte, redeclare
      package Medium_b = MediumWat,
    m_flow_nominal=mCoi_flow_nominal,
    show_T=true,
    TSat=TSat,
    pSat=pSat,
    dp_nominal=dp_nominal/2)        "Steam coil"
    annotation (Placement(transformation(extent={{60,-80},{40,-60}})));
  Movers.SpeedControlled_y pum(
    redeclare package Medium = MediumWat, per=per)
    "Condensate pump"
    annotation (Placement(transformation(extent={{0,-80},{-20,-60}})));

  Modelica.Blocks.Math.Gain m_flow(k=QBoi_flow_nominal/hfg)
    "Gain to calculate m_flow"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Sources.Boundary_pT sou(redeclare package Medium = MediumWat, nPorts=1)
            "Source"
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  Sources.Boundary_pT sin(redeclare package Medium = MediumWat, nPorts=1)
    "Sink" annotation (Placement(transformation(extent={{20,-40},{0,-20}})));
  Modelica.Blocks.Sources.TimeTable y(table=[0,0; 1800,1; 1800,0; 2400,0; 2400,1;
        3600,1])
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  Controls.Continuous.LimPID con(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=3,
    Ti=3)                        "Controller"
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Sensors.MassFlowRate senMasFlo(redeclare package Medium = MediumWat)
    annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
equation
  connect(boi.port_b, coi.port_a) annotation (Line(points={{60,30},{80,30},{80,-70},
          {60,-70}},      color={0,127,255}));
  connect(coi.port_b, pum.port_a)
    annotation (Line(points={{40,-70},{0,-70}},   color={0,127,255}));
  connect(sou.ports[1], boi.port_a)
    annotation (Line(points={{20,30},{40,30}}, color={0,127,255}));
  connect(y.y, boi.y) annotation (Line(points={{-59,70},{30,70},{30,38},{38,38}},
        color={0,0,127}));
  connect(y.y, m_flow.u) annotation (Line(points={{-59,70},{-48,70},{-48,20},{-90,
          20},{-90,0},{-82,0}},
                color={0,0,127}));
  connect(m_flow.y, con.u_s)
    annotation (Line(points={{-59,0},{-42,0}}, color={0,0,127}));
  connect(senMasFlo.m_flow, con.u_m)
    annotation (Line(points={{-30,-19},{-30,-12}}, color={0,0,127}));
  connect(con.y, pum.y)
    annotation (Line(points={{-19,0},{-10,0},{-10,-58}}, color={0,0,127}));
  connect(pum.port_b, senMasFlo.port_a) annotation (Line(points={{-20,-70},{-50,
          -70},{-50,-30},{-40,-30}}, color={0,127,255}));
  connect(senMasFlo.port_b, sin.ports[1])
    annotation (Line(points={{-20,-30},{0,-30}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=3600, __Dymola_Algorithm="Dassl"),
__Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Boilers/Examples/SteamBoilerSimple.mos"
        "Simulate and plot"));
end SteamBoilerSimple;
