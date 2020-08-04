within Buildings.Applications.DHC.EnergyTransferStations.Examples;
model CoolingDirectUncontrolled
  "Example model for the direct cooling energy transfer station with 
  uncontrolled district-building fluid transfer within the ETS"
  extends Modelica.Icons.Example;

  package Medium = Buildings.Media.Water "Water medium";

  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal = 150000
    "Nominal cooling load";

  parameter Modelica.SIunits.MassFlowRate m_flow_nominal=Q_flow_nominal/(cp*(16-7))
    "Nominal mass flow rate";

 parameter Modelica.SIunits.SpecificHeatCapacity cp=
   Medium.specificHeatCapacityCp(
      Medium.setState_pTX(Medium.p_default, Medium.T_default, Medium.X_default))
    "Default specific heat capacity of medium";

  Buildings.Applications.DHC.EnergyTransferStations.CoolingDirectUncontrolled coo(
    show_T=true,
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    dpSup=6000,
    dpRet=6000)
    "Direct cooling energy transfer station"
    annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));

  Buildings.Fluid.Sources.Boundary_pT souDis(
    redeclare package Medium = Medium,
    p(displayUnit="Pa") = 300000 + 100000,
    use_T_in=true,
    T=280.15,
    nPorts=1)
    "District (primary) source"
    annotation (Placement(transformation(extent={{-100,-20},{-80,0}})));

  Buildings.Fluid.Sources.Boundary_pT sinDis(
    redeclare package Medium = Medium,
    p(displayUnit="Pa") = 300000,
    T=289.15,
    nPorts=1)
    "District-side (primary) sink"
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));

  Buildings.Fluid.HeatExchangers.HeaterCooler_u loa(
    redeclare package Medium = Medium,
    allowFlowReversal=false,
    m_flow_nominal=m_flow_nominal,
    from_dp=false,
    linearizeFlowResistance=true,
    show_T=true,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    Q_flow_nominal=-1,
    dp_nominal=0)
    "Aggregate building cooling load"
    annotation (Placement(transformation(extent={{80,-20},{100,0}})));

  Modelica.Blocks.Sources.CombiTimeTable QCoo(
    table=[0,-100E3; 6,-80E3; 6,-50E3; 12,-20E3; 18,-150E3; 24,-100E3],
    timeScale=3600,
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
    "Cooling demand"
    annotation (Placement(transformation(extent={{-80,80},{-60,100}})));

  Modelica.Blocks.Sources.Ramp ram(
    height=1,
    duration(displayUnit="h") = 18000,
    startTime(displayUnit="h") = 3600)
    "Ramp load from zero"
    annotation (Placement(transformation(extent={{-80,40},{-60,60}})));

  Modelica.Blocks.Math.Product pro
    "Multiplyer to ramp load from zero"
    annotation (Placement(transformation(extent={{40,46},{60,66}})));

  Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage val(
    redeclare package Medium = Medium,
    allowFlowReversal=false,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=10000,
    riseTime=10,
    dpFixed_nominal=100000)
    "In-building terminal control valve"
    annotation (Placement(transformation(extent={{120,-20},{140,0}})));

  Modelica.Blocks.Sources.RealExpression Q_flow_max(
    y=-Q_flow_nominal)
    "Maximum Q_flow"
    annotation (Placement(transformation(extent={{-40,68},{-20,88}})));

  Modelica.Blocks.Math.Division div
    "Division"
    annotation (Placement(transformation(extent={{0,74},{20,94}})));

  Modelica.Blocks.Sources.Trapezoid tra(
    amplitude=2,
    rising(displayUnit="h") = 10800,
    width(displayUnit="h") = 10800,
    falling(displayUnit="h") = 10800,
    period(displayUnit="h") = 43200,
    offset=273.15 + 6)
    "District supply temperature trapezoid signal"
    annotation (Placement(transformation(extent={{-140,-16},{-120,4}})));

equation
  connect(tra.y, souDis.T_in)
    annotation (Line(points={{-119,-6},{-102,-6}}, color={0,0,127}));
  connect(QCoo.y[1], div.u1)
    annotation (Line(points={{-59,90},{-2,90}}, color={0,0,127}));
  connect(Q_flow_max.y, div.u2)
    annotation (Line(points={{-19,78},{-2,78}}, color={0,0,127}));
  connect(div.y, val.y)
    annotation (Line(points={{21,84},{130,84},{130,2}}, color={0,0,127}));
  connect(QCoo.y[1], pro.u1)
    annotation (Line(points={{-59,90},{-50,90},{-50,62},{38,62}}, color={0,0,127}));
  connect(ram.y, pro.u2)
    annotation (Line(points={{-59,50},{38,50}}, color={0,0,127}));
  connect(loa.u, pro.y)
    annotation (Line(points={{78,-4},{70,-4},{70,56},{61,56}},color={0,0,127}));
  connect(loa.port_b, val.port_a)
    annotation (Line(points={{100,-10},{120,-10}}, color={0,127,255}));
  connect(coo.port_a1, souDis.ports[1])
    annotation (Line(points={{-10,-44},{-20,-44},{-20,-10},{-80,-10}},
     color={0,127,255}));
  connect(sinDis.ports[1], coo.port_b2)
    annotation (Line(points={{-80,-90},{-20,-90},{-20,-56},{-10,-56}},
     color={0,127,255}));
  connect(loa.port_a, coo.port_b1)
    annotation (Line(points={{80,-10},{20,-10},{20,-44},{10,-44}},
     color={0,127,255}));
  connect(val.port_b, coo.port_a2)
    annotation (Line(points={{140,-10},{150,-10},{150,-90},{20,-90},{20,-56},
     {10,-56}}, color={0,127,255}));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false,
      extent={{-160,-120},{160,120}})),
  __Dymola_Commands(file=
    "modelica://Buildings/Resources/Scripts/Dymola/Applications/DHC/EnergyTransferStations/Examples/CoolingDirectUncontrolled.mos"
    "Simulate and plot"),
  experiment(
    StartTime=0,
    StopTime=86400,
    Tolerance=1e-06),
    Documentation(info="<html>
<p>
This model provides an example for the direct cooling energy transfer station 
model, which does not contain bridged piping nor temperature control. The 
ultimate control lies with the two-way modulating valve at the lumped, terminal 
building load. The valve is actuated proportionally to the instantaneous 
cooling load with respect to the maximum load. This reflects a quasi linear 
relationship between the opening of an equal-percentage valve and the heat 
transfer rate across a cooling coil.
</p>
</html>", revisions="<html>
<ul>
<li>December 5, 2019, by Kathryn Hinkelman:<br/>First implementation. </li>
</ul>
</html>"));
end CoolingDirectUncontrolled;
