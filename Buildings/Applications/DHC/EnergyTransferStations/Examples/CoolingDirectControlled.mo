within Buildings.Applications.DHC.EnergyTransferStations.Examples;
model CoolingDirectControlled
  "Example model for direct cooling energy transfer station with a bypass pipe and controlled builidng supply temperature"
  extends Modelica.Icons.Example;

  package Medium = Buildings.Media.Water "Water medium";

//  parameter Modelica.SIunits.MassFlowRate mDis_flow_nominal=0.5
//    "Nominal mass flow rate of district cooling supply";

  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal=150000
    "Nominal cooling load";

  parameter Modelica.SIunits.MassFlowRate mBui_flow_nominal=
    Q_flow_nominal/(cp*(18 - 7))
    "Nominal mass flow rate";

 parameter Modelica.SIunits.SpecificHeatCapacity cp=
   Medium.specificHeatCapacityCp(
      Medium.setState_pTX(Medium.p_default, Medium.T_default, Medium.X_default))
    "Default specific heat capacity of medium";

  inner Modelica.Fluid.System system
    "System properties and default values"
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));

  Buildings.Applications.DHC.EnergyTransferStations.CoolingDirectControlled coo(
    show_T=true,
    redeclare package Medium = Medium,
    mBui_flow_nominal=mBui_flow_nominal,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.1,
    Ti=200,
    initType=Modelica.Blocks.Types.InitPID.InitialOutput,
    y_start=1)
    annotation (Placement(transformation(extent={{20,40},{40,60}})));

  Modelica.Blocks.Sources.Constant TSetBuiSup(k=273.15 + 7)
    "Building supply temperature setpont"
    annotation (Placement(transformation(extent={{-100,30},{-80,50}})));

  Buildings.Fluid.Sources.Boundary_pT sinDis(
    redeclare package Medium = Medium,
    p=300000,
    nPorts=1)
    "District sink"
    annotation (Placement(transformation(extent={{100,46},{80,66}})));

  Modelica.Blocks.Sources.RealExpression TDisSupNoi(y=(273.15 + 6) + sin(time*4
        *3.14/86400))
    "Sinusoidal noise signal for district supply temperature"
    annotation (Placement(transformation(extent={{-100,50},{-80,70}})));

  Buildings.Fluid.HeatExchangers.HeaterCooler_u loa(
    redeclare package Medium = Medium,
    m_flow_nominal=mBui_flow_nominal,
    from_dp=false,
    linearizeFlowResistance=true,
    show_T=true,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    Q_flow_nominal=-1,
    dp_nominal(displayUnit="bar") = 100000)
    "Aggregate building cooling load"
    annotation (Placement(transformation(extent={{40,20},{60,0}})));

  Buildings.Fluid.Movers.FlowControlled_m_flow pum(
    redeclare replaceable package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    m_flow_nominal=mBui_flow_nominal,
    addPowerToMedium=false,
    nominalValuesDefineDefaultPressureCurve=true,
    constantMassFlowRate=mBui_flow_nominal)
    "Building primary pump"
    annotation (Placement(transformation(extent={{0,20},{20,0}})));

  Modelica.Blocks.Math.Gain gai(k=-1/(cp*(16-7)))
    "Multiplier gain for calculating m_flow"
    annotation (Placement(transformation(extent={{-20,-40},{0,-20}})));

  Modelica.Blocks.Sources.Ramp ram(
    height=1,
    duration(displayUnit="h") = 3600,
    startTime(displayUnit="h") = 0) "Ramp load from zero"
    annotation (Placement(transformation(extent={{-100,-60},{-80,-40}})));

  Modelica.Blocks.Math.Product pro
    "Multiplyer to ramp load from zero"
    annotation (Placement(transformation(extent={{-60,-66},{-40,-46}})));

  Buildings.Fluid.Sources.Boundary_pT souDis(
    redeclare package Medium = Medium,
    p=350000,
    use_T_in=true,
    T(displayUnit="degC") = 280.15,
    nPorts=1)
    "District source"
    annotation (Placement(transformation(extent={{-40,46},{-20,66}})));

  Modelica.Blocks.Sources.CombiTimeTable QCoo(
    table=[0,-100E3; 6,-80E3; 6,-50E3; 12,-20E3; 18,-150E3; 24,-100E3],
    timeScale=3600,
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
    "Cooling demand"
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));

  Modelica.Blocks.Sources.Ramp TDisSup_ramp(
    height=-2,
    duration(displayUnit="h") = 21600,
    offset=273.15 + 7,
    startTime(displayUnit="h") = 43200) "Supply ramp"
    annotation (Placement(transformation(extent={{-48,86},{-28,106}})));
  Modelica.Blocks.Sources.BooleanStep conSta(startTime(displayUnit="h") = 3600)
    "ETS controller state"
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
equation
  connect(TSetBuiSup.y, coo.TSetBuiSup)
    annotation (Line(points={{-79,40},{18,40}},    color={0,0,127}));
  connect(pum.port_b, loa.port_a)
    annotation (Line(points={{20,10},{40,10}},   color={0,127,255}));
  connect(ram.y, pro.u1)
    annotation (Line(points={{-79,-50},{-62,-50}},
                                                color={0,0,127}));
  connect(QCoo.y[1], pro.u2)
    annotation (Line(points={{-79,-90},{-70,-90},{-70,-62},{-62,-62}},
      color={0,0,127}));
  connect(pro.y, gai.u)
    annotation (Line(points={{-39,-56},{-30,-56},{-30,-30},{-22,-30}},
      color={0,0,127}));

  connect(souDis.ports[1], coo.port_a1)
    annotation (Line(points={{-20,56},{20,56}}, color={0,127,255}));
  connect(coo.port_b2, pum.port_a) annotation (Line(points={{20,44},{-10,44},{
          -10,10},{0,10}},   color={0,127,255}));
  connect(loa.port_b, coo.port_a2) annotation (Line(points={{60,10},{70,10},{70,
          44},{40,44}},    color={0,127,255}));
  connect(coo.port_b1, sinDis.ports[1])
    annotation (Line(points={{40,56},{80,56}}, color={0,127,255}));
  connect(gai.y, pum.m_flow_in) annotation (Line(points={{1,-30},{10,-30},{10,
          -2},{10,-2}}, color={0,0,127}));
  connect(pro.y, loa.u) annotation (Line(points={{-39,-56},{30,-56},{30,4},{38,
          4}}, color={0,0,127}));
  connect(coo.trigger, conSta.y) annotation (Line(points={{18,35},{-60,35},{-60,
          0},{-79,0}}, color={255,0,255}));
  connect(TDisSupNoi.y, souDis.T_in)
    annotation (Line(points={{-79,60},{-42,60}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,
            -120},{120,120}})),
    Diagram(coordinateSystem(preserveAspectRatio=false,
        extent={{-120,-120},{120,120}})),
    __Dymola_Commands(file=
    "modelica://Buildings/Resources/Scripts/Dymola/Applications/DHC/EnergyTransferStations/Examples/CoolingDirectControlled.mos"
    "Simulate and plot"),
    experiment(
        StartTime=0,
        StopTime=86400,
        Tolerance=1e-06),
    Documentation(info="<html>
<p>
This model provides an example for the direct cooling energy transfer station 
model, which contains a bypass pipe and controls the building supply 
temperature. The building's primary variable speed pump is modulated to maintain
a constant temperature rise. Variation in the district supply temperature is modeled as 
sinusoidal to test the system's response. 
</p>
</html>", revisions="<html>
<ul>
<li>December 12, 2019, by Kathryn Hinkelman:<br/>First implementation. </li>
</ul>
</html>"));
end CoolingDirectControlled;
