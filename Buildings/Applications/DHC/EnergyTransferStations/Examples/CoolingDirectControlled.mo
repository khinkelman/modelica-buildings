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
    k=0.5,
    Ti=200)
    annotation (Placement(transformation(extent={{20,20},{40,40}})));

  Modelica.Blocks.Sources.Constant TSetBuiSup(k=273.15 + 7)
    "Building supply temperature setpont"
    annotation (Placement(transformation(extent={{-100,8},{-80,28}})));

  Buildings.Fluid.Sources.Boundary_pT sinDis(
    redeclare package Medium = Medium,
    p=300000,
    nPorts=1)
    "District sink"
    annotation (Placement(transformation(extent={{100,26},{80,46}})));

  Modelica.Blocks.Sources.RealExpression TDisSupNoi(
    y=(273.15 + 7) + 2*sin(time*4*3.14/86400))
    "Sinusoidal noise signal for district supply temperature"
    annotation (Placement(transformation(extent={{-100,30},{-80,50}})));

  Buildings.Fluid.HeatExchangers.HeaterCooler_u loa(
    redeclare package Medium = Medium,
    allowFlowReversal=false,
    m_flow_nominal=mBui_flow_nominal,
    from_dp=false,
    linearizeFlowResistance=true,
    show_T=true,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    Q_flow_nominal=-1,
    dp_nominal=100000)
    "Aggregate building cooling load"
    annotation (Placement(transformation(extent={{40,0},{60,-20}})));

  Buildings.Fluid.Movers.FlowControlled_m_flow pum(
    redeclare replaceable package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    m_flow_nominal=mBui_flow_nominal,
    addPowerToMedium=false,
    nominalValuesDefineDefaultPressureCurve=true,
    constantMassFlowRate=mBui_flow_nominal)
    "Building primary pump"
    annotation (Placement(transformation(extent={{0,0},{20,-20}})));

  Modelica.Blocks.Math.Gain gai(k=-1/(cp*(16-7)))
    "Multiplier gain for calculating m_flow"
    annotation (Placement(transformation(extent={{0,-100},{20,-80}})));

  Modelica.Blocks.Sources.Ramp ram(
    height=1,
    duration(displayUnit="h") = 3600,
    startTime(displayUnit="h") = 0)
    "Ramp load from zero"
    annotation (Placement(transformation(extent={{-100,-60},{-80,-40}})));

  Modelica.Blocks.Math.Product pro
    "Multiplyer to ramp load from zero"
    annotation (Placement(transformation(extent={{-60,-66},{-40,-46}})));

  Buildings.Fluid.Sources.Boundary_pT souDis(
    redeclare package Medium = Medium,
    p=350000,
    use_T_in=true,
    T=280.15,
    nPorts=1)
    "District source"
    annotation (Placement(transformation(extent={{-40,26},{-20,46}})));

  Modelica.Blocks.Sources.CombiTimeTable QCoo(
    table=[0,-100E3; 6,-80E3; 6,-50E3; 12,-20E3; 18,-150E3; 24,-100E3],
    timeScale=3600,
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
    "Cooling demand"
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));

equation
  connect(TSetBuiSup.y, coo.TSetBuiSup)
    annotation (Line(points={{-79,18},{18,18}},    color={0,0,127}));
  connect(pum.port_b, loa.port_a)
    annotation (Line(points={{20,-10},{40,-10}}, color={0,127,255}));
  connect(ram.y, pro.u1)
    annotation (Line(points={{-79,-50},{-62,-50}},
                                                color={0,0,127}));
  connect(QCoo.y[1], pro.u2)
    annotation (Line(points={{-79,-90},{-70,-90},{-70,-62},{-62,-62}},
      color={0,0,127}));
  connect(pro.y, gai.u)
    annotation (Line(points={{-39,-56},{-10,-56},{-10,-90},{-2,-90}},
      color={0,0,127}));
  connect(TDisSupNoi.y, souDis.T_in)
    annotation (Line(points={{-79,40},{-42,40}},   color={0,0,127}));

  connect(souDis.ports[1], coo.port_a1)
    annotation (Line(points={{-20,36},{20,36}}, color={0,127,255}));
  connect(coo.port_b2, pum.port_a) annotation (Line(points={{20,24},{-10,24},{
          -10,-10},{0,-10}}, color={0,127,255}));
  connect(loa.port_b, coo.port_a2) annotation (Line(points={{60,-10},{70,-10},{
          70,24},{40,24}}, color={0,127,255}));
  connect(coo.port_b1, sinDis.ports[1])
    annotation (Line(points={{40,36},{80,36}}, color={0,127,255}));
  connect(pro.y, pum.m_flow_in)
    annotation (Line(points={{-39,-56},{10,-56},{10,-22}}, color={0,0,127}));
  connect(gai.y, loa.u) annotation (Line(points={{21,-90},{30,-90},{30,-16},{38,
          -16}}, color={0,0,127}));
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
