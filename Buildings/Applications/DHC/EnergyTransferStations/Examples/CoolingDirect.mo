within Buildings.Applications.DHC.EnergyTransferStations.Examples;
model CoolingDirect "Example model for direct cooling energy transfer station"
  extends Modelica.Icons.Example;

  package Medium = Buildings.Media.Water;

  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal = 1000
    "Nominal cooling load";

  parameter Modelica.SIunits.MassFlowRate m_flow_nominal = Q_flow_nominal/(cp*(16 - 7))
    "Nominal mass flow rate";

 parameter Modelica.SIunits.SpecificHeatCapacity cp=
   Medium.specificHeatCapacityCp(
      Medium.setState_pTX(Medium.p_default, Medium.T_default, Medium.X_default))
    "Default specific heat capacity of medium";

  Buildings.Applications.DHC.EnergyTransferStations.CoolingDirect coo(
      m1_flow_nominal=m_flow_nominal,
    dpSup=900,
    dpRet=900)
    "Direct cooling energy transfer station"
    annotation (Placement(transformation(extent={{-10,0},{10,20}})));
  Fluid.Sources.Boundary_pT           souDis(
    redeclare package Medium = Medium,
    p(displayUnit="Pa") = 300000 + 800,
    T=280.15,
    nPorts=1) "District (primary) source" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-50,70})));
  Fluid.Sources.Boundary_pT           sinDis(
    redeclare package Medium = Medium,
    p=300000,
    T=289.15,
    nPorts=1) "District-side (primary) sink" annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={50,70})));
  Fluid.Sensors.TemperatureTwoPort           TDisSup(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    T_start=280.15) "District-side (primary) supply temperature sensor"
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=-90,
        origin={-50,30})));
  Fluid.Sensors.TemperatureTwoPort           TDisRet(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    T_start=289.15) "District-side (primary) return temperature sensor"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,30})));
  Fluid.Sensors.TemperatureTwoPort           TBuiRet(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    T_start=289.15) "Building-side (secondary) return temperature sensor"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,-40})));
  Fluid.Sensors.TemperatureTwoPort           TBuiSup(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    T_start=280.15) "Building-side (secondary) supply temperature sensor"
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-50,-40})));
public
  Fluid.HeatExchangers.HeaterCooler_u           loa(
    redeclare final package Medium = Medium,
    final allowFlowReversal=false,
    final m_flow_nominal=m_flow_nominal,
    final from_dp=false,
    final linearizeFlowResistance=true,
    final show_T=true,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    final Q_flow_nominal=-1,
    final dp_nominal=100) "Aggregate building cooling load"
    annotation (Placement(transformation(extent={{-10,-80},{10,-60}})));
  Fluid.Movers.FlowControlled_m_flow           pumBui(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    m_flow_nominal=m_flow_nominal,
    inputType=Buildings.Fluid.Types.InputType.Constant,
    nominalValuesDefineDefaultPressureCurve=true,
    dp_nominal=0) "Building-side (secondary) pump" annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-50,-10})));
  Fluid.Storage.ExpansionVessel           exp(redeclare package Medium = Medium,
      V_start=1000) "Expansion tank"
    annotation (Placement(transformation(extent={{-30,0},{-10,-20}})));
  Modelica.Blocks.Sources.Ramp QCooRam(
    height=-Q_flow_nominal,
    duration(displayUnit="h") = 43200,
    startTime=6) "Ramping cooling load"
    annotation (Placement(transformation(extent={{-80,-74},{-60,-54}})));
  Modelica.Blocks.Sources.RealExpression dT(y=TDisRet.T - TDisSup.T)
    "Temperature change across ETS (measured at district side)"
    annotation (Placement(transformation(extent={{46,80},{96,100}})));
equation
  connect(souDis.ports[1], TDisSup.port_a)
    annotation (Line(points={{-50,60},{-50,40}}, color={0,127,255}));
  connect(TDisSup.port_b, coo.port_a1)
    annotation (Line(points={{-50,20},{-50,16},{-10,16}}, color={0,127,255}));
  connect(coo.port_b2, pumBui.port_a)
    annotation (Line(points={{-10,4},{-50,4},{-50,0}}, color={0,127,255}));
  connect(pumBui.port_b, TBuiSup.port_a)
    annotation (Line(points={{-50,-20},{-50,-30}}, color={0,127,255}));
  connect(TBuiSup.port_b, loa.port_a) annotation (Line(points={{-50,-50},{-50,-70},
          {-10,-70}}, color={0,127,255}));
  connect(loa.port_b, TBuiRet.port_a)
    annotation (Line(points={{10,-70},{50,-70},{50,-50}},color={0,127,255}));
  connect(TBuiRet.port_b, coo.port_a2)
    annotation (Line(points={{50,-30},{50,4},{10,4}},color={0,127,255}));
  connect(coo.port_b1, TDisRet.port_a)
    annotation (Line(points={{10,16},{50,16},{50,20}},color={0,127,255}));
  connect(TDisRet.port_b, sinDis.ports[1])
    annotation (Line(points={{50,40},{50,60}}, color={0,127,255}));
  connect(exp.port_a, coo.port_b2)
    annotation (Line(points={{-20,0},{-20,4},{-10,4}}, color={0,127,255}));
  connect(QCooRam.y, loa.u)
    annotation (Line(points={{-59,-64},{-12,-64}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    __Dymola_Commands(file=
    "modelica://Buildings/Resources/Scripts/Dymola/Applications/DHC/EnergyTransferStations/Examples/CoolingDirect.mos"
    "Simulate and plot"),
  experiment(
    StartTime=0,
    StopTime=86400,
    Tolerance=1e-03));
end CoolingDirect;
