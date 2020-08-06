within Buildings.Applications.DHC.EnergyTransferStations;
model CoolingDirectControlled
  "Direct cooling ETS model for district energy systems with a bypass pipe and controlled building supply temperature"
  extends Buildings.Fluid.Interfaces.PartialFourPortInterface(
    redeclare final package Medium1 = Medium,
    redeclare final package Medium2 = Medium,
    final m1_flow_nominal = mDis_flow_nominal,
    final m2_flow_nominal = mBui_flow_nominal);

 replaceable package Medium =
   Modelica.Media.Interfaces.PartialMedium "Medium in the component";

  parameter Modelica.SIunits.MassFlowRate mBui_flow_nominal(
    final min=0,
    final start=0.5)
    "Nominal mass flow rate of building cooling side";

  // pressure drops
  parameter Modelica.SIunits.PressureDifference dpConVal_nominal(
    final min=0,
    displayUnit="Pa")=6000
    "Nominal pressure drop in the control valve";

  parameter Modelica.SIunits.PressureDifference dpCheVal_nominal(
    final min=0,
    displayUnit="Pa")=6000
    "Nominal pressure drop in the check valve";

  // Controller parameters
  parameter Modelica.Blocks.Types.SimpleController controllerType=
    Modelica.Blocks.Types.SimpleController.PI
    "Type of controller"
    annotation(Dialog(tab="Controller"));

  parameter Real k(final min=0,final unit="1") = 1
    "Gain of controller"
    annotation(Dialog(tab="Controller"));

  parameter Modelica.SIunits.Time Ti(
    final min=Modelica.Constants.small)=120
    "Time constant of integrator block"
     annotation (Dialog(tab="Controller", enable=
       controllerType == Modelica.Blocks.Types.SimpleController.PI or
       controllerType == Modelica.Blocks.Types.SimpleController.PID));

  parameter Modelica.SIunits.Time Td(final min=0)=0.1
    "Time constant of derivative block"
     annotation (Dialog(tab="Controller", enable=
       controllerType == Modelica.Blocks.Types.SimpleController.PD or
       controllerType == Modelica.Blocks.Types.SimpleController.PID));

  parameter Real wp(final min=0)=1
    "Set-point weight for Proportional block (0..1)"
    annotation(Dialog(tab="Controller"));

  parameter Real wd(final min=0) = 0
    "Set-point weight for Derivative block (0..1)"
    annotation(Dialog(tab="Controller", enable=
       controllerType == Modelica.Blocks.Types.SimpleController.PD or
       controllerType == Modelica.Blocks.Types.SimpleController.PID));

  parameter Real Ni(
    final min=100*Modelica.Constants.eps)=0.9
    "Ni*Ti is time constant of anti-windup compensation"
    annotation(Dialog(tab="Controller", enable=
       controllerType == Modelica.Blocks.Types.SimpleController.PI or
       controllerType == Modelica.Blocks.Types.SimpleController.PID));

  parameter Real Nd(
    final min=100*Modelica.Constants.eps)=10
    "The higher Nd, the more ideal the derivative block"
    annotation(Dialog(tab="Controller", enable=
       controllerType == Modelica.Blocks.Types.SimpleController.PD or
       controllerType == Modelica.Blocks.Types.SimpleController.PID));

  parameter Modelica.Blocks.Types.InitPID initType=
    Modelica.Blocks.Types.InitPID.DoNotUse_InitialIntegratorState
    "Type of initialization (1: no init, 2: steady state, 3: initial state, 
      4: initial output)"
    annotation(Dialog(group="Initialization",tab="Controller"));

  parameter Real xi_start=0
    "Initial or guess value value for integrator output (= integrator state)"
    annotation (Dialog(group="Initialization",
      tab="Controller", enable=
        controllerType == Modelica.Blocks.Types.SimpleController.PI or
        controllerType == Modelica.Blocks.Types.SimpleController.PID));

  parameter Real xd_start=0
    "Initial or guess value for state of derivative block"
    annotation (Dialog(group="Initialization",
      tab="Controller", enable=
        controllerType == Modelica.Blocks.Types.SimpleController.PD or
        controllerType == Modelica.Blocks.Types.SimpleController.PID));

  // Advanced parameters
  parameter Modelica.Fluid.Types.Dynamics energyDynamics=
    Modelica.Fluid.Types.Dynamics.FixedInitial
    "Type of energy balance: dynamic (3 initialization options) or steady state"
    annotation(Dialog(tab="Advanced"));

  parameter Modelica.Fluid.Types.Dynamics massDynamics=energyDynamics
    "Type of mass balance: dynamic (3 initialization options) or steady state"
    annotation(Dialog(tab="Advanced"));

//  parameter Modelica.SIunits.PressureDifference[3] dp_nominal=500*{1,-1,1}
//    "Nominal pressure drop in pipe junctions"
//    annotation(Dialog(tab="Advanced"));

  Modelica.Blocks.Interfaces.RealInput TSetBuiSup
    "Building supply setpoint temperature"
    annotation (Placement(transformation(extent={{-140,-140},{-100,-100}})));

  Modelica.Blocks.Interfaces.RealOutput Q_flow(
    final quantity="HeatFlowRate",
    final unit="W",
    displayUnit="kW") "Measured heat flow rate at the ETS"
    annotation (Placement(transformation(extent={{100,140},{120,160}})));

  Modelica.Blocks.Interfaces.RealOutput E(
    final quantity="Energy",
    final unit="J",
    displayUnit="kWh") "Measured energy consumption at the ETS"
    annotation (Placement(transformation(extent={{100,100},{120,120}})));

  Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage conVal(
    redeclare final package Medium = Medium,
    final m_flow_nominal=mDis_flow_nominal,
    final dpValve_nominal=dpConVal_nominal,
    riseTime(displayUnit="s") = 60) "Control valve"
    annotation (Placement(transformation(extent={{30,70},{50,50}})));

  Fluid.FixedResistances.CheckValve cheVal(
    redeclare final package Medium = Medium,
    allowFlowReversal=false,
    dpValve_nominal=dpCheVal_nominal,
    final m_flow_nominal=mByp_flow_nominal) "Check valve (backflow preventer)"
    annotation (Placement(transformation(
      extent={{10,-10},{-10,10}},origin={2,20})));

  Buildings.Fluid.Sensors.MassFlowRate senMasFlo(
    redeclare final package Medium = Medium)
    "District supply mass flow rate sensor"
    annotation (Placement(transformation(extent={{-90,50},{-70,70}})));

  Buildings.Fluid.Sensors.TemperatureTwoPort senTDisSup(
    redeclare final package Medium = Medium,
    final m_flow_nominal=mDis_flow_nominal)
    "District supply temperature sensor"
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));

  Buildings.Fluid.Sensors.TemperatureTwoPort senTDisRet(
    redeclare final package Medium = Medium,
    final m_flow_nominal=mDis_flow_nominal)
    "District return temperature sensor"
    annotation (Placement(transformation(extent={{60,50},{80,70}})));

  Buildings.Fluid.Sensors.TemperatureTwoPort senTBuiSup(
    redeclare final package Medium = Medium,
    final m_flow_nominal=mBui_flow_nominal)
    "Building supply temperature sensor"
    annotation (Placement(transformation(extent={{-40,-70},{-60,-50}})));

  Modelica.Blocks.Continuous.Integrator int(final k=1)
    "Integration"
    annotation (Placement(transformation(extent={{70,100},{90,120}})));

  Modelica.Blocks.Math.Add dTDis(final k1=+1, final k2=-1)
    "Temperature difference on the district side"
    annotation (Placement(transformation(extent={{-40,114},{-20,94}})));

  Modelica.Blocks.Math.Product pro
    "Product"
    annotation (Placement(transformation(extent={{-10,100},{10,120}})));

  Modelica.Blocks.Math.Gain cp(final k=cp_default)
    "Specific heat multiplier to calculate heat flow rate"
    annotation (Placement(transformation(extent={{30,100},{50,120}})));

  Buildings.Controls.Continuous.LimPID conPID(
    final controllerType=controllerType,
    final k=k,
    final Ti=Ti,
    final Td=Td,
    final yMax=1,
    final yMin=0,
    final wp=wp,
    final wd=wd,
    final Ni=Ni,
    final Nd=Nd,
    final initType=initType,
    final xi_start=xi_start,
    final xd_start=xd_start,
    final reverseAction=false)
    annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
protected
  parameter Modelica.SIunits.MassFlowRate mDis_flow_nominal= mBui_flow_nominal
    "Nominal mass flow rate of district cooling side";

  parameter Modelica.SIunits.MassFlowRate mByp_flow_nominal= mBui_flow_nominal
    "Nominal mass flow rate through the bypass segment";

  final parameter Medium.ThermodynamicState sta_default = Medium.setState_pTX(
    T=Medium.T_default,
    p=Medium.p_default,
    X=Medium.X_default) "Medium state at default properties";
  final parameter Modelica.SIunits.SpecificHeatCapacity cp_default=
    Medium.specificHeatCapacityCp(sta_default)
    "Specific heat capacity of the fluid";

equation
  connect(int.y,E)
    annotation (Line(points={{91,110},{110,110}}, color={0,0,127}));
  connect(pro.y,cp. u)
    annotation (Line(points={{11,110},{28,110}}, color={0,0,127}));
  connect(cp.y,int. u)
    annotation (Line(points={{51,110},{68,110}}, color={0,0,127}));
  connect(cp.y,Q_flow)
    annotation (Line(points={{51,110},{60,110},{60,150},{110,150}}, color={0,0,127}));
  connect(conVal.port_a, port_a2) annotation (Line(points={{30,60},{20,60},{20,-60},
          {100,-60}}, color={0,127,255}));
  connect(conVal.port_a, cheVal.port_a) annotation (Line(points={{30,60},{20,60},
          {20,20},{12,20}}, color={0,127,255}));
  connect(conVal.port_b, senTDisRet.port_a) annotation (Line(points={{50,60},{54,
          60},{54,60},{60,60}}, color={0,127,255}));
  connect(senTDisRet.port_b, port_b1)
    annotation (Line(points={{80,60},{100,60}}, color={0,127,255}));
  connect(senTBuiSup.port_b, port_b2)
    annotation (Line(points={{-60,-60},{-100,-60}}, color={0,127,255}));
  connect(senMasFlo.m_flow, pro.u1)
    annotation (Line(points={{-80,71},{-80,116},{-12,116}}, color={0,0,127}));
  connect(dTDis.y, pro.u2)
    annotation (Line(points={{-19,104},{-12,104}}, color={0,0,127}));
  connect(senTDisRet.T, dTDis.u1) annotation (Line(points={{70,71},{70,80},{-48,
          80},{-48,98},{-42,98}}, color={0,0,127}));
  connect(senTDisSup.T, dTDis.u2)
    annotation (Line(points={{-50,71},{-50,110},{-42,110}}, color={0,0,127}));
  connect(port_a1, senMasFlo.port_a)
    annotation (Line(points={{-100,60},{-90,60}}, color={0,127,255}));
  connect(senMasFlo.port_b, senTDisSup.port_a)
    annotation (Line(points={{-70,60},{-60,60}}, color={0,127,255}));
  connect(senTDisSup.port_b, senTBuiSup.port_a) annotation (Line(points={{-40,60},
          {-20,60},{-20,-60},{-40,-60}}, color={0,127,255}));
  connect(senTDisSup.port_b, cheVal.port_b) annotation (Line(points={{-40,60},{-20,
          60},{-20,20},{-8,20}}, color={0,127,255}));
  connect(TSetBuiSup, conPID.u_s) annotation (Line(points={{-120,-120},{-80,-120},
          {-80,-20},{-62,-20}}, color={0,0,127}));
  connect(senTBuiSup.T, conPID.u_m)
    annotation (Line(points={{-50,-49},{-50,-32}}, color={0,0,127}));
  connect(conPID.y, conVal.y)
    annotation (Line(points={{-39,-20},{40,-20},{40,48}}, color={0,0,127}));
  annotation (defaultComponentName="coo",
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,-56},{100,-64}},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-100,64},{100,56}},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={175,175,175},
          fillColor={35,138,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-22,8},{22,-8}},
          lineColor={0,0,0},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-80,68},{-22,52}},
          lineColor={0,0,0},
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-38,68},{-22,-68}},
          lineColor={0,0,0},
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-80,-52},{-22,-68}},
          lineColor={0,0,0},
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{22,-52},{80,-68}},
          lineColor={0,0,0},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{22,68},{38,-68}},
          lineColor={0,0,0},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{22,68},{80,52}},
          lineColor={0,0,0},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}),
    Diagram(coordinateSystem(preserveAspectRatio=false,
        extent={{-100,-160},{100,160}})),
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>
Direct cooling energy transfer station (ETS) model with a bypass pipe 
and controlled building supply temperature. The design is based on a typical 
district cooling ETS described in ASHRAE's 
<a href=\"https://www.ashrae.org/technical-resources/bookstore/district-heating-and-cooling-guides\">
District Cooling Guide</a>.  
As shown in the figure below, the district fluid flows directly from the district into
the building distribution system. The control valve ensures that the supply temperature 
to the building meets the prescribed setpoint by mixing returned chilled water from the 
building with the district supply. This configuration naturally results in a fluctuating 
district return temperature.
</p>
<p align=\"center\">
<img src=\"modelica://Buildings/Resources/Images/Applications/DHC/EnergyTransferStations/CoolingDirectControlled.PNG\" alt=\"DHC.ETS.CoolingDirectControlled\"/>
</p>
<h4>Reference</h4>
<p>
American Society of Heating, Refrigeration and Air-Conditioning Engineers. 
(2013). Chapter 5: End User Interface. In <i>District Cooling Guide</i>. 1st Edition. 
</p>
</html>", revisions="<html>
<ul>
<li>December 12, 2019, by Kathryn Hinkelman:<br/>First implementation. </li>
</ul>
</html>"));
end CoolingDirectControlled;
