within Buildings.Applications.DHC.EnergyTransferStations;
model CoolingDirectUncontrolled
  "Direct cooling ETS model for district energy systems without in-building 
  pumping or deltaT control"
  extends Buildings.Fluid.Interfaces.PartialFourPortInterface(
    redeclare final package Medium1 = Medium,
    redeclare final package Medium2 = Medium,
    final m1_flow_nominal = m_flow_nominal,
    final m2_flow_nominal = m_flow_nominal,
    final allowFlowReversal2 = allowFlowReversal1);

 replaceable package Medium =
   Modelica.Media.Interfaces.PartialMedium "Medium in the component";

  parameter Boolean show_heaFlo = false
    "Set to true to output the heat flow rate transferred to the connected load"
    annotation(Evaluate=true);

  parameter Modelica.SIunits.MassFlowRate m_flow_nominal(
    final min=0,
    final start=0.5)
    "Nominal mass flow rate";

  // pressure drops
  parameter Modelica.SIunits.PressureDifference dpSup(
    final min=0,
    displayUnit="Pa")=5000
  "Pressure drop in the ETS supply side";

  parameter Modelica.SIunits.PressureDifference dpRet(
    final min=0,
    displayUnit="Pa")=5000
  "Pressure drop in the ETS return side";

  Modelica.Blocks.Interfaces.RealOutput Q_flow(
    final quantity="HeatFlowRate",
    final unit="W",
    displayUnit="kW") if show_heaFlo
    "Measured heat flow rate at the ETS"
    annotation (Placement(transformation(extent={{100,140},{120,160}})));

  Modelica.Blocks.Interfaces.RealOutput E(
    final quantity="Energy",
    final unit="J",
    displayUnit="kWh") if show_heaFlo
    "Measured energy consumption at the ETS"
    annotation (Placement(transformation(extent={{100,100},{120,120}})));

  Buildings.Fluid.FixedResistances.PressureDrop pipSup(
    redeclare final package Medium = Medium,
    final allowFlowReversal=allowFlowReversal1,
    final m_flow_nominal=m_flow_nominal,
    final dp_nominal=dpSup)
    "Supply pipe"
    annotation (Placement(transformation(extent={{-30,50},{-10,70}})));

  Buildings.Fluid.FixedResistances.PressureDrop pipRet(
    redeclare final package Medium = Medium,
    final allowFlowReversal=allowFlowReversal1,
    final m_flow_nominal=m_flow_nominal,
    final dp_nominal=dpRet)
    "Return pipe"
    annotation (Placement(transformation(extent={{80,-70},{60,-50}})));

  Buildings.Fluid.Sensors.MassFlowRate senMasFlo(
    redeclare final package Medium = Medium,
    final allowFlowReversal=allowFlowReversal1) if show_heaFlo
    "District supply mass flow rate sensor"
    annotation (Placement(transformation(extent={{-90,50},{-70,70}})));

  Buildings.Fluid.Sensors.TemperatureTwoPort senTDisSup(
    redeclare final package Medium = Medium,
    final allowFlowReversal=allowFlowReversal1,
    final m_flow_nominal=m_flow_nominal) if show_heaFlo
    "District supply temperature sensor"
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));

  Buildings.Fluid.Sensors.TemperatureTwoPort senTDisRet(
    redeclare final package Medium = Medium,
    final allowFlowReversal=allowFlowReversal1,
    final m_flow_nominal=m_flow_nominal) if show_heaFlo
    "District return temperature sensor"
    annotation (Placement(transformation(extent={{60,50},{80,70}})));

  Modelica.Blocks.Continuous.Integrator int(final k=1) if show_heaFlo
    "Integration"
    annotation (Placement(transformation(extent={{70,100},{90,120}})));

  Modelica.Blocks.Math.Add dTDis(final k1=+1, final k2=-1) if show_heaFlo
    "Temperature difference on the district side"
    annotation (Placement(transformation(extent={{-30,114},{-10,94}})));

  Modelica.Blocks.Math.Product pro if show_heaFlo
    "Delta T times flow rate"
    annotation (Placement(transformation(extent={{2,100},{22,120}})));

  Modelica.Blocks.Math.Gain cp(final k=cp_default) if show_heaFlo
    "Specific heat multiplier to calculate heat flow rate"
    annotation (Placement(transformation(extent={{30,100},{50,120}})));

protected
  final parameter Medium.ThermodynamicState sta_default = Medium.setState_pTX(
    T=Medium.T_default,
    p=Medium.p_default,
    X=Medium.X_default) "Medium state at default properties";
  final parameter Modelica.SIunits.SpecificHeatCapacity cp_default=
    Medium.specificHeatCapacityCp(sta_default)
    "Specific heat capacity of the fluid";

equation
  if not show_heaFlo then
    connect(pipSup.port_a, port_a1)
      annotation (Line(points={{-30,60},{-36,60},{-36,40},{-94,40},{-94,60},{-100,60}},
        color={0,127,255}));
    connect(pipRet.port_b, port_b1)
      annotation (Line(points={{60,-60},{40,-60},{40,60},{56,60},{56,40},{84,40},{84,60},{100,60}},
        color={0,127,255}));
  end if;

  connect(int.y,E)
    annotation (Line(points={{91,110},{110,110}}, color={0,0,127}));
  connect(pro.y, cp.u)
    annotation (Line(points={{23,110},{28,110}}, color={0,0,127}));
  connect(cp.y, int.u)
    annotation (Line(points={{51,110},{68,110}}, color={0,0,127}));
  connect(cp.y, Q_flow)
    annotation (Line(points={{51,110},{60,110},{60,150},{110,150}}, color={0,0,127}));
  connect(port_a1, senMasFlo.port_a)
    annotation (Line(points={{-100,60},{-90,60}}, color={0,127,255}));
  connect(senMasFlo.m_flow, pro.u1)
    annotation (Line(points={{-80,71},{-80,116},{0,116}}, color={0,0,127}));
  connect(senTDisSup.T, dTDis.u2)
    annotation (Line(points={{-50,71},{-50,110},{-32,110}}, color={0,0,127}));
  connect(senTDisRet.T, dTDis.u1)
    annotation (Line(points={{70,71},{70,80},{-46,80},{-46,98},{-32,98}}, color={0,0,127}));
  connect(dTDis.y, pro.u2)
    annotation (Line(points={{-9,104},{0,104}}, color={0,0,127}));
  connect(senMasFlo.port_b, senTDisSup.port_a)
    annotation (Line(points={{-70,60},{-60,60}}, color={0,127,255}));
  connect(senTDisSup.port_b, pipSup.port_a)
    annotation (Line(points={{-40,60},{-30,60}}, color={0,127,255}));
  connect(pipSup.port_b, port_b2)
    annotation (Line(points={{-10,60},{0,60},{0,-60},{-100,-60}}, color={0,127,255}));
  connect(port_a2, pipRet.port_a)
    annotation (Line(points={{100,-60},{80,-60}}, color={0,127,255}));
  connect(pipRet.port_b, senTDisRet.port_a)
    annotation (Line(points={{60,-60},{40,-60},{40,60},{60,60}}, color={0,127,255}));
  connect(senTDisRet.port_b, port_b1)
    annotation (Line(points={{80,60},{100,60}}, color={0,127,255}));
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
          extent={{-80,68},{-22,52}},
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
          extent={{-80,-52},{-22,-68}},
          lineColor={0,0,0},
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{22,68},{80,52}},
          lineColor={0,0,0},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-38,68},{-22,-68}},
          lineColor={0,0,0},
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}),
    Diagram(coordinateSystem(preserveAspectRatio=false,
        extent={{-100,-100},{100,160}})),
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>
Direct cooling energy transfer station (ETS) model with no bridge piping 
nor temperature control. The design is based on a typical district cooling ETS 
described in ASHRAE's 
<a href=\"https://www.ashrae.org/technical-resources/bookstore/district-heating-and-cooling-guides\">
District Cooling Guide</a>.  
As shown in the figure below, the district fluid flows directly from the district into
the building distribution system.  
</p>
<p align=\"center\">
<img src=\"modelica://Buildings/Resources/Images/Applications/DHC/EnergyTransferStations/CoolingDirectUncontrolled.PNG\" alt=\"DHC.ETS.CoolingDirectUncontrolled\"/>
</p>
<h4>Reference</h4>
<p>
American Society of Heating, Refrigeration and Air-Conditioning Engineers. 
(2013). Chapter 5: End User Interface. In <i>District Cooling Guide</i>. 1st Edition. 
</p>
</html>", revisions="<html>
<ul>
<li>November 13, 2019, by Kathryn Hinkelman:<br/>First implementation. </li>
</ul>
</html>"));
end CoolingDirectUncontrolled;
