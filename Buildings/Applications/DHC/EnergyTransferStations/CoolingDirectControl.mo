within Buildings.Applications.DHC.EnergyTransferStations;
model CoolingDirectControl
  "Direct cooling ETS model for district energy systems with in-building pumping and deltaT control"
  extends
    Buildings.Applications.DHC.EnergyTransferStations.BaseClasses.PartialCooling(
    indirectCooling=false,
    senTDisRetInd(m_flow_nominal=m1_flow_nominal),
    senTDisSup(m_flow_nominal=m1_flow_nominal));
  extends
    Buildings.Applications.DHC.EnergyTransferStations.BaseClasses.PartialControl;

 parameter Modelica.SIunits.SpecificHeatCapacity cp=
   Medium.specificHeatCapacityCp(
      Medium.setState_pTX(Medium.p_default, Medium.T_default, Medium.X_default))
    "Default specific heat capacity of medium";

  // mass flow rates
  parameter Modelica.SIunits.MassFlowRate m1_flow_nominal(min=0,start=0.5)
    "Nominal mass flow rate";

  // pressure drops
  parameter Modelica.SIunits.PressureDifference dpSup(displayUnit="Pa")=50
  "Pressure drop in the ETS supply side (piping, valves, etc.)";

  parameter Modelica.SIunits.PressureDifference dpRet(displayUnit="Pa")=50
  "Pressure drop in the ETS return side (piping, valves, etc.)";

  parameter Modelica.SIunits.PressureDifference dpByp(displayUnit="Pa")=10
  "Pressure drop in the bypass line (piping, valves, etc.)";

  Buildings.Fluid.FixedResistances.PressureDrop pipSup(
    redeclare package Medium = Medium,
    m_flow_nominal=m1_flow_nominal,
    dp_nominal=dpSup) "Supply pipe"
                                 annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-10,60})));
  Buildings.Fluid.FixedResistances.PressureDrop pipRet(
    redeclare package Medium = Medium,
    m_flow_nominal=m1_flow_nominal,
    dp_nominal=dpRet)
                   "Return pipe" annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={-10,-60})));
  Fluid.FixedResistances.PressureDrop pipByp(
    redeclare package Medium = Medium,
    m_flow_nominal=m1_flow_nominal,
    dp_nominal=dpSup) "Bypass pipe" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={30,0})));
  Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear val(m_flow_nominal=
        m1_flow_nominal, dpValve_nominal=50)
    annotation (Placement(transformation(extent={{40,-50},{20,-70}})));
  Fluid.Sensors.TemperatureTwoPort           TBuiRet(redeclare package Medium =
        Medium, m_flow_nominal=m1_flow_nominal)
    "Building-side (secondary) return temperature"
    annotation (Placement(transformation(extent={{80,-70},{60,-50}})));
  Fluid.FixedResistances.Junction jun(
    redeclare package Medium = Medium,
    m_flow_nominal={m1_flow_nominal,-m1_flow_nominal,0},
    dp_nominal=50*{1,-1,1})
    annotation (Placement(transformation(extent={{20,50},{40,70}})));
equation
  connect(con.y, val.y) annotation (Line(points={{-69,0},{-40,0},{-40,-80},{30,
          -80},{30,-72}}, color={0,0,127}));
  connect(TBuiRet.T, con.u_m) annotation (Line(points={{70,-49},{70,-30},{-80,
          -30},{-80,-12}}, color={0,0,127}));
  connect(port_a2, TBuiRet.port_a)
    annotation (Line(points={{100,-60},{80,-60}}, color={0,127,255}));
  connect(TBuiRet.port_b, val.port_1)
    annotation (Line(points={{60,-60},{40,-60}}, color={0,127,255}));
  connect(val.port_2, pipRet.port_a)
    annotation (Line(points={{20,-60},{0,-60}}, color={0,127,255}));
  connect(pipRet.port_b, senTDisRetDir.port_b)
    annotation (Line(points={{-20,-60},{-70,-60}}, color={0,127,255}));
  connect(senMasFlo.port_b, pipSup.port_a)
    annotation (Line(points={{-40,60},{-20,60}}, color={0,127,255}));
  connect(val.port_3, pipByp.port_a)
    annotation (Line(points={{30,-50},{30,-10}},          color={0,127,255}));
  connect(pipByp.port_b, jun.port_3)
    annotation (Line(points={{30,10},{30,50}}, color={0,127,255}));
  connect(pipSup.port_b, jun.port_1)
    annotation (Line(points={{0,60},{20,60}}, color={0,127,255}));
  connect(jun.port_2, senTDisRetInd.port_a)
    annotation (Line(points={{40,60},{70,60}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
        Text(
          extent={{-52,40},{54,-40}},
          lineColor={0,0,0},
          fillColor={35,138,255},
          fillPattern=FillPattern.Solid,
          textStyle={TextStyle.Bold},
          textString="ETS")}), Diagram(coordinateSystem(preserveAspectRatio=
            false)),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>
Direct cooling energy transfer station (ETS) model with in-building pumping and deltaT control.
The design is based on a typical district cooling ETS described in ASHRAE's 
<a href=\"https://www.ashrae.org/technical-resources/bookstore/district-heating-and-cooling-guides\">
District Cooling Guide</a>.  
As shown in the figure below, the district and building piping are hydronically coupled. The valve
on the district return controls the return temperature to the district cooling network.
</p>
<p align=\"center\">
<img src=\"modelica://Buildings/Resources/Images/Applications/DHC/EnergyTransferStations/CoolingDirectControl.PNG\"/>
</p>
<h4>Reference</h4>
<p>
American Society of Heating, Refrigeration and Air-Conditioning Engineers. (2013). Chapter 5: End User Interface. In <i>District Cooling Guide</i>. 1st Edition. 
</p>
</html>", revisions="<html>
<ul>
<li>November 13, 2019, by Kathryn Hinkelman:<br>First implementation. </li>
</ul>
</html>"));
end CoolingDirectControl;
