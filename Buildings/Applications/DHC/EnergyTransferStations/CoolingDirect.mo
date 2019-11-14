within Buildings.Applications.DHC.EnergyTransferStations;
model CoolingDirect "Direct cooling ETS model for district energy systems without in-building pumping or deltaT control"
  extends
    Buildings.Applications.DHC.EnergyTransferStations.BaseClasses.PartialCooling(
      indirectCooling=true);

 parameter Modelica.SIunits.SpecificHeatCapacity cp=
   Medium.specificHeatCapacityCp(
      Medium.setState_pTX(Medium.p_default, Medium.T_default, Medium.X_default))
    "Default specific heat capacity of medium";

  // mass flow rates
  parameter Modelica.SIunits.MassFlowRate m1_flow_nominal(min=0,start=0.5)
    "Nominal mass flow rate";

  // pressure drops
  parameter Modelica.SIunits.PressureDifference dpSup=50
  "Pressure drop in the ETS supply side (piping, valves, etc.)";

  parameter Modelica.SIunits.PressureDifference dpRet=50
  "Pressure drop in the ETS return side (piping, valves, etc.)";

  Buildings.Fluid.FixedResistances.PressureDrop pipSup(
    redeclare package Medium = Medium,
    m_flow_nominal=m1_flow_nominal,
    dp_nominal=dpSup)
                   "Supply pipe" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={10,60})));
  Buildings.Fluid.FixedResistances.PressureDrop pipRet(
    redeclare package Medium = Medium,
    m_flow_nominal=m1_flow_nominal,
    dp_nominal=dpRet)
                   "Return pipe" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={10,-60})));
equation
  connect(senMasFlo.port_b, pipSup.port_a)
    annotation (Line(points={{-40,60},{0,60}}, color={0,127,255}));
  connect(pipSup.port_b, senTDisRetInd.port_a)
    annotation (Line(points={{20,60},{70,60}}, color={0,127,255}));
  connect(senTDisRetDir.port_b, pipRet.port_b)
    annotation (Line(points={{-70,-60},{0,-60}}, color={0,127,255}));
  connect(pipRet.port_a, port_a2) annotation (Line(points={{20,-60},{60,-60},{60,
          -60},{100,-60}}, color={0,127,255}));
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
Direct cooling energy transfer station (ETS) model without in-building pumping or deltaT control.
The design is based on a typical district cooling ETS described in ASHRAE's 
<a href=\"https://www.ashrae.org/technical-resources/bookstore/district-heating-and-cooling-guides\">
District Cooling Guide</a>.  
As shown in the figure below, the district and building piping are hydronically coupled. This direct 
ETS connections relies on individual thermostatic control valves at each individual in-building 
terminal unit for control. 
</p>
<p align=\"center\">
<img src=\"modelica://Buildings/Resources/Images/Applications/DHC/EnergyTransferStations/CoolingDirect.PNG\"/>
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
end CoolingDirect;
