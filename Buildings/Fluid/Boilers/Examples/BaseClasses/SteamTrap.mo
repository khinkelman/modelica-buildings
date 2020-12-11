within Buildings.Fluid.Boilers.Examples.BaseClasses;
model SteamTrap
  "Steam trap with isenthalpic expansion from high to low pressure and 
  condensation of flashed steam"
  extends Buildings.Fluid.Interfaces.PartialTwoPortInterface;
  parameter Modelica.SIunits.AbsolutePressure pLow
     "Low pressure discharge state";
  Medium.SpecificEnthalpy hFlash;
equation
  // Set outlet pressure to setpoint
  port_b.p = pLow;

  // Isenthalpic process assumed: no change in enthalpy
  hFlash = inStream(port_a.h_outflow);

  // Flashed steam condenses

  // Reverse flow

  // Steady state conservation of mass
  port_a.m_flow + port_b.m_flow = 0;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,-40},{40,40}},
          lineColor={0,0,0},
          lineThickness=1),
        Ellipse(
          extent={{-40,40},{40,-40}},
          lineColor={0,0,0},
          lineThickness=1,
          startAngle=-45,
          endAngle=135,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}),
                                 Diagram(coordinateSystem(preserveAspectRatio=false)));
end SteamTrap;
