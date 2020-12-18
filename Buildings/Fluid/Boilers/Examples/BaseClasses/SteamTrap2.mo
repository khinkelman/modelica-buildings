within Buildings.Fluid.Boilers.Examples.BaseClasses;
model SteamTrap2 "Steam trap with isenthalpic expansion from high to atmospheric pressure and 
  condensation of flashed steam"
  extends Buildings.Fluid.Interfaces.PartialTwoPortInterface;
//  parameter Modelica.SIunits.AbsolutePressure pSte
//    "Steam pressure at trap inlet";
  final parameter Modelica.SIunits.AbsolutePressure pAtm=101325
     "Atmospheric pressure discharge state";
  final parameter Modelica.SIunits.Temperature TSat=100+273.15
    "Saturation temperature at atmospheric pressure";
//  final parameter Medium.SpecificEnthalpy hl=419100
//    "Enthalpy of saturated liquid at atmospheric pressure";
  Medium.SpecificEnthalpy dh
    "Change in enthalpy";
  Modelica.Blocks.Interfaces.RealOutput QLos_flow(unit="W") "Heat transfer loss rate"
    annotation (Placement(transformation(extent={{100,60},{120,80}})));
equation
  // Pressure setpoints
  port_b.p = pAtm;

  // Isenthalpic process assumed: no change in enthalpy
  port_a.h_outflow = inStream(port_a.h_outflow);

  // Flashed steam condenses
  port_b.h_outflow = Medium.specificEnthalpy(
    state=Medium.setState_pTX(
      p=pAtm,T=TSat,X=inStream(port_a.Xi_outflow)));
  dh = port_b.h_outflow - inStream(port_a.h_outflow);

  // Reverse flow
//  inStream(port_b.h_outflow) = port_b.h_outflow;
//  port_a.h_outflow = hl + dh;

  // Steady state conservation of mass
  port_a.m_flow + port_b.m_flow = 0;

  // Conservation of energy
  port_a.m_flow*dh = QLos_flow;

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
end SteamTrap2;
