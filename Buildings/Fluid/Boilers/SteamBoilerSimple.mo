within Buildings.Fluid.Boilers;
model SteamBoilerSimple "Simple steam boiler based on EnergyPlus"
  extends Buildings.Fluid.Interfaces.PartialTwoPortTwoMedium;
  parameter Modelica.SIunits.Temperature TSat
     "Saturation temperature";
  parameter Modelica.SIunits.AbsolutePressure pSat
     "Saturation pressure";
  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal
    "Nominal heat flow rate of boiler"
    annotation(Dialog(group = "Nominal condition"));
  parameter Buildings.Fluid.Types.EfficiencyCurves effCur=Buildings.Fluid.Types.EfficiencyCurves.Polynomial
    "Curve used to compute the efficiency";
  parameter Real a[:] = {0.9} "Coefficients for efficiency curve";

  parameter Buildings.Fluid.Data.Fuels.Generic fue "Fuel type"
   annotation (choicesAllMatching = true);

  // Dynamics
  parameter Modelica.SIunits.Time tau = 30
    "Time constant at nominal flow (if energyDynamics <> SteadyState)"
     annotation (Dialog(tab = "Dynamics", group="Nominal condition"));
  parameter Modelica.Fluid.Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
    "Type of energy balance: dynamic (3 initialization options) or steady state"
    annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));
  parameter Modelica.Fluid.Types.Dynamics massDynamics=energyDynamics
    "Type of mass balance: dynamic (3 initialization options) or steady state"
    annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));

  // Diagnostics
   parameter Boolean show_T = false
    "= true, if actual temperature at port is computed"
    annotation(Dialog(tab="Advanced",group="Diagnostics"));

  Modelica.SIunits.Efficiency eta=
    if effCur ==Buildings.Fluid.Types.EfficiencyCurves.Constant then
      a[1]
    elseif effCur ==Buildings.Fluid.Types.EfficiencyCurves.Polynomial then
      Buildings.Utilities.Math.Functions.polynomial(a=a, x=y)
   elseif effCur ==Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear then
      Buildings.Utilities.Math.Functions.quadraticLinear(a=aQuaLin, x1=y, x2=TSat)
   else
      0
  "Boiler efficiency";
  Modelica.SIunits.HeatFlowRate QFue_flow = y * Q_flow_nominal/eta_nominal
    "Heat released by fuel";
  Modelica.SIunits.HeatFlowRate QWatTot_flow = eta * QFue_flow
    "Heat transfer from gas into water";
  Modelica.SIunits.MassFlowRate mFue_flow = QFue_flow/fue.h
    "Fuel mass flow rate";
  Modelica.SIunits.VolumeFlowRate VFue_flow = mFue_flow/fue.d
    "Fuel volume flow rate";

  Modelica.Blocks.Interfaces.RealOutput QOut_flow(
    final quantity="HeatFlowRate",
    final unit="W",
    displayUnit="kW") "Heat transfer rate of boiler" annotation (Placement(
        transformation(extent={{100,40},{120,60}}), iconTransformation(extent={{
            100,40},{120,60}})));
  Modelica.Blocks.Interfaces.RealOutput QFueOut_flow(
    final quantity="HeatFlowRate",
    final unit="W",
    displayUnit="kW") "Heat transfer rate of fuel" annotation (Placement(
        transformation(extent={{100,60},{120,80}}), iconTransformation(extent={{
            100,80},{120,100}})));
  Modelica.Blocks.Interfaces.RealOutput etaOut "Efficiency of boiler"
    annotation (Placement(transformation(extent={{100,80},{120,100}}),
        iconTransformation(extent={{100,80},{120,100}})));
  BaseClasses.Evaporation eva(
    redeclare package Medium_a = Medium_a,
    redeclare package Medium_b = Medium_b,
    m_flow_nominal=m_flow_nominal,
    TSat=TSat,
    pSat=pSat)                     "Evaporation"
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Movers.FlowControlled_dp pum(redeclare package Medium = Medium_a,
    energyDynamics=energyDynamics,
    massDynamics=massDynamics,
      m_flow_nominal=m_flow_nominal,
    tau=tau)                         "Pump"
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
  Modelica.Blocks.Math.Product pro "Product"
    annotation (Placement(transformation(extent={{60,40},{80,60}})));
  Modelica.Blocks.Interfaces.RealInput y(min=0, max=1) "Part load ratio"
    annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
  Sources.Boundary_pT exp(
    redeclare package Medium = Medium_a,
    p(displayUnit="Pa") = pSat,
    nPorts=1) "Expansion tank"
    annotation (Placement(transformation(extent={{-18,-50},{2,-30}})));
protected
  Sensors.Pressure senPre(redeclare package Medium = Medium_a)
    "Measured absolute pressure of inflowing fluid"
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
  Modelica.Blocks.Sources.RealExpression pSteSet(y=pSat)
    "Pressure setpoint for steam"
    annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
  Modelica.Blocks.Math.Add dpSen(k2=-1)
    "Change in pressure needed to meet setpoint"
    annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
  Modelica.Blocks.Sources.RealExpression boiEff(y=eta) "Boiler efficiency"
    annotation (Placement(transformation(extent={{60,80},{80,100}})));
  Sensors.MassFlowRate senMasFlo(redeclare package Medium = Medium_b)
    "Measured mass flow rate"
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  Modelica.Blocks.Sources.RealExpression HeaFloFue(y=QFue_flow)
    "Heat flow rate of fuel"
    annotation (Placement(transformation(extent={{60,60},{80,80}})));


// Boiler
protected
  parameter Real eta_nominal(fixed=false, start=0.9) "Boiler efficiency at nominal condition";
  parameter Real aQuaLin[6] = if size(a, 1) == 6 then a else fill(0, 6)
  "Auxiliary variable for efficiency curve because quadraticLinear requires exactly 6 elements";

initial equation
  // Boiler efficiency
  if  effCur == Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear then
    assert(size(a, 1) == 6,
    "The boiler has the efficiency curve set to 'Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear',
    and hence the parameter 'a' must have exactly 6 elements.
    However, only " + String(size(a, 1)) + " elements were provided.");
  end if;

  if effCur ==Buildings.Fluid.Types.EfficiencyCurves.Constant then
    eta_nominal = a[1];
  elseif effCur ==Buildings.Fluid.Types.EfficiencyCurves.Polynomial then
    eta_nominal = Buildings.Utilities.Math.Functions.polynomial(a=a, x=1);
  elseif effCur ==Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear then
    // For this efficiency curve, a must have 6 elements.
    eta_nominal = Buildings.Utilities.Math.Functions.quadraticLinear(a=aQuaLin, x1=1, x2=TSat);
  else
     eta_nominal = 999;
  end if;
equation
  assert(eta > 0.001, "Efficiency curve is wrong.");
  connect(port_a, pum.port_a)
    annotation (Line(points={{-100,0},{-20,0}}, color={0,127,255}));
  connect(pum.port_b, eva.port_a)
    annotation (Line(points={{0,0},{20,0}}, color={0,127,255}));
  connect(senPre.port, port_a)
    annotation (Line(points={{-70,20},{-70,0},{-100,0}}, color={0,127,255}));
  connect(pSteSet.y, dpSen.u1) annotation (Line(points={{-59,50},{-50,50},{-50,56},
          {-42,56}}, color={0,0,127}));
  connect(dpSen.u2, senPre.p) annotation (Line(points={{-42,44},{-50,44},{-50,30},
          {-59,30}}, color={0,0,127}));
  connect(dpSen.y, pum.dp_in)
    annotation (Line(points={{-19,50},{-10,50},{-10,12}}, color={0,0,127}));
  connect(eva.port_b, senMasFlo.port_a)
    annotation (Line(points={{40,0},{60,0}}, color={0,127,255}));
  connect(senMasFlo.port_b, port_b)
    annotation (Line(points={{80,0},{100,0},{100,0}}, color={0,127,255}));
  connect(pro.u1, eva.dh)
    annotation (Line(points={{58,56},{44,56},{44,6},{41,6}}, color={0,0,127}));
  connect(senMasFlo.m_flow, pro.u2) annotation (Line(points={{70,11},{70,20},{48,
          20},{48,44},{58,44}}, color={0,0,127}));
  connect(pro.y, QOut_flow)
    annotation (Line(points={{81,50},{110,50}}, color={0,0,127}));
  connect(boiEff.y, etaOut)
    annotation (Line(points={{81,90},{110,90}}, color={0,0,127}));
  connect(HeaFloFue.y, QFueOut_flow)
    annotation (Line(points={{81,70},{110,70}}, color={0,0,127}));
  connect(exp.ports[1], eva.port_a) annotation (Line(points={{2,-40},{10,-40},{10,
          0},{20,0}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Text(
          extent={{-149,-124},{151,-164}},
          lineColor={0,0,255},
          textString="%name"),
        Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,40},{40,-40}},
          fillColor={127,0,0},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
      Line(
        points={{20,18},{0,8},{20,-12},{0,-22}},
        color={238,46,47},
        smooth=Smooth.Bezier,
          extent={{-60,-22},{-36,2}}),
      Line(
        points={{-2,18},{-22,8},{-2,-12},{-22,-22}},
        color={238,46,47},
        smooth=Smooth.Bezier,
          extent={{-60,-22},{-36,2}})}), Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end SteamBoilerSimple;
