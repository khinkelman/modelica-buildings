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
  parameter Modelica.SIunits.PressureDifference dpSte_nominal=pSat-101325
    "Pressure drop at nominal mass flow rate";
  parameter Modelica.SIunits.PressureDifference dpVal_nominal=6000
    "Pressure drop at nominal mass flow rate";
  parameter Buildings.Fluid.Movers.Data.Generic per(
   pressure(V_flow={0,m_flow_nominal,2*m_flow_nominal}/1000,
                   dp=(dpSte_nominal+dpVal_nominal)*{2,1,0}))
    "Performance data for primary pumps";

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

  // Controller
  parameter Modelica.Blocks.Types.SimpleController controllerType=
         Modelica.Blocks.Types.SimpleController.PI "Type of controller";
  parameter Real k(min=0) = 1 "Gain of controller";
  parameter Modelica.SIunits.Time Ti(min=Modelica.Constants.small)=0.5
    "Time constant of Integrator block" annotation (Dialog(enable=
          controllerType == Modelica.Blocks.Types.SimpleController.PI or
          controllerType == Modelica.Blocks.Types.SimpleController.PID));
  parameter Modelica.SIunits.Time Td(min=0)=0.1
    "Time constant of Derivative block" annotation (Dialog(enable=
          controllerType == Modelica.Blocks.Types.SimpleController.PD or
          controllerType == Modelica.Blocks.Types.SimpleController.PID));

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
  Movers.SpeedControlled_y pum(redeclare package Medium = Medium_a,
    energyDynamics=energyDynamics,
    massDynamics=massDynamics,
    per=per,
    addPowerToMedium=false,
    tau=tau)                         "Pump"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Modelica.Blocks.Math.Product pro "Product"
    annotation (Placement(transformation(extent={{60,40},{80,60}})));
  Modelica.Blocks.Interfaces.RealInput y(min=0, max=1) "Part load ratio"
    annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
  MixingVolumes.MixingVolume volWat(
    redeclare package Medium = Medium_a,
    energyDynamics=energyDynamics,
    massDynamics=massDynamics,
      m_flow_nominal=m_flow_nominal,
    V=m_flow_nominal*tau/rho_a_default,
    nPorts=2) "Water volume"
    annotation (Placement(transformation(extent={{-10,0},{10,20}})));
  Controls.Continuous.LimPID con(
    controllerType=controllerType,
    k=k,
    Ti=Ti,
    Td=Td)                       "Controller"
    annotation (Placement(transformation(extent={{-30,40},{-10,60}})));
  FixedResistances.CheckValve cheVal(
    redeclare package Medium = Medium_a,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=dpVal_nominal,
    dpFixed_nominal=0)
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
protected
  Sensors.Pressure senPre(redeclare package Medium = Medium_a)
    "Measured absolute pressure of inflowing fluid"
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  Modelica.Blocks.Sources.RealExpression pSteSet(y=pSat)
    "Pressure setpoint for steam"
    annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
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
  parameter Medium_a.ThermodynamicState sta_a_default=Medium_a.setState_pTX(
      T=Medium_a.T_default, p=Medium_a.p_default, X=Medium_a.X_default);
  parameter Modelica.SIunits.Density rho_a_default=Medium_a.density(sta_a_default)
    "Density, used to compute fluid volume";
  parameter Medium_b.ThermodynamicState sta_b_default=Medium_b.setState_pTX(
      T=Medium_b.T_default, p=Medium_b.p_default, X=Medium_b.X_default);
  parameter Modelica.SIunits.Density rho_b_default=Medium_b.density(sta_b_default)
    "Density, used to compute fluid volume";
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
  connect(senMasFlo.port_b, port_b)
    annotation (Line(points={{80,0},{100,0},{100,0}}, color={0,127,255}));
  connect(pro.u1, eva.dh)
    annotation (Line(points={{58,56},{44,56},{44,6},{41,6}}, color={0,0,127}));
  connect(senMasFlo.m_flow, pro.u2) annotation (Line(points={{70,11},{70,20},{52,
          20},{52,44},{58,44}}, color={0,0,127}));
  connect(pro.y, QOut_flow)
    annotation (Line(points={{81,50},{110,50}}, color={0,0,127}));
  connect(boiEff.y, etaOut)
    annotation (Line(points={{81,90},{110,90}}, color={0,0,127}));
  connect(HeaFloFue.y, QFueOut_flow)
    annotation (Line(points={{81,70},{110,70}}, color={0,0,127}));
  connect(port_a, pum.port_a)
    annotation (Line(points={{-100,0},{-80,0}}, color={0,127,255}));
  connect(senPre.port, pum.port_b)
    annotation (Line(points={{-50,10},{-50,0},{-60,0}}, color={0,127,255}));
  connect(senPre.p, con.u_m)
    annotation (Line(points={{-39,20},{-20,20},{-20,38}}, color={0,0,127}));
  connect(con.u_s, pSteSet.y)
    annotation (Line(points={{-32,50},{-39,50}}, color={0,0,127}));
  connect(con.y, pum.y) annotation (Line(points={{-9,50},{0,50},{0,70},{-70,70},
          {-70,12}}, color={0,0,127}));
  connect(pum.port_b, cheVal.port_a)
    annotation (Line(points={{-60,0},{-40,0}}, color={0,127,255}));
  connect(cheVal.port_b, volWat.ports[1])
    annotation (Line(points={{-20,0},{-2,0}}, color={0,127,255}));
  connect(volWat.ports[2], eva.port_a)
    annotation (Line(points={{2,0},{20,0}}, color={0,127,255}));
  connect(eva.port_b, senMasFlo.port_a)
    annotation (Line(points={{40,0},{60,0}}, color={0,127,255}));
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
