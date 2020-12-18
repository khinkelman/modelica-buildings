within Buildings.Fluid.Boilers.Examples.BaseClasses;
model SteamCoil "Steam coil based on EnergyPlus"
  extends Buildings.Fluid.Interfaces.PartialTwoPortTwoMedium;
  parameter Modelica.SIunits.Temperature TSatHig
     "Saturation temperature at high pressure";
  parameter Modelica.SIunits.AbsolutePressure pSat
     "Saturation pressure";
  parameter Modelica.SIunits.Temperature TSatLow=373.15
    "Saturation temperature at low pressure";
  parameter Modelica.SIunits.TemperatureDifference TSubCoo=5
    "Degree of subcooling at the heating coil";
  parameter Modelica.SIunits.TemperatureDifference TLooSubCoo=15
    "Degree of subcooling at loop";
  parameter Modelica.SIunits.PressureDifference dpCoi_nominal=6000
    "Nominal pressure drop in coil";
  parameter Modelica.SIunits.PressureDifference dpSte_nominal = pSat - 101325
    "Nominal pressure difference in steam network";

  // Dynamics
  parameter Modelica.SIunits.Time tau = 30
    "Time constant at nominal flow (if energyDynamics <> SteadyState)"
     annotation (Dialog(tab = "Dynamics", group="Nominal condition"));
  parameter Modelica.Fluid.Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState
    "Type of energy balance: dynamic (3 initialization options) or steady state"
    annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));
  parameter Modelica.Fluid.Types.Dynamics massDynamics=energyDynamics
    "Type of mass balance: dynamic (3 initialization options) or steady state"
    annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));

  Condensation con(
    redeclare package Medium_a = Medium_a,
    redeclare package Medium_b = Medium_b,
    m_flow_nominal=m_flow_nominal,
    show_T=show_T,
    TSat=TSatHig,
    pSat=pSat) "Condensation process"
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  HeatExchangers.SensibleCooler_T subCoo(
    redeclare package Medium = Medium_b,
    m_flow_nominal=m_flow_nominal,
    show_T=show_T,
    dp_nominal=dpCoi_nominal/2,
    tau=tau,
    T_start=TSatLow - TSubCoo,
    energyDynamics=energyDynamics) "Subcool the condensate"
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  Modelica.Blocks.Interfaces.RealOutput QOut_flow(
    final quantity="HeatFlowRate",
    final unit="W",
    displayUnit="kW") "Heat transfer rate of coil into working fluid"
                                                     annotation (Placement(
        transformation(extent={{100,80},{120,100}}),iconTransformation(extent={{100,80},
            {120,100}})));
  Modelica.Blocks.Math.Product pro "Product"
    annotation (Placement(transformation(extent={{-10,60},{10,80}})));
  Modelica.Blocks.Math.Add addOut "Total useful heat flow out"
    annotation (Placement(transformation(extent={{70,80},{90,100}})));
  Modelica.Blocks.Math.Add TOutHex(
    k1=-1,
    y(unit = "K"))
                 "Heat exchanger outlet temperature"
    annotation (Placement(transformation(extent={{30,20},{50,40}})));
  SteamTrap steTra(
    redeclare package Medium = Medium_b,
    m_flow_nominal=m_flow_nominal,
    show_T=show_T,
    dpSet=dpSte_nominal,
    TSatLow=TSatLow)
                   "Steam trap"
    annotation (Placement(transformation(extent={{-80,-90},{-60,-70}})));
  HeatExchangers.SensibleCooler_T subCooLoo(
    redeclare package Medium = Medium_b,
    m_flow_nominal=m_flow_nominal,
    show_T=show_T,
    dp_nominal=dpCoi_nominal/2,
    tau=tau,
    T_start=TSatLow - TSubCoo - TLooSubCoo,
    energyDynamics=energyDynamics) "Loop Subcooling"
    annotation (Placement(transformation(extent={{30,-90},{50,-70}})));
  Modelica.Blocks.Math.Add TOutCoi(
    k1=-1,
    y(unit = "K")) "Coil outlet temperature"
    annotation (Placement(transformation(extent={{30,-60},{50,-40}})));
  Modelica.Blocks.Interfaces.RealOutput QLos_flow(
    final quantity="HeatFlowRate",
    final unit="W",
    displayUnit="kW") "Transfer rate of lost heat" annotation (Placement(
        transformation(extent={{100,30},{120,50}}), iconTransformation(extent={
            {100,50},{120,70}})));
  Modelica.Blocks.Math.Add addLos "Total losses in heat flow"
    annotation (Placement(transformation(extent={{70,30},{90,50}})));
protected
  Modelica.Blocks.Sources.RealExpression TSubCooSet(y=TSubCoo)
    "Subcooling temperature setpoint"
    annotation (Placement(transformation(extent={{-10,26},{10,46}})));
  Sensors.MassFlowRate senMasFlo(redeclare package Medium = Medium_a)
    "Measured mass flow rate"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
public
  Sensors.TemperatureTwoPort temSen(redeclare package Medium = Medium_b,
      m_flow_nominal=m_flow_nominal,
    T_start=TSatLow)                 "Temperature sensor"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
protected
  Modelica.Blocks.Sources.RealExpression TLooSubCooSet(y=TLooSubCoo)
    "Loop subcooling temperature setpoint"
    annotation (Placement(transformation(extent={{-10,-54},{10,-34}})));
public
  Sensors.TemperatureTwoPort temSen1(redeclare package Medium = Medium_b,
      m_flow_nominal=m_flow_nominal,
    T_start=TSatLow - TSubCoo)       "Temperature sensor"
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}})));
  MixingVolumes.MixingVolume volWat(
    redeclare package Medium = Medium_b,
    energyDynamics=energyDynamics,
    massDynamics=massDynamics,
    m_flow_nominal=m_flow_nominal,
    V=m_flow_nominal*tau/rho_b_default,
    nPorts=2) "Condensate water volume"
    annotation (Placement(transformation(extent={{-40,-80},{-20,-60}})));
protected
  parameter Medium_b.ThermodynamicState sta_b_default=Medium_b.setState_pTX(
      T=Medium_b.T_default, p=Medium_b.p_default, X=Medium_b.X_default);
  parameter Modelica.SIunits.Density rho_b_default=Medium_b.density(sta_b_default)
    "Density, used to compute fluid volume";
equation
  connect(port_a, senMasFlo.port_a)
    annotation (Line(points={{-100,0},{-80,0}}, color={0,127,255}));
  connect(senMasFlo.port_b, con.port_a)
    annotation (Line(points={{-60,0},{-50,0}}, color={0,127,255}));
  connect(senMasFlo.m_flow, pro.u1)
    annotation (Line(points={{-70,11},{-70,76},{-12,76}},color={0,0,127}));
  connect(con.dh, pro.u2) annotation (Line(points={{-29,6},{-20,6},{-20,64},{
          -12,64}},
                color={0,0,127}));
  connect(pro.y, addOut.u1) annotation (Line(points={{11,70},{40,70},{40,96},{
          68,96}}, color={0,0,127}));
  connect(addOut.u2, subCoo.Q_flow)
    annotation (Line(points={{68,84},{60,84},{60,8},{51,8}}, color={0,0,127}));
  connect(addOut.y, QOut_flow)
    annotation (Line(points={{91,90},{110,90}}, color={0,0,127}));
  connect(TSubCooSet.y, TOutHex.u1)
    annotation (Line(points={{11,36},{28,36}}, color={0,0,127}));
  connect(temSen.T, TOutHex.u2)
    annotation (Line(points={{0,11},{0,24},{28,24}}, color={0,0,127}));
  connect(con.port_b, temSen.port_a)
    annotation (Line(points={{-30,0},{-10,0}}, color={0,127,255}));
  connect(temSen.port_b, subCoo.port_a)
    annotation (Line(points={{10,0},{30,0}}, color={0,127,255}));
  connect(TOutHex.y, subCoo.TSet) annotation (Line(points={{51,30},{56,30},{56,14},
          {20,14},{20,8},{28,8}}, color={0,0,127}));
  connect(subCoo.port_b, steTra.port_a) annotation (Line(points={{50,0},{50,-20},
          {-84,-20},{-84,-80},{-80,-80}},         color={0,127,255}));
  connect(temSen1.port_b, subCooLoo.port_a)
    annotation (Line(points={{10,-80},{30,-80}}, color={0,127,255}));
  connect(subCooLoo.port_b, port_b) annotation (Line(points={{50,-80},{80,-80},{
          80,0},{100,0}}, color={0,127,255}));
  connect(TLooSubCooSet.y, TOutCoi.u1)
    annotation (Line(points={{11,-44},{28,-44}}, color={0,0,127}));
  connect(temSen1.T, TOutCoi.u2)
    annotation (Line(points={{0,-69},{0,-56},{28,-56}}, color={0,0,127}));
  connect(TOutCoi.y, subCooLoo.TSet) annotation (Line(points={{51,-50},{56,-50},
          {56,-64},{20,-64},{20,-72},{28,-72}}, color={0,0,127}));
  connect(QLos_flow, QLos_flow)
    annotation (Line(points={{110,40},{110,40}}, color={0,0,127}));
  connect(addLos.u1, steTra.QLos_flow) annotation (Line(points={{68,46},{62,46},
          {62,-30},{-56,-30},{-56,-73},{-59,-73}}, color={0,0,127}));
  connect(subCooLoo.Q_flow, addLos.u2) annotation (Line(points={{51,-72},{64,
          -72},{64,34},{68,34}}, color={0,0,127}));
  connect(addLos.y, QLos_flow) annotation (Line(points={{91,40},{96,40},{96,40},
          {110,40}}, color={0,0,127}));
  connect(steTra.port_b, volWat.ports[1])
    annotation (Line(points={{-60,-80},{-32,-80}}, color={0,127,255}));
  connect(volWat.ports[2], temSen1.port_a)
    annotation (Line(points={{-28,-80},{-10,-80}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid), Line(
          points={{-100,0},{-60,0},{-50,0},{-50,40},{-10,-40},{-10,40},{30,-40},
              {30,40},{50,0},{100,0}},
          color={0,0,0},
          thickness=0.5),
        Text(
          extent={{-147,-116},{153,-156}},
          lineColor={0,0,255},
          textString="%name")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SteamCoil;
