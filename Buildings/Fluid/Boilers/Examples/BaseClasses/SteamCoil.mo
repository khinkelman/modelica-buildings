within Buildings.Fluid.Boilers.Examples.BaseClasses;
model SteamCoil "Steam coil based on EnergyPlus"
  extends Buildings.Fluid.Interfaces.PartialTwoPortTwoMedium;
  parameter Modelica.SIunits.Temperature TSat
     "Saturation temperature";
  parameter Modelica.SIunits.AbsolutePressure pSat
     "Saturation pressure";
  parameter Modelica.SIunits.Temperature TSubCoo=278.15
    "Degree of subcooling at the heating coil";
  parameter Modelica.SIunits.Temperature TLooSubCoo=273.15+25
    "Degree of subcooling at loop";

  // Dynamics
  parameter Modelica.SIunits.Time tau = 30
    "Time constant at nominal flow (if energyDynamics <> SteadyState)"
     annotation (Dialog(tab = "Dynamics", group="Nominal condition"));
  parameter Modelica.Fluid.Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState
    "Type of energy balance: dynamic (3 initialization options) or steady state"
    annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));

  Condensation con(
    redeclare package Medium_a = Medium_a,
    redeclare package Medium_b = Medium_b,
    m_flow_nominal=m_flow_nominal,
    TSat=TSat,
    pSat=pSat) "Condensation process"
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  HeatExchangers.SensibleCooler_T subCoo(
    redeclare package Medium = Medium_b,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=6000,
    tau=tau,
    energyDynamics=energyDynamics) "Subcool the condensate"
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  Modelica.Blocks.Interfaces.RealOutput QOut_flow(
    final quantity="HeatFlowRate",
    final unit="W",
    displayUnit="kW") "Heat transfer rate of boiler" annotation (Placement(
        transformation(extent={{100,60},{120,80}}), iconTransformation(extent={{100,60},
            {120,80}})));
  Modelica.Blocks.Math.Product pro "Product"
    annotation (Placement(transformation(extent={{-10,66},{10,86}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{70,60},{90,80}})));
  Modelica.Blocks.Math.Add TOutHex(k1=-1) "Heat exchanger outlet temperature"
    annotation (Placement(transformation(extent={{30,20},{50,40}})));
  SteamTrap steTra(
    redeclare package Medium = Medium_b,
    m_flow_nominal=m_flow_nominal,
    pLow=Medium_b.p_default)
                   "Steam trap"
    annotation (Placement(transformation(extent={{-50,-90},{-30,-70}})));
  HeatExchangers.SensibleCooler_T subCooLoo(
    redeclare package Medium = Medium_b,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=6000,
    tau=tau,
    energyDynamics=energyDynamics) "Loop Subcooling"
    annotation (Placement(transformation(extent={{30,-90},{50,-70}})));
  Modelica.Blocks.Math.Add TOutCoi(k1=-1) "Coil outlet temperature"
    annotation (Placement(transformation(extent={{30,-60},{50,-40}})));
protected
  Modelica.Blocks.Sources.RealExpression TSubCooSet(y=TSubCoo)
    "Subcooling temperature setpoint"
    annotation (Placement(transformation(extent={{-10,26},{10,46}})));
  Sensors.MassFlowRate senMasFlo(redeclare package Medium = Medium_a)
    "Measured mass flow rate"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Sensors.TemperatureTwoPort temSen(redeclare package Medium = Medium_a,
      m_flow_nominal=m_flow_nominal) "Temperature sensor"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
protected
  Modelica.Blocks.Sources.RealExpression TLooSubCooSet(y=TLooSubCoo)
    "Loop subcooling temperature setpoint"
    annotation (Placement(transformation(extent={{-10,-54},{10,-34}})));
  Sensors.TemperatureTwoPort temSen1(redeclare package Medium = Medium_b,
      m_flow_nominal=m_flow_nominal) "Temperature sensor"
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}})));
equation
  connect(port_a, senMasFlo.port_a)
    annotation (Line(points={{-100,0},{-80,0}}, color={0,127,255}));
  connect(senMasFlo.port_b, con.port_a)
    annotation (Line(points={{-60,0},{-50,0}}, color={0,127,255}));
  connect(senMasFlo.m_flow, pro.u1)
    annotation (Line(points={{-70,11},{-70,82},{-12,82}},color={0,0,127}));
  connect(con.dh, pro.u2) annotation (Line(points={{-29,6},{-20,6},{-20,70},{-12,
          70}}, color={0,0,127}));
  connect(pro.y, add.u1)
    annotation (Line(points={{11,76},{68,76}}, color={0,0,127}));
  connect(add.u2, subCoo.Q_flow)
    annotation (Line(points={{68,64},{64,64},{64,8},{51,8}}, color={0,0,127}));
  connect(add.y, QOut_flow)
    annotation (Line(points={{91,70},{110,70}}, color={0,0,127}));
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
  connect(subCoo.port_b, steTra.port_a) annotation (Line(points={{50,0},{60,0},{
          60,-26},{-60,-26},{-60,-80},{-50,-80}}, color={0,127,255}));
  connect(steTra.port_b, temSen1.port_a)
    annotation (Line(points={{-30,-80},{-10,-80}}, color={0,127,255}));
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
