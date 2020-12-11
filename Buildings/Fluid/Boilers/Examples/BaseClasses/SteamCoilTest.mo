within Buildings.Fluid.Boilers.Examples.BaseClasses;
model SteamCoilTest "Steam coil based on EnergyPlus"
  extends Buildings.Fluid.Interfaces.PartialTwoPortInterface;
  parameter Modelica.SIunits.Temperature TSat
     "Saturation temperature";
  parameter Modelica.SIunits.AbsolutePressure pSat
     "Saturation pressure";
  parameter Modelica.SIunits.Temperature TSubCoo=278.15
    "Degree of subcooling at the heating coil";

  // Dynamics
  parameter Modelica.SIunits.Time tau = 30
    "Time constant at nominal flow (if energyDynamics <> SteadyState)"
     annotation (Dialog(tab = "Dynamics", group="Nominal condition"));
  parameter Modelica.Fluid.Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState
    "Type of energy balance: dynamic (3 initialization options) or steady state"
    annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));

  HeatExchangers.SensibleCooler_T subCoo(
    redeclare package Medium = Medium_b,
    m_flow_nominal=m_flow_nominal,
    show_T=show_T,
    dp_nominal=6000,
    tau=tau,
    energyDynamics=energyDynamics) "Subcool the condensate"
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  Modelica.Blocks.Math.Add TOutHex(
    k1=-1,
    y(unit = "degC"))
                 "Heat exchanger outlet temperature"
    annotation (Placement(transformation(extent={{30,20},{50,40}})));
protected
  Modelica.Blocks.Sources.RealExpression TSubCooSet(y=TSubCoo)
    "Subcooling temperature setpoint"
    annotation (Placement(transformation(extent={{-10,26},{10,46}})));
public
  Sensors.TemperatureTwoPort temSen(redeclare package Medium = Medium_b,
      m_flow_nominal=m_flow_nominal,
    T_start=TSat)                    "Temperature sensor"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation
  connect(TSubCooSet.y, TOutHex.u1)
    annotation (Line(points={{11,36},{28,36}}, color={0,0,127}));
  connect(temSen.T, TOutHex.u2)
    annotation (Line(points={{0,11},{0,24},{28,24}}, color={0,0,127}));
  connect(temSen.port_b, subCoo.port_a)
    annotation (Line(points={{10,0},{30,0}}, color={0,127,255}));
  connect(TOutHex.y, subCoo.TSet) annotation (Line(points={{51,30},{56,30},{56,14},
          {20,14},{20,8},{28,8}}, color={0,0,127}));
  connect(port_a, temSen.port_a)
    annotation (Line(points={{-100,0},{-10,0}}, color={0,127,255}));
  connect(subCoo.port_b, port_b)
    annotation (Line(points={{50,0},{100,0}}, color={0,127,255}));
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
end SteamCoilTest;
