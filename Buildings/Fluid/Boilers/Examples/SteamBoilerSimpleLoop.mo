within Buildings.Fluid.Boilers.Examples;
model SteamBoilerSimpleLoop "Loop example based on EnergyPlus"
  extends Modelica.Icons.Example;

  package MediumSte = Buildings.Media.Steam (
     T_default=110+273.15,
     p_default=143380) "Steam medium";
  package MediumWat = Buildings.Media.Water "Water medium";

  parameter Modelica.SIunits.Temperature TSat=273.15+110
     "Saturation temperature";
  parameter Modelica.SIunits.AbsolutePressure pSat=143380
     "Saturation pressure";
  parameter Modelica.SIunits.Temperature TSubCoo=278.15
    "Degree of subcooling at the heating coil";

  parameter Modelica.SIunits.MassFlowRate mBoi_flow_nominal
    "Nominal mass flow rate of boiler";
  parameter Modelica.SIunits.HeatFlowRate QBoi_flow_nominal
    "Nominal heat flow rate of boiler";
  parameter Modelica.SIunits.MassFlowRate mCoi_flow_nominal=mBoi_flow_nominal
    "Nominal mass flow rate of Coil";

  parameter Modelica.SIunits.PressureDifference dp_nominal=10000
    "Pressure drop at nominal mass flow rate in district network";
  SteamBoilerSimple boi(redeclare package Medium_a = MediumWat, redeclare
      package Medium_b = MediumSte,
    m_flow_nominal=mBoi_flow_nominal,
    TSat=TSat,
    pSat=pSat,
    Q_flow_nominal=QBoi_flow_nominal)
                                    "Boiler"
    annotation (Placement(transformation(extent={{10,60},{30,80}})));
  BaseClasses.SteamCoil coi(redeclare package Medium_a = MediumSte, redeclare
      package Medium_b = MediumWat,
    m_flow_nominal=mCoi_flow_nominal,
    TSat=TSat,
    pSat=pSat)                      "Steam coil"
    annotation (Placement(transformation(extent={{30,-20},{10,0}})));
  Movers.FlowControlled_m_flow pum(redeclare package Medium = MediumWat)
    "Condensate pump"
    annotation (Placement(transformation(extent={{-20,-20},{-40,0}})));
public
  FixedResistances.PressureDrop                 preDro(
    redeclare package Medium = MediumWat,
    final m_flow_nominal=mBoi_flow_nominal,
    final homotopyInitialization=homotopyInitialization,
    final dp_nominal=dp_nominal) "Flow resistance"
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
equation
  connect(pum.port_b, preDro.port_a) annotation (Line(points={{-40,-10},{-60,
          -10},{-60,70},{-40,70}},
                              color={0,127,255}));
  connect(preDro.port_b, boi.port_a)
    annotation (Line(points={{-20,70},{10,70}},color={0,127,255}));
  connect(boi.port_b, coi.port_a) annotation (Line(points={{30,70},{50,70},{50,
          -10},{30,-10}}, color={0,127,255}));
  connect(coi.port_b, pum.port_a)
    annotation (Line(points={{10,-10},{-20,-10}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SteamBoilerSimpleLoop;
