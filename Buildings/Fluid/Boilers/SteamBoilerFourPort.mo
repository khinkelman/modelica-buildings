within Buildings.Fluid.Boilers;
model SteamBoilerFourPort
  "Model for a steam boiler with four ports for air and water flows, including medium changes"
  extends Buildings.Fluid.Interfaces.PartialFourPortFourMedium(
    redeclare final package Medium_a1 = MediumWat,
    redeclare final package Medium_b1 = MediumSte,
    redeclare final package Medium_a2 = MediumAir,
    redeclare final package Medium_b2 = MediumAir);
  extends Buildings.Fluid.Boilers.BaseClasses.PartialSteamBoiler(
    redeclare final package Medium_a = MediumWat,
    redeclare final package Medium_b = MediumSte,
    final m_flow_nominal = m1_flow_nominal,
    vol(
      m_flow_small=m1_flow_small,
      V=m1_flow_nominal*tau/rho_default),
    eva(final m_flow_nominal=m1_flow_nominal, m_flow_small=m1_flow_small),
    dpCon(
      m_flow_nominal=m1_flow_nominal,
      m_flow_small=m1_flow_small,
      addPowerToMedium=false,
      nominalValuesDefineDefaultPressureCurve=true));

  replaceable package MediumWat =
      Modelica.Media.Interfaces.PartialMedium
    "Medium model for liquid water";
  replaceable package MediumSte =
      Modelica.Media.Interfaces.PartialMedium
    "Medium model for steam vapor";
  replaceable package MediumAir =
      Modelica.Media.Interfaces.PartialMedium
    "Medium model for air";

//  parameter Real ratAirFue = 10
//    "Air-to-fuel ratio (by volume)";

  BaseClasses.Combustion com(
    m_flow_nominal=m2_flow_nominal,
    redeclare final package Medium = MediumAir,
    final show_T = show_T) "Combustion process"
    annotation (Placement(transformation(extent={{-40,-70},{-60,-50}})));
  FixedResistances.PressureDrop preDroAir(
    redeclare package Medium = MediumAir,
    m_flow_nominal=m2_flow_nominal,
    dp_nominal=1400,
    final show_T = show_T) "Air side total pressure drop"
    annotation (Placement(transformation(extent={{40,-70},{20,-50}})));
  Sensors.VolumeFlowRate senVolFloAir(redeclare package Medium = MediumAir,
      m_flow_nominal=m2_flow_nominal) "Sensor for air volumetric flow rate"
    annotation (Placement(transformation(extent={{0,-50},{-20,-70}})));
  Modelica.Blocks.Interfaces.RealOutput ratAirFue
    "Ratio of air to fuel by volume" annotation (Placement(transformation(
          extent={{100,-100},{120,-80}}), iconTransformation(extent={{100,-100},
            {120,-80}})));
  Modelica.Blocks.Math.Division airToFue "Air to fuel ratio"
    annotation (Placement(transformation(extent={{60,-100},{80,-80}})));
  Modelica.Blocks.Logical.Switch swi "Switch"
    annotation (Placement(transformation(extent={{0,-110},{20,-90}})));
  Modelica.Blocks.Logical.GreaterThreshold graZer "Greater than zero threshold"
    annotation (Placement(transformation(extent={{-40,-110},{-20,-90}})));
  Modelica.Blocks.Sources.Constant negOne(k=-1)
    "Negative number to avoid dividing by zero"
    annotation (Placement(transformation(extent={{-80,-130},{-60,-110}})));
protected
  Modelica.Blocks.Sources.RealExpression volFloFue(y=VFue_flow)
    "Volumetric flow rate of fuel"
    annotation (Placement(transformation(extent={{-80,-110},{-60,-90}})));
equation
  connect(port_a2, port_a2) annotation (Line(points={{100,-60},{100,-60},{100,
          -60}},
        color={0,127,255}));
  connect(y, com.y) annotation (Line(points={{-120,100},{-86,100},{-86,-40},{
          -34,-40},{-34,-52},{-39,-52}},
                                   color={0,0,127}));
  connect(port_a1, senMasFlo.port_a) annotation (Line(points={{-100,60},{-94,60},
          {-94,0},{-80,0}}, color={0,127,255}));
  connect(temSen_out.port_b, port_b1) annotation (Line(points={{90,0},{94,0},{
          94,60},{100,60}}, color={0,127,255}));
  connect(port_a2, preDroAir.port_a) annotation (Line(points={{100,-60},{40,-60}},
                              color={0,127,255}));
  connect(com.port_b, port_b2) annotation (Line(points={{-60,-60},{-100,-60}},
                                color={0,127,255}));
  connect(com.port_a, senVolFloAir.port_b)
    annotation (Line(points={{-40,-60},{-20,-60}}, color={0,127,255}));
  connect(senVolFloAir.port_a, preDroAir.port_b)
    annotation (Line(points={{0,-60},{20,-60}}, color={0,127,255}));
  connect(senVolFloAir.V_flow, airToFue.u1)
    annotation (Line(points={{-10,-71},{-10,-84},{58,-84}}, color={0,0,127}));
  connect(airToFue.y, ratAirFue)
    annotation (Line(points={{81,-90},{110,-90}}, color={0,0,127}));
  connect(volFloFue.y, graZer.u)
    annotation (Line(points={{-59,-100},{-42,-100}}, color={0,0,127}));
  connect(ratAirFue, ratAirFue)
    annotation (Line(points={{110,-90},{110,-90}}, color={0,0,127}));
  connect(graZer.y, swi.u2)
    annotation (Line(points={{-19,-100},{-2,-100}}, color={255,0,255}));
  connect(volFloFue.y, swi.u1) annotation (Line(points={{-59,-100},{-50,-100},{
          -50,-82},{-14,-82},{-14,-92},{-2,-92}}, color={0,0,127}));
  connect(negOne.y, swi.u3) annotation (Line(points={{-59,-120},{-12,-120},{-12,
          -108},{-2,-108}}, color={0,0,127}));
  connect(swi.y, airToFue.u2) annotation (Line(points={{21,-100},{50,-100},{50,
          -96},{58,-96}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,64},{74,56}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{0,56},{100,64}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,40},{40,-40}},
          fillColor={127,0,0},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-100,-56},{100,-64}},
          lineColor={244,125,35},
          pattern=LinePattern.None,
          fillColor={255,170,85},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{0,-56},{100,-64}},
          lineColor={244,125,35},
          pattern=LinePattern.None,
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
      Line(
        points={{-2,20},{-22,10},{-2,-10},{-22,-20}},
        color={238,46,47},
        smooth=Smooth.Bezier,
          extent={{-60,-22},{-36,2}}),
      Line(
        points={{20,20},{0,10},{20,-10},{0,-20}},
        color={238,46,47},
        smooth=Smooth.Bezier,
          extent={{-60,-22},{-36,2}})}),                         Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},{100,
            120}})));
end SteamBoilerFourPort;
