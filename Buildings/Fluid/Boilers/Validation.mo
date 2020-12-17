within Buildings.Fluid.Boilers;
package Validation "Validation of the main model"
  extends Modelica.Icons.ExamplesPackage;

  model SteamBoilerSimpleEnergyPlus "Loop example based on EnergyPlus"
    extends Modelica.Icons.Example;

    package MediumSte = Buildings.Media.Steam (
       T_default=110+273.15,
       p_default=143380) "Steam medium";
    package MediumWat = Buildings.Media.Water "Water medium";

    parameter Modelica.SIunits.AbsolutePressure pSat=143380
      "Nominal steam pressure";
    parameter Modelica.SIunits.Temperature TSat=
      MediumSte.saturationTemperature(pSat)
      "Steam temperature";
    parameter Modelica.SIunits.SpecificEnthalpy hfg=
      MediumSte.specificEnthalpy(state=
        MediumSte.setState_pTX(p=pSat, T=TSat, X={1})) -
      MediumWat.specificEnthalpy(state=
        MediumWat.setState_pTX(p=pSat, T=TSat, X={1}))
      "Enthalpy of vaporation";
    parameter Modelica.SIunits.Temperature TSubCoo=278.15
      "Degree of subcooling at the heating coil";

    parameter Modelica.SIunits.MassFlowRate mBoi_flow_nominal=QBoi_flow_nominal/hfg
      "Nominal mass flow rate of boiler";
    parameter Modelica.SIunits.HeatFlowRate QBoi_flow_nominal=33380
      "Nominal heat flow rate of boiler";

    parameter Modelica.SIunits.MassFlowRate mCoi1_flow_nominal=QCoi1_flow_nominal/hfg
      "Nominal mass flow rate of Coil";
    parameter Modelica.SIunits.MassFlowRate mCoi2_flow_nominal=QCoi2_flow_nominal/hfg
      "Nominal mass flow rate of Coil";
    parameter Modelica.SIunits.MassFlowRate mCoi3_flow_nominal=QCoi3_flow_nominal/hfg
      "Nominal mass flow rate of Coil";
    parameter Modelica.SIunits.MassFlowRate mCoi4_flow_nominal=QCoi4_flow_nominal/hfg
      "Nominal mass flow rate of Coil";
    parameter Modelica.SIunits.MassFlowRate mCoi5_flow_nominal=QCoi5_flow_nominal/hfg
      "Nominal mass flow rate of Coil";

    parameter Modelica.SIunits.HeatFlowRate QCoi1_flow_nominal=5176
      "Nominal heat flow rate of coil";
    parameter Modelica.SIunits.HeatFlowRate QCoi2_flow_nominal=1608
      "Nominal heat flow rate of coil";
    parameter Modelica.SIunits.HeatFlowRate QCoi3_flow_nominal=5002
      "Nominal heat flow rate of coil";
    parameter Modelica.SIunits.HeatFlowRate QCoi4_flow_nominal=1729
      "Nominal heat flow rate of coil";
    parameter Modelica.SIunits.HeatFlowRate QCoi5_flow_nominal=3060
      "Nominal heat flow rate of coil";

    parameter Modelica.SIunits.PressureDifference dp_nominal=10000
      "Pressure drop at nominal mass flow rate in district network";
    .Buildings.Fluid.Boilers.SteamBoilerSimple boi(
      redeclare package Medium_a = MediumWat,
      redeclare package Medium_b = MediumSte,
      m_flow_nominal=mBoi_flow_nominal,
      TSat=TSat,
      pSat=pSat,
      Q_flow_nominal=QBoi_flow_nominal,
      effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
      a={0.8},
      fue=Data.Fuels.NaturalGasLowerHeatingValue()) "Boiler"
      annotation (Placement(transformation(extent={{40,60},{60,80}})));
    FixedResistances.PressureDrop preDro(
      redeclare package Medium = MediumWat,
      m_flow_nominal=mBoi_flow_nominal,
      dp_nominal=dp_nominal) "Flow resistance"
      annotation (Placement(transformation(extent={{0,60},{20,80}})));
    Modelica.Blocks.Sources.CombiTimeTable datRea(
      tableOnFile=true,
      fileName=ModelicaServices.ExternalReferences.loadResource("modelica://Buildings//Resources/Data/Fluid/Boilers/Validation/SteamBoilerSimpleLoop_EnergyPlus.dat"),
      verboseRead=false,
      columns=2:15,
      tableName="EnergyPlus",
      smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments)
      "Reader for \"CoolingTower_VariableSpeed_Merkel.idf\" energy plus example results"
      annotation (Placement(transformation(extent={{-80,100},{-60,120}})));

    Modelica.Blocks.Math.Gain plr(k=1/QBoi_flow_nominal) "Gain to calculate PLR"
      annotation (Placement(transformation(extent={{-40,100},{-20,120}})));
    Movers.FlowControlled_m_flow pum2(redeclare package Medium = MediumWat,
        m_flow_nominal=mCoi2_flow_nominal)
                                          "Condensate pump"
      annotation (Placement(transformation(extent={{20,-30},{0,-10}})));
    Examples.BaseClasses.SteamCoil
                          coi2(
      redeclare package Medium_a = MediumSte,
      redeclare package Medium_b = MediumWat,
      m_flow_nominal=mCoi2_flow_nominal,
      TSat=TSat,
      pSat=pSat) "Steam coil"
      annotation (Placement(transformation(extent={{60,-30},{40,-10}})));
    Movers.FlowControlled_m_flow pum1(redeclare package Medium = MediumWat,
        m_flow_nominal=mCoi1_flow_nominal)
                                          "Condensate pump"
      annotation (Placement(transformation(extent={{20,10},{0,30}})));
    Examples.BaseClasses.SteamCoil
                          coi1(
      redeclare package Medium_a = MediumSte,
      redeclare package Medium_b = MediumWat,
      m_flow_nominal=mCoi1_flow_nominal,
      TSat=TSat,
      pSat=pSat) "Steam coil"
      annotation (Placement(transformation(extent={{60,10},{40,30}})));
    Movers.FlowControlled_m_flow pum3(redeclare package Medium = MediumWat,
        final m_flow_nominal=mCoi3_flow_nominal)
                                          "Condensate pump"
      annotation (Placement(transformation(extent={{20,-70},{0,-50}})));
    Examples.BaseClasses.SteamCoil
                          coi3(
      redeclare package Medium_a = MediumSte,
      redeclare package Medium_b = MediumWat,
      final m_flow_nominal=mCoi3_flow_nominal,
      TSat=TSat,
      pSat=pSat) "Steam coil"
      annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
    Movers.FlowControlled_m_flow pum4(redeclare package Medium = MediumWat,
        m_flow_nominal=mCoi1_flow_nominal)
                                          "Condensate pump"
      annotation (Placement(transformation(extent={{20,-110},{0,-90}})));
    Examples.BaseClasses.SteamCoil
                          coi4(
      redeclare package Medium_a = MediumSte,
      redeclare package Medium_b = MediumWat,
      m_flow_nominal=mCoi4_flow_nominal,
      TSat=TSat,
      pSat=pSat) "Steam coil"
      annotation (Placement(transformation(extent={{60,-110},{40,-90}})));
    Movers.FlowControlled_m_flow pum5(redeclare package Medium = MediumWat,
        m_flow_nominal=mCoi5_flow_nominal)
                                          "Condensate pump"
      annotation (Placement(transformation(extent={{20,-150},{0,-130}})));
    Examples.BaseClasses.SteamCoil
                          coi5(
      redeclare package Medium_a = MediumSte,
      redeclare package Medium_b = MediumWat,
      m_flow_nominal=mCoi5_flow_nominal,
      TSat=TSat,
      pSat=pSat) "Steam coil"
      annotation (Placement(transformation(extent={{60,-150},{40,-130}})));
  equation
    connect(preDro.port_b, boi.port_a)
      annotation (Line(points={{20,70},{40,70}}, color={0,127,255}));
    connect(plr.y, boi.y) annotation (Line(points={{-19,110},{30,110},{30,78},{38,
            78}},
          color={0,0,127}));
    connect(boi.port_b, coi2.port_a) annotation (Line(points={{60,70},{70,70},{70,
            -20},{60,-20}}, color={0,127,255}));
    connect(boi.port_b, coi1.port_a) annotation (Line(points={{60,70},{70,70},{70,
            20},{60,20}},   color={0,127,255}));
    connect(pum2.port_b, preDro.port_a) annotation (Line(points={{0,-20},{-10,-20},
            {-10,70},{0,70}}, color={0,127,255}));
    connect(pum1.port_b, preDro.port_a) annotation (Line(points={{0,20},{-10,20},{
            -10,70},{0,70}},  color={0,127,255}));
    connect(coi2.port_b, pum2.port_a)
      annotation (Line(points={{40,-20},{20,-20}}, color={0,127,255}));
    connect(coi1.port_b, pum1.port_a)
      annotation (Line(points={{40,20},{20,20}},   color={0,127,255}));
    connect(pum3.port_b, preDro.port_a) annotation (Line(points={{0,-60},{-10,-60},
            {-10,70},{0,70}}, color={0,127,255}));
    connect(pum4.port_b, preDro.port_a) annotation (Line(points={{0,-100},{-10,-100},
            {-10,70},{0,70}}, color={0,127,255}));
    connect(pum5.port_b, preDro.port_a) annotation (Line(points={{0,-140},{-10,-140},
            {-10,70},{0,70}}, color={0,127,255}));
    connect(coi3.port_a, coi2.port_a) annotation (Line(points={{60,-60},{70,-60},{
            70,-20},{60,-20}}, color={0,127,255}));
    connect(coi4.port_a, coi2.port_a) annotation (Line(points={{60,-100},{70,-100},
            {70,-20},{60,-20}}, color={0,127,255}));
    connect(coi5.port_a, coi2.port_a) annotation (Line(points={{60,-140},{70,-140},
            {70,-20},{60,-20}}, color={0,127,255}));
    connect(coi5.port_b, pum5.port_a)
      annotation (Line(points={{40,-140},{20,-140}}, color={0,127,255}));
    connect(coi4.port_b, pum4.port_a) annotation (Line(points={{40,-100},{32,-100},
            {32,-100},{20,-100}}, color={0,127,255}));
    connect(coi3.port_b, pum3.port_a)
      annotation (Line(points={{40,-60},{20,-60}}, color={0,127,255}));
    connect(pum1.m_flow_in, datRea.y[3]) annotation (Line(points={{10,32},{10,40},
            {-50,40},{-50,110},{-59,110}}, color={0,0,127}));
    connect(pum2.m_flow_in, datRea.y[5]) annotation (Line(points={{10,-8},{10,0},{
            -50,0},{-50,110},{-59,110}}, color={0,0,127}));
    connect(pum3.m_flow_in, datRea.y[7]) annotation (Line(points={{10,-48},{10,-40},
            {-50,-40},{-50,110},{-59,110}}, color={0,0,127}));
    connect(pum4.m_flow_in, datRea.y[9]) annotation (Line(points={{10,-88},{10,-80},
            {-50,-80},{-50,110},{-59,110}}, color={0,0,127}));
    connect(pum5.m_flow_in, datRea.y[11]) annotation (Line(points={{10,-128},{10,-120},
            {-50,-120},{-50,110},{-59,110}}, color={0,0,127}));
    connect(plr.u, datRea.y[12])
      annotation (Line(points={{-42,110},{-59,110}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-160},
              {100,140}})),                                        Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-160},{100,140}})),
      experiment(StopTime=100800, __Dymola_Algorithm="Dassl"));
  end SteamBoilerSimpleEnergyPlus;
end Validation;
