within Buildings.Experimental.DHC.CentralPlants.Cooling.Controls.Validation;
model ChillerStage
  "Example to test the chiller staging controller"
  extends Modelica.Icons.Example;
  Buildings.Experimental.DHC.CentralPlants.Cooling.Controls.ChillerStage chiStaCon(
    tWai=30,
    QChi_nominal=-200*3.517*1000)
    "Chiller staging controller"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Sources.BooleanTable on(
    table(
      each displayUnit="s")={300,900})
    "On signal of the cooling plant"
    annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
  Modelica.Blocks.Sources.Sine mFlo(
    amplitude=0.5*chiStaCon.QChi_nominal/(-10)/4200,
    freqHz=1/300,
    offset=0.5*chiStaCon.QChi_nominal/(-10)/4200)
    "Total mass flow rate"
    annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
  Modelica.Blocks.Sources.Constant TRet(
    k=18)
    "Return temperature"
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  Modelica.Blocks.Sources.Constant TSup(
    k=8)
    "Supply temperature"
    annotation (Placement(transformation(extent={{-60,-20},{-40,0}})));
equation
  connect(on.y,chiStaCon.on)
    annotation (Line(points={{-39,50},{-28,50},{-28,6},{-12,6}},color={255,0,255}));
  connect(mFlo.y,chiStaCon.mFloChiWat)
    annotation (Line(points={{-39,-40},{-28,-40},{-28,-6},{-12,-6}},color={0,0,127}));
  connect(TRet.y,chiStaCon.TChiWatRet)
    annotation (Line(points={{-39,20},{-34,20},{-34,2},{-12,2}},color={0,0,127}));
  connect(TSup.y,chiStaCon.TChiWatSup)
    annotation (Line(points={{-39,-10},{-34,-10},{-34,-2},{-12,-2}},color={0,0,127}));
  annotation (
    Icon(
      coordinateSystem(
        preserveAspectRatio=false)),
    Diagram(
      coordinateSystem(
        preserveAspectRatio=false)),
    experiment(
      StopTime=1200,
      Tolerance=1e-06),
    __Dymola_Commands(
      file="Resources/Scripts/Dymola/Experimental/DHC/CentralPlants/Cooling/Controls/Validation/ChillerStage.mos" "Simulate and Plot"),
    Documentation(
      revisions="<html>
<ul>
<li>
August 6, 2020 by Jing Wang:<br/>
First implementation.
</li>
</ul>
</html>",
      info="<html>
<p>This model validates the chiller staging control logic implemented in
<a href=\"modelica://Buildings.Experimental.DHC.CentralPlants.Cooling.Controls.ChillerStage\">
Buildings.Experimental.DHC.CentralPlants.Cooling.Controls.ChillerStage</a>.
</p>
</html>"));
end ChillerStage;