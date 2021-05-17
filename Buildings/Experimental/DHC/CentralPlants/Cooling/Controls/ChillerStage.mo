within Buildings.Experimental.DHC.CentralPlants.Cooling.Controls;
model ChillerStage
  "Chiller staging controller for plants with two same size chillers"
  extends Modelica.Blocks.Icons.Block;
  replaceable package Medium=Buildings.Media.Water
    constrainedby Modelica.Media.Interfaces.PartialMedium
    "Service side medium";
  parameter Modelica.SIunits.Time tWai
    "Waiting time";
  parameter Modelica.SIunits.Power QChi_nominal(
    final max=0)
    "Nominal cooling capaciaty (negative)";
  parameter Modelica.SIunits.Power criPoiLoa=0.55*QChi_nominal
    "Critical point of cooling load for switching one chiller on or off";
  parameter Modelica.SIunits.Power dQ=0.25*QChi_nominal
    "Deadband for critical point of cooling load";
  Modelica.Blocks.Interfaces.BooleanInput on
    "Enabling signal of the plant. True: chiller should be enabled"
    annotation (Placement(transformation(extent={{-200,40},{-160,80}}),iconTransformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput TChiWatRet
    "Chilled water return temperature"
    annotation (Placement(transformation(extent={{-200,0},{-160,40}}),iconTransformation(extent={{-140,0},{-100,40}})),iconTransformation(extent={{-140,0},{-100,40}}));
  Modelica.Blocks.Interfaces.RealInput TChiWatSup
    "Chilled water supply temperature"
    annotation (Placement(transformation(extent={{-200,-40},{-160,0}}),iconTransformation(extent={{-140,-40},{-100,0}})),iconTransformation(extent={{-140,-40},{-100,0}}));
  Modelica.Blocks.Interfaces.RealInput mFloChiWat
    "Chilled water mass flow rate"
    annotation (Placement(transformation(extent={{-200,-80},{-160,-40}}),iconTransformation(extent={{-140,-80},{-100,-40}})));
  Modelica.Blocks.Interfaces.BooleanOutput y[2]
    "On/off signal for the chillers - 0: off; 1: on"
    annotation (Placement(transformation(extent={{160,-10},{180,10}}),iconTransformation(extent={{100,-10},{120,10}})));
  Modelica.StateGraph.InitialStep off(
    nIn=1)
    "No cooling is demanded"
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},rotation=-90,origin={10,70})));
  Modelica.StateGraph.StepWithSignal oneOn(
    nOut=2,
    nIn=2)
    "Status of one chiller on"
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},rotation=90,origin={10,0})));
  Modelica.StateGraph.StepWithSignal twoOn
    "Status of two chillers on"
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},rotation=90,origin={10,-70})));
  Modelica.StateGraph.TransitionWithSignal offToOne(
    enableTimer=true,
    waitTime=tWai)
    "Condition of transition from off to one chiller on"
    annotation (Placement(transformation(extent={{10,10},{-10,-10}},rotation=90,origin={10,40})));
  Modelica.StateGraph.TransitionWithSignal oneToTwo(
    enableTimer=true,
    waitTime=tWai)
    "Condition of transition from one chiller to two chillers"
    annotation (Placement(transformation(extent={{10,10},{-10,-10}},rotation=90,origin={10,-40})));
  Modelica.StateGraph.TransitionWithSignal twoToOne(
    enableTimer=true,
    waitTime=tWai)
    "Condition of transion from two chillers to one chiller"
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},rotation=90,origin={60,-40})));
  Modelica.StateGraph.TransitionWithSignal oneToOff(
    enableTimer=true,
    waitTime=tWai)
    "Condition of transition from one chiller to off"
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},rotation=90,origin={40,40})));
  inner Modelica.StateGraph.StateGraphRoot stateGraphRoot
    "State graph root"
    annotation (Placement(transformation(extent={{120,60},{140,80}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToInteger booToInt(
    final integerTrue=1,
    final integerFalse=0)
    "Boolean to integer"
    annotation (Placement(transformation(extent={{80,-40},{100,-20}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToInteger booToInt1(
    final integerTrue=2,
    final integerFalse=0)
    "Boolean to integer"
    annotation (Placement(transformation(extent={{80,-80},{100,-60}})));
  Buildings.Controls.OBC.CDL.Integers.Add addInt
    "Calculator of chiller stage index. 0: off; 1: one chiller enabled; 2: two chillers enabled"
    annotation (Placement(transformation(extent={{120,-60},{140,-40}})));
  Buildings.Controls.OBC.CDL.Integers.GreaterThreshold chiOne
    "On signal of chiller one"
    annotation (Placement(transformation(extent={{120,20},{140,40}})));
  Buildings.Controls.OBC.CDL.Integers.GreaterThreshold chiTwo(
    final t=1)
    "On signal of chiller two"
    annotation (Placement(transformation(extent={{120,-20},{140,0}})));
  Buildings.Controls.OBC.CDL.Continuous.GreaterThreshold thrOneToTwo(
    final t=criPoiLoa+dQ)
    "Threshold of turning two chillers on"
    annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
  Buildings.Controls.OBC.CDL.Continuous.LessThreshold thrTwoToOne(
    final t=criPoiLoa-dQ)
    "Threshold of turning off the second chiller"
    annotation (Placement(transformation(extent={{-30,-80},{-10,-60}})));
  Buildings.Controls.OBC.CDL.Logical.Not notOn
    "Not on"
    annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
  Modelica.Blocks.Math.Add dT(
    final k1=-1,
    final k2=+1)
    "Temperature difference"
    annotation (Placement(transformation(extent={{-150,-4},{-130,16}})));
  Modelica.Blocks.Math.Product pro
    "Product"
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Blocks.Math.Gain cp(
    final k=-cp_default)
    "Specific heat multiplier to calculate heat flow rate"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
protected
  final parameter Medium.ThermodynamicState sta_default=Medium.setState_pTX(
    T=Medium.T_default,
    p=Medium.p_default,
    X=Medium.X_default)
    "Medium state at default properties";
  final parameter Modelica.SIunits.SpecificHeatCapacity cp_default=Medium.specificHeatCapacityCp(
    sta_default)
    "Specific heat capacity of the fluid";
equation
  connect(off.outPort[1],offToOne.inPort)
    annotation (Line(points={{10,59.5},{10,44}},color={0,0,0}));
  connect(oneToOff.outPort,off.inPort[1])
    annotation (Line(points={{40,41.5},{40,88},{10,88},{10,81}},color={0,0,0}));
  connect(oneToTwo.outPort,twoOn.inPort[1])
    annotation (Line(points={{10,-41.5},{10,-59}},color={0,0,0}));
  connect(twoOn.outPort[1],twoToOne.inPort)
    annotation (Line(points={{10,-80.5},{10,-88},{60,-88},{60,-44}},color={0,0,0}));
  connect(twoToOne.outPort,oneOn.inPort[2])
    annotation (Line(points={{60,-38.5},{60,16},{10.5,16},{10.5,11}},color={0,0,0}));
  connect(offToOne.outPort,oneOn.inPort[1])
    annotation (Line(points={{10,38.5},{10,11},{9.5,11}},color={0,0,0}));
  connect(oneOn.outPort[2],oneToOff.inPort)
    annotation (Line(points={{10.25,-10.5},{10.25,-18},{40,-18},{40,36}},color={0,0,0}));
  connect(oneOn.outPort[1],oneToTwo.inPort)
    annotation (Line(points={{9.75,-10.5},{9.75,-18},{10,-18},{10,-36}},color={0,0,0}));
  connect(addInt.u2,booToInt1.y)
    annotation (Line(points={{118,-56},{110,-56},{110,-70},{102,-70}},color={255,127,0}));
  connect(oneOn.active,booToInt.u)
    annotation (Line(points={{21,0},{70,0},{70,-30},{78,-30}},color={255,0,255}));
  connect(twoOn.active,booToInt1.u)
    annotation (Line(points={{21,-70},{78,-70}},color={255,0,255}));
  connect(booToInt.y,addInt.u1)
    annotation (Line(points={{102,-30},{110,-30},{110,-44},{118,-44}},color={255,127,0}));
  connect(addInt.y,chiTwo.u)
    annotation (Line(points={{142,-50},{150,-50},{150,-30},{114,-30},{114,-10},{118,-10}},color={255,127,0}));
  connect(addInt.y,chiOne.u)
    annotation (Line(points={{142,-50},{150,-50},{150,-30},{114,-30},{114,30},{118,30}},color={255,127,0}));
  connect(chiOne.y,y[1])
    annotation (Line(points={{142,30},{150,30},{150,-5},{170,-5}},color={255,0,255}));
  connect(chiTwo.y,y[2])
    annotation (Line(points={{142,-10},{150,-10},{150,5},{170,5}},color={255,0,255}));
  connect(on,offToOne.condition)
    annotation (Line(points={{-180,60},{-50,60},{-50,40},{-2,40}},color={255,0,255}));
  connect(oneToTwo.condition,thrOneToTwo.y)
    annotation (Line(points={{-2,-40},{-8,-40}},color={255,0,255}));
  connect(thrTwoToOne.y,twoToOne.condition)
    annotation (Line(points={{-8,-70},{-6,-70},{-6,-56},{40,-56},{40,-40},{48,-40}},color={255,0,255}));
  connect(on,notOn.u)
    annotation (Line(points={{-180,60},{-50,60},{-50,20},{-32,20}},color={255,0,255}));
  connect(notOn.y,oneToOff.condition)
    annotation (Line(points={{-8,20},{28,20},{28,40}},color={255,0,255}));
  connect(dT.y,pro.u1)
    annotation (Line(points={{-129,6},{-112,6}},color={0,0,127}));
  connect(cp.u,pro.y)
    annotation (Line(points={{-82,0},{-89,0}},color={0,0,127}));
  connect(cp.y,thrOneToTwo.u)
    annotation (Line(points={{-59,0},{-50,0},{-50,-40},{-32,-40}},color={0,0,127}));
  connect(cp.y,thrTwoToOne.u)
    annotation (Line(points={{-59,0},{-50,0},{-50,-70},{-32,-70}},color={0,0,127}));
  connect(dT.u1,TChiWatRet)
    annotation (Line(points={{-152,12},{-156,12},{-156,20},{-180,20}},color={0,0,127}));
  connect(TChiWatSup,dT.u2)
    annotation (Line(points={{-180,-20},{-156,-20},{-156,0},{-152,0}},color={0,0,127}));
  connect(pro.u2,mFloChiWat)
    annotation (Line(points={{-112,-6},{-120,-6},{-120,-60},{-180,-60}},color={0,0,127}));
  annotation (
    defaultComponentName="chiStaCon",
    Diagram(
      coordinateSystem(
        preserveAspectRatio=false,
        extent={{-160,-100},{160,100}})),
    Icon(
      coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}}),
      graphics={
        Text(
          extent={{-150,150},{150,110}},
          textString="%name",
          lineColor={0,0,255})}),
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
<p>This model implements the staging control logic as follows: </p>
<ul>
<li>When the plant enabling signal <code>on</code> changes from <code>false</code> to <code>true</code>, one chiller is enabled. </li>
<li>When the total cooling load <code>QLoa</code> exceeds 80 percent (adjustable) of one chiller&apos;s nominal capacity <code>QChi_nominal</code>, a second chiller is enabled. </li>
<li>When the total cooling load <code>QLoa</code> drops below 60 percent (adjustable) of one chiller&apos;s nominal capacity <code>QChi_nominal</code>(i.e., 30 percent each chiller), the second chiller is disabled. </li>
<li>When the plant enabling signal <code>on</code> changes from <code>true</code> to <code>false</code>, the operating chiller is disabled.</li>
<li>Parameter <code>tWai</code> assures a transitional time is kept between each operation. </li>
</ul>
<p><br>It is assumed that both chillers have the same capacity of <code>QChi_nominal</code>. </p>
<p>Note: This model can be used for plants with two chillers with or without waterside econimizer (WSE). For plants with WSE, extra control logic on top of this model needs to be added. </p>
<p><img alt=\"State graph\"
src=\"modelica://Buildings/Resources/Images/Experimental/DHC/CentralPlants/Cooling/Controls/ChillerStage.tif\"/>. </p>
</html>"));
end ChillerStage;