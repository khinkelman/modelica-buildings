within Buildings.Controls.OBC.CDL.Reals;
block Multiply "Output product of the two inputs"
  Buildings.Controls.OBC.CDL.Interfaces.RealInput u1
    "Input to be multiplied"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput u2
    "Input to be multiplied"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput y
    "Product of the inputs"
    annotation (Placement(transformation(extent={{100,-20},{140,20}})));

equation
  y=u1*u2;
  annotation (
    defaultComponentName="mul",
    Documentation(
      info="<html>
<p>
Block that outputs <code>y = u1 * u2</code>,
where
<code>u1</code> and <code>u2</code> are inputs.
</p>
</html>",
revisions="<html>
<ul>
<li>
January 27, 2022, by Jianjun Hu:<br/>
Renamed the block name from Product to Multiply.<br/>
This is for <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2865\">Buildings, issue 2865</a>.
</li>
<li>
March 2, 2020, by Michael Wetter:<br/>
Changed icon to display dynamically the output value.
</li>
<li>
January 3, 2017, by Michael Wetter:<br/>
First implementation, based on the implementation of the
Modelica Standard Library.
</li>
</ul>
</html>"),
    Icon(
      coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(
          extent={{-100,-100},{100,100}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          lineColor={0,0,127},
          extent={{-50,-50},{50,50}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-100,60},{-40,60},{-30,40}},
          color={0,0,127}),
        Line(
          points={{-100,-60},{-40,-60},{-30,-40}},
          color={0,0,127}),
        Line(
          points={{50,0},{100,0}},
          color={0,0,127}),
        Line(
          points={{-30,0},{30,0}}),
        Line(
          points={{-15,25.99},{15,-25.99}}),
        Line(
          points={{-15,-25.99},{15,25.99}}),
        Text(
          extent={{-150,150},{150,110}},
          textString="%name",
          textColor={0,0,255}),
        Text(
          extent={{226,60},{106,10}},
          textColor={0,0,0},
          textString=DynamicSelect("",String(y,
            leftJustified=false,
            significantDigits=3)))}));
end Multiply;
