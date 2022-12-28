within Buildings.Experimental.DHC.Loads.Cooling;
model BuildingTimeSeriesWithETS
  "Model of a building with loads provided as time series, connected to an ETS for cooling"
  extends BaseClasses.PartialBuildingWithETS(
    final dpRet=0,
    final dpSup=0,
    redeclare BaseClasses.BuildingTimeSeries bui(
      final have_pum=true,
      final filNam=filNam,
      facMulCoo=40*QCoo_flow_nominal/(-1.5E5),
      T_aChiWat_nominal=TChiWatSup_nominal,
      T_bChiWat_nominal=TChiWatRet_nominal),
      mBui_flow_nominal=-QCoo_flow_nominal/(cp*dT_nominal),
      ets(QChiWat_flow_nominal=QCoo_flow_nominal, dp_nominal={0,0,0}));
  final parameter Modelica.Units.SI.HeatFlowRate QCoo_flow_nominal=bui.QCoo_flow_nominal
    "Space cooling design load (<=0)";
  parameter Modelica.Units.SI.TemperatureDifference dT_nominal(min=0)=9
    "Water temperature drop/increase accross load and source-side HX (always positive)"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.Units.SI.Temperature TChiWatSup_nominal=280.15
    "Chilled water supply temperature"
    annotation (Dialog(group="Nominal conditions"));
  final parameter Modelica.Units.SI.Temperature TChiWatRet_nominal=
    TChiWatSup_nominal + dT_nominal
    "Chilled water return temperature"
    annotation (Dialog(group="Nominal condition"));
  parameter String filNam
    "Library path of the file with thermal loads as time series";
protected
  parameter Modelica.Units.SI.SpecificHeatCapacity cp=MediumSer.specificHeatCapacityCp(
    MediumSer.setState_pTX(
      MediumSer.p_default,
      MediumSer.T_default,
      MediumSer.X_default))
    "Default specific heat capacity of medium";
  annotation (
  defaultComponentName="loa",
  Documentation(info="<html>
<p>
This model is composed of a direct uncontrolled energy transfer station model for cooling
<a href=\"modelica://Buildings/Experimental/DHC/EnergyTransferStations/Cooling/DirectUncontrolled.mo\">
Buildings.Experimental.DHC.EnergyTransferStations.Cooling.DirectUncontrolled</a> 
connected to a simplified building model <a href=\"modelica://Buildings/Experimental/DHC/Loads/Cooling/BaseClasses/BuildingTimeSeries.mo\">
Buildings.Experimental.DHC.Loads.Cooling.BaseClasses.BuildingTimeSeries</a> 
where the space cooling loads are provided as time series. 
</p>
</html>", revisions="<html>
<ul>
<li>
December 23, 2022, by Kathryn Hinkelman:<br>
Revised ETS from direct uncontrolled to direct controlled. 
This is for <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2912\">#2912</a>.
</li>
<li>
December 21, 2022 by Kathryn Hinkelman:<br>
Removed in-building pumping because of coupling with the direct/uncontrolled ETS.<br> 
This is for <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2912\">#2912</a>.
</li>
</ul>
<ul>
<li>March 20, 2022 by Chengnan Shi:<br>First implementation. </li>
</ul>
</html>"));
end BuildingTimeSeriesWithETS;
