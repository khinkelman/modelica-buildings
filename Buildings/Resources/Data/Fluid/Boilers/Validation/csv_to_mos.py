#!/usr/bin/env python3
import sys
sys.path.append('../../../../Scripts/EnergyPlus')
import energyplus_csv_to_mos as e

if __name__ == '__main__':
  dat_fil = "SteamBoilerSimpleLoop_EnergyPlus.dat"
  output_list =[
   "Environment:Site Outdoor Air Drybulb Temperature [C](Hourly)",
   "SPACE1-1 REHEAT COIL:Heating Coil Heating Rate [W](Hourly)",
   "SPACE1-1 REHEAT COIL:Heating Coil Steam Mass Flow Rate [kg/s](Hourly)",
   "SPACE2-1 REHEAT COIL:Heating Coil Heating Rate [W](Hourly)",
   "SPACE2-1 REHEAT COIL:Heating Coil Steam Mass Flow Rate [kg/s](Hourly)",
   "SPACE3-1 REHEAT COIL:Heating Coil Heating Rate [W](Hourly)",
   "SPACE3-1 REHEAT COIL:Heating Coil Steam Mass Flow Rate [kg/s](Hourly)",
   "SPACE4-1 REHEAT COIL:Heating Coil Heating Rate [W](Hourly)",
   "SPACE4-1 REHEAT COIL:Heating Coil Steam Mass Flow Rate [kg/s](Hourly)",
   "SPACE5-1 REHEAT COIL:Heating Coil Heating Rate [W](Hourly)",
   "SPACE5-1 REHEAT COIL:Heating Coil Steam Mass Flow Rate [kg/s](Hourly)",
   "STEAM BOILER PLANT BOILER:Boiler Heating Rate [W](Hourly)",
   "STEAM BOILER PLANT BOILER:Boiler Steam Efficiency [](Hourly)",
   "Heating:NaturalGas [J](Hourly)"
  ]

  e.energyplus_csv_to_mos(
    output_list = output_list,
    dat_file_name=dat_fil,
    step_size=3600,
    final_time=1036800)
