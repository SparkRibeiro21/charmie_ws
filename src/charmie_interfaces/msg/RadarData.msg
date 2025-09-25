# Simplified radar output from all sectors

std_msgs/Header header
int32 number_of_sectors
float64 sector_ang_range  # Range of each sector in rads
RadarSector[] sectors     # Array of all sectors in order