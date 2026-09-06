# PauloPaperMobileSed_rj

See the Wiki for documentation: [MobileTurbidity Wiki](https://github.com/Robwerg/MobileTurbidity/wiki)

## TODO

- Find out why 107 sd card not working correctly.
  - Program was crashing.
  - Program works if SD_EN is set to 0
  - Updated `file.open("debug.txt")` to `file.open(((char*)"debug.txt")`
  - Program does not write to `debug.txt` and the node file.
  - Program writes to turb files though. I think only if they exist initially.
- Calibrate ALS 0 sensor. Need minimum 2 sets of temp/min/max values to work correctly.
