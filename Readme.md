# PauloPaperMobileSed_rj

See the Wiki for documentation: [MobileTurbidity Wiki](https://github.com/Robwerg/MobileTurbidity/wiki)

## TODO

- Find out why 107 sd card not working correctly.
  - Program was crashing.
  - Updated `file.open("debug.txt")` to `file.open(((char*)"debug.txt")`
  - Program now continues and creates files.
  - Program does not write to `debug.txt` and the node file.
  - Program writes to turb files though.
- Calibrate ALS 0 sensor. Need minimum 2 sets of temp/min/max values to work correctly.
