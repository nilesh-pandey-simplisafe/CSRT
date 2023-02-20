# CSRT-tracker-standalone
Dynamically Build CSRT

mkdir build
cd ./build
cd .. ;rm -r ./build/;mkdir build;cd ./build;cmake ..;make -j9
strip --strip-unneeded csrt 

./csrt /home/nile649/annokin/Alley_11112022_095242PM_EST.mp4  ./lol.mp4

Standalone repository of the CSRT tracker.

size = 276kb
no scaling features.
no colornames features.
