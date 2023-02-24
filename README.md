# Simpli-CSRT
OPENCV installation

> sudo apt install build-essential cmake git pkg-config libgtk-3-dev \    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \    gfortran openexr libatlas-base-dev python3-dev python3-numpy \    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
> 

> mkdir ~/opencv_build && cd ~/opencv_build;git clone https://github.com/opencv/opencv.git;git clone https://github.com/opencv/opencv_contrib.git;git checkout 3.4.19;cd ~/opencv_build/opencv;mkdir -p build && cd build
>

- Opencv Installation Dynamic library
    
    Dynamic libraries are created as .so (shared object installed in the system). This can be shared by any program which needs dependency on the Opencv
    
    cmake -D CMAKE_BUILD_TYPE=RELEASE \ -D CMAKE_INSTALL_PREFIX=/usr/local \ -D INSTALL_C_EXAMPLES=ON \ -D INSTALL_PYTHON_EXAMPLES=ON \ -D OPENCV_GENERATE_PKGCONFIG=ON \ -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \ -D BUILD_EXAMPLES=ON ..
    
    make -j8;sudo make install
    
- Opencv installation Static library
    
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_SHARED_LIBS=OFF -D OPENCV_GENERATE_PKGCONFIG=ON -D WITH_OPENCL=OFF -D WITH_TBB=OFF ..
    
    make -j8;sudo make install
    
`sudo make uninstall`

`sudo rm -rf /usr/local/include/opencv /usr/local/include/opencv2 /usr/local/share/OpenCV /usr/local/lib/libopencv*`

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
