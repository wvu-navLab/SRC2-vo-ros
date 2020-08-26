#Author: Jared Strader
#I have not tested with the extra modules options on

#opencv version
OPENCV_VERSION='3.2.0'

#install extra modules
INSTALL_EXTRA_MODULES='NO'

#update
sudo apt-get -y update

#dependencies
sudo apt-get install -y build-essential cmake pkg-config #build tools
sudo apt-get install -y git wget #download
sudo apt-get install -y libgtk2.0-dev #gtk
sudo apt-get install -y python-dev python-numpy #python
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libtheora-dev #encoders/decoders
sudo apt-get install -y libjpeg-dev libpng-dev libjasper-dev #images
sudo apt-get install -y libtbb2 libtbb-dev #multi-core (if removed set -DWITH_TBB=OFF)
sudo apt-get install -y qt5-default #qt (if removed set -DWITH_QT=OFF)

#download opencv
cd
wget https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.tar.gz
tar -xf ${OPENCV_VERSION}.tar.gz
rm ${OPENCV_VERSION}.tar.gz
mv opencv-${OPENCV_VERSION} opencv

#download extra modules
if [ $INSTALL_EXTRA_MODULES = 'YES' ]; then
  wget https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.tar.gz
  tar -xf ${OPENCV_VERSION}.tar.gz
  rm ${OPENCV_VERSION}.tar.gz
  mv opencv_contrib-${OPENCV_VERSION} opencv_contrib
  mv opencv_contrib opencv
fi

#build opencv
cd opencv
mkdir build
cd build

if [ $INSTALL_EXTRA_MODULES = 'NO' ]
then
  cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DWITH_TBB=ON -DENABLE_PRECOMPILED_HEADERS=OFF ..
fi

if [ $INSTALL_EXTRA_MODULES = 'YES' ]
then
  cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DWITH_TBB=ON -DENABLE_PRECOMPILED_HEADERS=OFF -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ..
fi

#make
make -j7

#install
sudo make install
sudo ldconfig
