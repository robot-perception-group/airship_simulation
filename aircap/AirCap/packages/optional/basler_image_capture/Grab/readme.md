Grab:

This is a program adapted from a sample code provided in Pylon software. 'Pylon' is the official driver for basler cameras which comes for free and has a number of utilities. However, it is not fully open source.


Compilation Instructions:

Switch to directory libs (3 levels up from Grab and in the same level as the 'lateral' directory). Uncompress pylon-5.0.1.6388-x86_64.tar.gz from 'third_party' folder and follow its installation instructions.

NOTE: This procedure is same for the server board SBCs (planned for octocopters)

NOTE2: You might need a newer version of ffmpeg, and might need to compile from source:
sudo apt-get install yasm
git clone https://git.ffmpeg.org/ffmpeg.git ffmpeg
cd ffmpeg
./configure --enable-shared
make
sudo make install
