//
// Created by glawless on 01.09.17.
//

#include <neural_network_detector/Overlay.h>

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "neural_network_detector_overlay");

    neural_network_detector::Overlay overlay;

    ros::spin();

    return EXIT_SUCCESS;
}