#include "ids_driver.h"
#include <iostream>

int main() {
    auto camera_list = getCameras();
    if (camera_list.empty()) {
        std::cout << "empty" << std::endl;
        return 1;
    }

    auto id = camera_list.begin()->first;
    auto cam = camera_list.begin()->second;
    std::cout << "Found " << id << " serial: " << cam.serial << std::endl;

    auto ids_camera = std::make_unique<IDSCamera>(cam.hid, cam.serial);
    if(ids_camera->connectCam() != IS_SUCCESS) {
        std::cout << "Failed to connect to camera." << std::endl;
        return 1;
    }

    ids_camera->disconnectCam();

    return 0;
}