#include <iostream>

#include "openpnp-capture/common/logging.h"
#include "openpnp-capture/include/openpnp-capture.h"
#include "openpnp-capture/mac/uvcctrl.h"

int main(int argc, char *argv[]) {
    if (argc != 7) {
        std::cerr << "Usage: ns_iokit_ctl <vid> <pid> <location> <disable autoexposure> <exposure> <gain>" <<
                std::endl;
        std::cerr << "\tvid: IOKit Vendor ID in hex" << std::endl;
        std::cerr << "\tpid: IOKit Product ID in hex" << std::endl;
        std::cerr << "\tlocation: IOKit Location ID in hex, or 0 to select the first found location" << std::endl;
        std::cerr << "\tdisable autoexposure: 1 to disable auto exposure, 0 to enable" << std::endl;
        std::cerr << "\texposure: exposure value, decimal" << std::endl;
        std::cerr << "\tgain: gain value, decimal" << std::endl;
        return 1;
    }

    unsigned int vid = std::stoul(argv[1], nullptr, 16);
    unsigned int pid = std::stoul(argv[2], nullptr, 16);
    unsigned int location = std::stoul(argv[3], nullptr, 16);
    int disableAutoexposure = std::stoi(argv[4], nullptr, 10);
    int exposure = std::stoi(argv[5], nullptr, 10);
    int gain = std::stoi(argv[6], nullptr, 10);

    setLogLevel(LOG_DEBUG);
    std::shared_ptr<UVCCtrl> ctrl(UVCCtrl::create(vid, pid, location));

    if (!ctrl) {
        std::cerr << "Failed to create UVCCtrl" << std::endl;
        return 1;
    }

    // Read out initial settings
    bool oldAutoExposure;
    int oldExposure;
    int oldGain;

    ctrl->getAutoProperty(CAPPROPID_EXPOSURE, &oldAutoExposure);
    ctrl->getProperty(CAPPROPID_EXPOSURE, &oldExposure);
    ctrl->getProperty(CAPPROPID_GAIN, &oldGain);

    std::cout << "Disable autoexposure was " << static_cast<int>(!oldAutoExposure) << ", will set to " <<
            disableAutoexposure << std::endl;
    std::cout << "Exposure was " << oldExposure << ", will set to " << exposure << std::endl;
    std::cout << "Gain was " << oldGain << ", will set to " << gain << std::endl;

    // Set new settings
    ctrl->setAutoProperty(CAPPROPID_EXPOSURE, !disableAutoexposure);
    ctrl->setProperty(CAPPROPID_EXPOSURE, exposure);
    ctrl->setProperty(CAPPROPID_GAIN, gain);

    std::cout << "Done" << std::endl;
    return 0;
}
