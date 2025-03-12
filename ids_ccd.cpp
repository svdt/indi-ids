/*
    Driver type: IDS Camera INDI Driver

    Copyright (C) 2025 Sebastian von der Thannen

*/

#include "ids_ccd.h"
#include "config.h"

#include <algorithm>
#include <stream/streammanager.h>

#include <sharedblob.h>
#include <deque>
#include <memory>
#include <math.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/stat.h>


static class Loader
{
    public:
        std::deque<std::unique_ptr<IDSCCD>> cameras;

    public:
        Loader()
        {

            int availableCameras = getNumCameras();
            /* Now open all cameras we autodected for usage */
            IDLog("Number of cameras detected: %d.\n", availableCameras);

            if (availableCameras == 0)
            {
                IDLog("Failed to detect any cameras. Check power and make sure camera is not mounted by other programs "
                      "and try again.\n");
                return;
            }

            std::map<uint32_t, CameraWidget> camera_widgets = getCameras();
            for(auto cam = camera_widgets.begin(); cam != camera_widgets.end(); cam++) {
                cameras.push_back(std::unique_ptr<IDSCCD>(new IDSCCD(cam->second.hid, cam->second.serial)));
            }
        }

        ~Loader()
        {
            // TODO free IDS Cams
        }
} loader;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
IDSCCD::IDSCCD(int hid_, std::string serial_) : hid(hid_), serial(serial_)
{
    LOGF_INFO("Init cam: %d %s", hid, serial.c_str());
    std::string fullName = getDefaultName();
    fullName += "[" + serial_ + "]";
    setDeviceName(fullName.c_str());
    ids_set_debug(getDeviceName());

    setVersion(INDI_IDS_VERSION_MAJOR, INDI_IDS_VERSION_MINOR);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char * IDSCCD::getDefaultName()
{
    return "IDS CCD";
}

/////////////////////////////////////////////////////////////////////////////
void IDSCCD::initSwitch(INDI::PropertySwitch &switchSP, int n, const char **names, int set_on)
{

    switchSP.resize(n);
    for (int i = 0; i < n; i++)
    {
        switchSP[i].fill(names[i], names[i], ISS_OFF);
    }

    int onIndex = -1;
    if (IUGetConfigOnSwitchIndex(getDeviceName(), "CAMERAS", &onIndex) == 0)
        switchSP[onIndex].setState(ISS_ON);
    else
        switchSP[set_on].setState(ISS_ON);

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::initProperties()
{
    // Init parent properties first
    INDI::CCD::initProperties();

    ColorModeSP.fill(getDeviceName(), "ColorMode", "Color Mode", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    AWBSwitch.fill(getDeviceName(), "AWB", "AWB", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    const char *awbMode[] = { "off", "on"};
    initSwitch(AWBSwitch, 2, awbMode);

    AWBSettings[RedOffset].fill("AWB_RED_OFFSET", "Red Offset", "%.2f", -50.00, 50.00, 1, 0.00);
    AWBSettings[BlueOffset].fill("AWB_BLUE_OFFSET", "Blue Offset", "%.2f", -50.00, 50.00, 1, 0.00);
    AWBSettings.fill(getDeviceName(), "AWB_PARAMS", "ABW Settings", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    AutoGainSwitch.fill(getDeviceName(), "AutoGain", "Auto Gain", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    const char *gainMode[] = { "off", "on"};
    initSwitch(AutoGainSwitch, 2, gainMode);

    GainNP[MasterGain].fill("GAIN", "Gain", "%.2f", 0.00, 100.00, 1.00, 0.00);
    GainNP[RGain].fill("RGAIN", "Red Gain", "%.2f", 0.00, 100.00, 1.00, 0.00);
    GainNP[GGain].fill("GGAIN", "Green Gain", "%.2f", 0.00, 100.00, 1.00, 0.00);
    GainNP[BGain].fill("BGAIN", "Blue Gain", "%.2f", 0.00, 100.00, 1.00, 0.00);
    GainNP.fill(getDeviceName(), "CCD_GAIN", "Gain", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    // Most cameras have this by default, so let's set it as default.
    BayerTP[2].setText("RGGB");

    SetCCDCapability(CCD_CAN_SUBFRAME | CCD_CAN_BIN | CCD_CAN_ABORT | CCD_HAS_BAYER | CCD_HAS_STREAMING);

    // Add Debug, Simulator, and Configuration controls
    // addAuxControls();
    addDebugControl();

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::updateProperties()
{
    INDI::CCD::updateProperties();

    if (isConnected())
    {
        defineProperty(ColorModeSP);
        defineProperty(AWBSwitch);
        defineProperty(AWBSettings);
        defineProperty(GainNP);
        defineProperty(AutoGainSwitch);

        // if (isSimulation())
        //     isTemperatureSupported = false;
        // else
        //     isTemperatureSupported = gphoto_supports_temperature(gphotodrv);

        if (isTemperatureSupported)
        {
            TemperatureNP.setPermission(IP_RO);
            defineProperty(TemperatureNP);
        }
    }
    else
    {
        deleteProperty(ColorModeSP);
        deleteProperty(AWBSwitch);
        deleteProperty(AWBSettings);
        deleteProperty(GainNP);
        deleteProperty(AutoGainSwitch);

        if (isTemperatureSupported)
            deleteProperty(TemperatureNP);

    }

    return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Color mode config
        if (ColorModeSP.isNameMatch(name))
        {
            ColorModeSP.update(states, names, n);
            ColorModeSP.setState(IPS_OK);
            ColorModeSP.apply();
            saveConfig(ColorModeSP);
            return true;
        }

        // Auto White Blance
        if (AWBSwitch.isNameMatch(name))
        {
            AWBSwitch.update(states, names, n);
            AWBSwitch.setState(IPS_OK);
            AWBSwitch.apply();
            saveConfig(AWBSwitch);
            return true;
        }

        // Auto Gain
        if (AutoGainSwitch.isNameMatch(name))
        {
            AutoGainSwitch.update(states, names, n);
            AutoGainSwitch.setState(IPS_OK);
            AutoGainSwitch.apply();
            saveConfig(AutoGainSwitch);
            return true;
        }
    }

    return INDI::CCD::ISNewSwitch(dev, name, states, names, n);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (AWBSettings.isNameMatch(name))
        {
            AWBSettings.update(values, names, n);
            AWBSettings.setState(IPS_OK);
            AWBSettings.apply();
            saveConfig(AWBSettings);

            return true;
        }
        if (GainNP.isNameMatch(name))
        {
            GainNP.update(values, names, n);
            GainNP.setState(IPS_OK);
            GainNP.apply();
            saveConfig(GainNP);
            return true;
        }
    }

    return INDI::CCD::ISNewNumber(dev, name, values, names, n);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::Connect()
{
    LOGF_INFO("Connecting to %s", serial.c_str());
    ids_camera = std::make_unique<IDSCamera>(hid, serial);
    if(ids_camera->connectCam() != IS_SUCCESS) {
        LOG_ERROR("Failed to connect to camera.");
        return false;
    }
    
    for (auto cm = COLOR_DICTIONARY.cbegin(); cm != COLOR_DICTIONARY.cend(); cm++) {
        std::string mode = cm->first;
        if (ids_camera->isColorModeSupported(mode)) {
            LOGF_DEBUG("Color support %s", mode.c_str());
            colorModes.push_back(mode);
        }
    }
    std::string mode = "mono16";
    if (ids_camera->getSensorColorMode() == IS_COLORMODE_BAYER)
        mode = "bayer_rggb16";
    ids_camera->setColorMode(mode, true);

    std::string color_name = ids_camera->getColorMode();
    auto it = std::find(colorModes.begin(), colorModes.end(), color_name);
    int set_on = it - colorModes.begin();

    std::vector<const char *> colorModesLabel;
    for (size_t i = 0; i < colorModes.size(); i++) {
        colorModesLabel.push_back(colorModes[i].c_str());
    }
    initSwitch(ColorModeSP, colorModesLabel.size(), colorModesLabel.data(), set_on);

    bool auto_wb = false;
    INT r_off = AWBSettings[RedOffset].getValue();
    INT b_off = AWBSettings[BlueOffset].getValue();
    ids_camera->getWhiteBalance(auto_wb, r_off, b_off);
    LOGF_DEBUG("Got awb settings: auto_wb: %s\n  red offset: %d\n blue offset: %d\n", (auto_wb) ? "enabled" : "disabled", r_off, b_off);

    AWBSettings[RedOffset].setValue(r_off);
    AWBSettings[RedOffset].setValue(b_off);

    bool auto_gain = false;
    INT main_gain = GainNP[MasterGain].getValue();
    INT r_gain = GainNP[RGain].getValue();
    INT g_gain = GainNP[GGain].getValue();
    INT b_gain = GainNP[BGain].getValue();
    bool gain_boost = true;
    ids_camera->getGain(auto_gain, main_gain, r_gain, g_gain, b_gain, gain_boost);

    LOGF_DEBUG("Got gain settings: auto_gain: %s\n  master gain: %d\n  red gain: %d\n  green gain: %d\n  blue gain: %d\n  gain boost: %s", (auto_gain) ? "enabled" : "disabled",
         main_gain, r_gain, g_gain, b_gain, (gain_boost) ? "enabled" : "disabled");

    GainNP[MasterGain].setValue(main_gain);
    GainNP[RGain].setValue(r_gain);
    GainNP[GGain].setValue(g_gain);
    GainNP[BGain].setValue(b_gain);

    double pixel = 0;
    int nMaxWidth = 0, nMaxHeight = 0, bits_per_pixel = 8;
    ids_camera->getSensorInfo(&pixel, &nMaxWidth, &nMaxHeight);

    PrimaryCCD.setFrame(0, 0, nMaxWidth, nMaxHeight);
    PrimaryCCD.setResolution(nMaxWidth, nMaxHeight);
    PrimaryCCD.setPixelSize(pixel, pixel);
    PrimaryCCD.setBin(1, 1);
    PrimaryCCD.setBPP(bits_per_pixel);

    auto nvp = PrimaryCCD.getCCDInfo();
    if (!nvp.isValid())
        return false;

    // Load the necessary pixel size information
    // The maximum resolution and bits per pixel depend on the capture itself.
    // while the pixel size data remains constant.
    if (pixel > 0) {
        nvp[INDI::CCDChip::CCD_PIXEL_SIZE].setValue(pixel);
        nvp[INDI::CCDChip::CCD_PIXEL_SIZE_X].setValue(pixel);
        nvp[INDI::CCDChip::CCD_PIXEL_SIZE_Y].setValue(pixel);
    }
    if (nMaxWidth > 0)
        nvp[INDI::CCDChip::CCD_MAX_X].setValue(nMaxWidth);
    if (nMaxHeight > 0)
        nvp[INDI::CCDChip::CCD_MAX_Y].setValue(nMaxHeight);
    if (nMaxHeight > 0)
        nvp[INDI::CCDChip::CCD_BITSPERPIXEL].setValue(bits_per_pixel);

    double fps = 0.1;
    ids_camera->setFrameRate(fps);

    double min_exposure, max_exposure;
    ids_camera->getMinMaxExposure(&min_exposure, &max_exposure);
    double max_exp_s = max_exposure / 1000.;
    char sz[64];
    sprintf(sz, "%.3lf\n", max_exp_s);
    max_exp_s = atof(sz);
    LOGF_INFO("Exposure min: %f max: %f", min_exposure / 1000., max_exp_s);

    PrimaryCCD.setMinMaxStep("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", min_exposure / 1000., max_exp_s, 0.001, true);

    for (int bin_i = 2; bin_i < 17; bin_i++) {
        if (ids_camera->isBinningSupported(bin_i)) {
            binModes.push_back(bin_i);
            LOGF_DEBUG("bin %d is supported.", bin_i);
        } else {
            LOGF_DEBUG("bin %d is not supported.", bin_i);
        }
    }
    int max_bin = 1;
    if (binModes.size() > 0)
        max_bin = binModes.back();
    LOGF_DEBUG("Bin min: %d max: %d", 1, max_bin);
    PrimaryCCD.setMinMaxStep("CCD_BINNING", "HOR_BIN", 1, max_bin, 1, true);
    PrimaryCCD.setMinMaxStep("CCD_BINNING", "VER_BIN", 1, max_bin, 1, true);

    LOGF_INFO("%s is online.", getDeviceName());
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::Disconnect()
{
    if (ids_camera)
    {
        ids_camera->disconnectCam();
        ids_camera.reset();
    }
    LOGF_INFO("%s is offline.", getDeviceName());
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::StartExposure(float duration)
{
    if (PrimaryCCD.getPixelSizeX() == 0.0)
    {
        LOG_INFO("Please update the CCD Information in the Image Info section before "
                 "proceeding. The camera resolution shall be updated after the first exposure "
                 "is complete.");
        return false;
    }

    if (InExposure)
    {
        LOG_ERROR("GPhoto driver is already exposing.");
        return false;
    }

    std::string color_mode = colorModes[ColorModeSP.findOnSwitchIndex()];
    ids_camera->setColorMode(color_mode);
    
    bool auto_wb = false;
    INT r_off = AWBSettings[RedOffset].getValue();
    INT b_off = AWBSettings[BlueOffset].getValue();
    ids_camera->setWhiteBalance(auto_wb, r_off, b_off);

    bool auto_gain = false;
    if (AutoGainSwitch.findOnSwitchIndex() == 1) {
        auto_gain = true;
        LOG_INFO("Enable auto gain.");
    }
    INT main_gain = GainNP[MasterGain].getValue();
    INT r_gain = GainNP[RGain].getValue();
    INT g_gain = GainNP[GGain].getValue();
    INT b_gain = GainNP[BGain].getValue();
    bool gain_boost = true;
    ids_camera->setGain(auto_gain, main_gain, r_gain, g_gain, b_gain, gain_boost);

    bool auto_exposure = false;
    double exposure_target = 128;

    // Miliseconds
    double exp_ms = duration * 1e3;
    double min_exposure, max_exposure;
    ids_camera->getMinMaxExposure(&min_exposure, &max_exposure);
    CAP(exp_ms, min_exposure, max_exposure);
    duration = exp_ms / 1e3;
    PrimaryCCD.setExposureDuration(duration);
    if (IS_SUCCESS == ids_camera->setExposure(auto_exposure, exposure_target, exp_ms)) {
        LOGF_INFO("Starting %g seconds exposure.", duration);
    } else {
        LOG_ERROR("Error starting exposure");
        return false;
    }
    if (IS_SUCCESS == ids_camera->getExposure(exp_ms)) {
        LOGF_INFO("Exposure set to %g seconds.", exp_ms / 1e3);
    } else {
        LOG_ERROR("Error getting exposure");
        return false;
    }

    INT binx = PrimaryCCD.getBinX();
    LOGF_INFO("Set binning to %d.", binx);
    if (!ids_camera->isBinningSupported(binx)) {
        LOGF_ERROR("Binning %dX is not supported!", binx);
        return false;
    } else if (ids_camera->setBinning(binx, true) != IS_SUCCESS) {
        LOGF_ERROR("Error could not set binning to %d", binx);
        PrimaryCCD.setBin(1, 1);
        return false;
    }

    INT w = PrimaryCCD.getSubW();
    INT h = PrimaryCCD.getSubH();
    INT x = PrimaryCCD.getSubX();
    INT y = PrimaryCCD.getSubY();
    LOGF_INFO("Rect w: %d, h: %d, x: %d, y: %d", w, h, x, y);
    ids_camera->setResolution(w, h, x, y, true);
    
    LOGF_INFO("Starting %g seconds exposure.", duration);
    if (ids_camera->setFreeRunMode() != IS_SUCCESS) {
        LOG_ERROR("Error starting exposure");
        return false;
    }

    ExposureRequest = duration;
    gettimeofday(&ExpStart, nullptr);
    InExposure = true;

    SetTimer(getCurrentPollingPeriod());

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::AbortExposure()
{
    InExposure = false;
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::UpdateCCDFrame(int x, int y, int w, int h)
{
    LOGF_INFO("Update CCD w: %d, h: %d, x: %d, y: %d", w, h, x, y);

    PrimaryCCD.setFrame(x, y, w, h);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::UpdateCCDBin(int hor, int ver)
{
    LOGF_INFO("UpdateCCDBin CCD hor: %d, ver: %d", hor, ver);

    return INDI::CCD::UpdateCCDBin(hor, ver);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double IDSCCD::CalcTimeLeft()
{
    struct timeval now, diff;
    gettimeofday(&now, nullptr);

    timersub(&now, &ExpStart, &diff);
    double timesince = diff.tv_sec + diff.tv_usec / 1000000.0;
    return (ExposureRequest - timesince);
}

void IDSCCD::TimerHit()
{
    if (isConnected() == false)
        return;

    LOG_INFO("TimerHit");
    if (InExposure)
    {
        int timerID   = -1;
        double timeleft = CalcTimeLeft();

        if (timeleft < 0)
            timeleft = 0;

        PrimaryCCD.setExposureLeft(timeleft);
        LOGF_INFO("Time left %f", timeleft);

        if (timeleft < 1.0)
        {
            if (timeleft > 0.25)
                timerID = SetTimer(timeleft * 900);
            else
            {
                PrimaryCCD.setExposureLeft(0);
                InExposure = false;
                // grab and save image
                bool rc = grabImage();
                if (rc == false)
                {
                    PrimaryCCD.setExposureFailed();
                }
            }

            // if (isTemperatureSupported)
            // {
            //     double cameraTemperature = ids_camera->getTemperature();
            //     if (fabs(cameraTemperature - TemperatureNP[0].getValue()) > 0.01)
            //     {
            //         // Check if we are getting bogus temperature values and set property to alert
            //         // unless it is already set
            //         if (cameraTemperature < MINUMUM_CAMERA_TEMPERATURE)
            //         {
            //             if (TemperatureNP.getState() != IPS_ALERT)
            //             {
            //                 TemperatureNP.setState(IPS_ALERT);
            //                 TemperatureNP.apply();
            //             }
            //         }
            //         else
            //         {
            //             TemperatureNP.setState(IPS_OK);
            //             TemperatureNP[0].setValue(cameraTemperature);
            //             TemperatureNP.apply();
            //         }
            //     }
            // }
        }

        if (InExposure && timerID == -1)
            SetTimer(getCurrentPollingPeriod());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::grabImage()
{
    LOG_DEBUG("Grabbing image...");
    uint8_t * memptr = PrimaryCCD.getFrameBuffer();
    const char * previewData = nullptr;
    size_t memsize = 0;
    int naxis = 2;
    INT w = 0, h = 0, x = 0, y = 0;

    double exp_ms;
    ids_camera->getExposure(exp_ms);
    UINT eventTimeout = exp_ms + 1000;

    previewData = ids_camera->processNextFrame(eventTimeout);
    if (previewData != nullptr) {
        memptr = reinterpret_cast<uint8_t *>(const_cast<char *>(previewData));
        memsize = ids_camera->getFrameBufferSize();
    }
    ids_camera->getResolution(w, h, x, y);
    LOGF_INFO("Area Of Interest (AOI) is %d x %d with top-left corner at (%d, %d)", w, h, x, y);

    // if (w > 0 && h > 0)
    //     PrimaryCCD.setFrame(x, y, w, h);
    PrimaryCCD.setFrameBuffer(memptr);
    PrimaryCCD.setFrameBufferSize(memsize, false);
    // if (w > 0 && h > 0)
    //     PrimaryCCD.setResolution(w, h);
    PrimaryCCD.setNAxis(naxis);
    auto color_mode = ids_camera->getColorMode();
    PrimaryCCD.setBPP(colormode2bpp(name2colormode(color_mode)));

    ExposureComplete(&PrimaryCCD);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::StartStreaming()
{
    LOG_INFO("Starting live view...log_info");
    INT w = PrimaryCCD.getSubW();
    INT h = PrimaryCCD.getSubH();
    INT width = PrimaryCCD.getXRes();
    INT height = PrimaryCCD.getYRes();
    INT x = PrimaryCCD.getSubX();
    INT y = PrimaryCCD.getSubY();
    LOGF_INFO("w: %d, h: %d, x: %d, y: %d, width: %d, height: %d", w, h, x, y, width, height);

    INT binx = PrimaryCCD.getBinX();
    ids_camera->setBinning(binx, true);
    ids_camera->getResolution(w, h, x, y);

    std::string mode = "mono8";
    ids_camera->setColorMode(mode, true);

    LOGF_INFO("Area Of Interest (AOI) is %d x %d with top-left corner at (%d, %d)", w, h, x, y);
    ids_camera->setResolution(w, h, x, y, true);

    // if (gphoto_start_preview(gphotodrv) == GP_OK)
    if (ids_camera && ids_camera->isConnected())
    {
        Streamer->setPixelFormat(INDI_MONO);
        std::unique_lock<std::mutex> guard(liveStreamMutex);
        m_RunLiveStream = true;
        guard.unlock();
        m_LiveViewThread = std::thread(&IDSCCD::streamLiveView, this);
        return true;
    }

    return false;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::StopStreaming()
{
    std::unique_lock<std::mutex> guard(liveStreamMutex);
    m_RunLiveStream = false;
    guard.unlock();
    m_LiveViewThread.join();
    // return (gphoto_stop_preview(gphotodrv) == GP_OK);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IDSCCD::streamLiveView()
{
    const char * previewData = nullptr;
    unsigned long int previewSize = 0;

    double fps_stream = Streamer->getTargetFPS();
    LOGF_INFO("Sfps %f", fps_stream);

    double fps = fps_stream;
    ids_camera->setFrameRate(fps);

    double min_exposure, max_exposure;
    ids_camera->getMinMaxExposure(&min_exposure, &max_exposure);

    bool auto_exposure = false;
    double exposure_target = 128;
    double exposure = 0;
    double max_fps = 30.;

    UINT eventTimeout = 1000;

    while (true)
    {
        std::unique_lock<std::mutex> guard(liveStreamMutex);
        if (m_RunLiveStream == false)
            break;
        guard.unlock();

        auto c_start = std::clock();
        double exposure_update = Streamer->getTargetExposure() * 1000;
        ids_camera->getMinMaxExposure(&min_exposure, &max_exposure);
        CAP(exposure_update, min_exposure, max_exposure);
        if (exposure != exposure_update) {
            LOGF_DEBUG("Setting exposure from %f to %f", exposure, exposure_update);
            exposure = exposure_update;
            if (ids_camera->setExposure(auto_exposure, exposure_target, exposure_update) != IS_SUCCESS) {
                LOG_ERROR("Error setting exposure.");
                break;
            }
        }

        // double fps_update = Streamer->getTargetFPS();
        // double fps_update = 1 / exposure;
        // CAP(fps_update, 0.1, 1 / exposure);
        // if (fps != fps_update) {
        //     fps = fps_update;
        //     if (ids_camera->setFrameRate(fps) != IS_SUCCESS) {
        //         LOG_ERROR("Error setting fps.");
        //         break;
        //     }
        // }

        if (ids_camera->setFreeRunMode() != IS_SUCCESS) {
            break;
        }

        if (ids_camera->isCapturing()) {
            previewData = ids_camera->processNextFrame(eventTimeout);
            if (previewData) {
                previewSize = ids_camera->getFrameBufferSize();
            } else {
                continue;
            }
        }

        uint8_t * inBuffer = reinterpret_cast<uint8_t *>(const_cast<char *>(previewData));
        std::unique_lock<std::mutex> ccdguard(ccdBufferLock);
        Streamer->newFrame(inBuffer, previewSize);
        ccdguard.unlock();

        auto c_stop = std::clock();
        double dtime = 1e6 * (c_stop - c_start) / CLOCKS_PER_SEC;
        if (dtime * max_fps < 1e6)
            usleep((1e6 / max_fps - dtime));
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::saveConfigItems(FILE * fp)
{
    // Save regular CCD properties
    INDI::CCD::saveConfigItems(fp);

    ColorModeSP.save(fp);
    AWBSwitch.save(fp);
    AWBSettings.save(fp);
    GainNP.save(fp);
    AutoGainSwitch.save(fp);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IDSCCD::addFITSKeywords(INDI::CCDChip * targetChip, std::vector<INDI::FITSRecord> &fitsKeywords)
{
    INDI::CCD::addFITSKeywords(targetChip, fitsKeywords);

    if (isTemperatureSupported)
    {
        fitsKeywords.push_back({"CCD-TEMP", TemperatureNP[0].getValue(), 3, "CCD Temperature (Celsius)"});
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::SetCaptureFormat(uint8_t index)
{
    INDI_UNUSED(index);
    return true;
}

