/*******************************************************************************
* 
* SOFTWARE LICENSE AGREEMENT (BSD LICENSE):
*
* Copyright (c) 2013-2016, Anqi Xu and contributors
* Copyright (C) 2025 Sebastian von der Thannen
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of the School of Computer Science, McGill University,
*    nor the names of its contributors may be used to endorse or promote
*    products derived from this software without specific prior written
*    permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#pragma once

#include <map>
#include <string>
#include <memory>

#include <ueye.h>

// typedef enum
// {
//     SAVE_IMAGE,
//     DELETE_IMAGE,
//     IGNORE_IMAGE

// } CameraImageHandling;

#define NUM_BUFFERS 2

#define CAP(val, min, max) \
    if (val < min)         \
    {                      \
        val = min;         \
    }                      \
    else if (val > max)    \
    {                      \
        val = max;         \
    }

#define IS_SUBSAMPLING_2X  (IS_SUBSAMPLING_2X_VERTICAL | IS_SUBSAMPLING_2X_HORIZONTAL)
#define IS_SUBSAMPLING_4X  (IS_SUBSAMPLING_4X_VERTICAL | IS_SUBSAMPLING_4X_HORIZONTAL)
#define IS_SUBSAMPLING_8X  (IS_SUBSAMPLING_8X_VERTICAL | IS_SUBSAMPLING_8X_HORIZONTAL)
#define IS_SUBSAMPLING_16X (IS_SUBSAMPLING_16X_VERTICAL | IS_SUBSAMPLING_16X_HORIZONTAL)

#define IS_BINNING_2X  (IS_BINNING_2X_VERTICAL | IS_BINNING_2X_HORIZONTAL)
#define IS_BINNING_3X  (IS_BINNING_3X_VERTICAL | IS_BINNING_3X_HORIZONTAL)
#define IS_BINNING_4X  (IS_BINNING_4X_VERTICAL | IS_BINNING_4X_HORIZONTAL)
#define IS_BINNING_5X  (IS_BINNING_5X_VERTICAL | IS_BINNING_5X_HORIZONTAL)
#define IS_BINNING_6X  (IS_BINNING_6X_VERTICAL | IS_BINNING_6X_HORIZONTAL)
#define IS_BINNING_8X  (IS_BINNING_8X_VERTICAL | IS_BINNING_8X_HORIZONTAL)
#define IS_BINNING_16X (IS_BINNING_16X_VERTICAL | IS_BINNING_16X_HORIZONTAL)

/**
 * Stringifies UEye API error flag.
 */
void ids_set_debug(const char *name);
const char *err2str(INT error);
INT name2colormode(const std::string &name);
std::string colormode2name(const INT &mode);
INT colormode2bpp(INT mode);
INT getBinFlag(int rate);

const std::map<std::string, INT> COLOR_DICTIONARY = {
  { "bayer_rggb8", IS_CM_SENSOR_RAW8 },
  { "bayer_rggb10", IS_CM_SENSOR_RAW10 },
  { "bayer_rggb12", IS_CM_SENSOR_RAW12 },
  { "bayer_rggb16", IS_CM_SENSOR_RAW16 },
  { "mono8", IS_CM_MONO8 },
  { "mono10", IS_CM_MONO10 },
  { "mono12", IS_CM_MONO12 },
  { "mono16", IS_CM_MONO16 },
  { "rgb8", IS_CM_RGB8_PACKED },
  { "bgr8", IS_CM_BGR8_PACKED },
  { "rgb10", IS_CM_RGB10_PACKED },
  { "bgr10", IS_CM_BGR10_PACKED },
  { "rgb10u", IS_CM_RGB10_UNPACKED },
  { "bgr10u", IS_CM_BGR10_UNPACKED },
  { "rgb12u", IS_CM_RGB12_UNPACKED },
  { "bgr12u", IS_CM_BGR12_UNPACKED }
};


struct CameraWidget
{
    uint32_t hid;
    std::string serial;
};

int getNumCameras();

std::map<uint32_t, CameraWidget> getCameras();

class IDSCamera
{
  public:
    using UniquePtr = std::unique_ptr<IDSCamera>;
    using SharedPtr = std::shared_ptr<IDSCamera>;

    explicit IDSCamera(int cam_id, std::string cam_name);
    ~IDSCamera();

    inline bool isConnected() { return (cam_handle_ != HIDS(0)); }

    inline bool freeRunModeActive()
    {
        return ((cam_handle_ != HIDS(0)) &&
                (is_SetExternalTrigger(cam_handle_, IS_GET_EXTERNALTRIGGER) == IS_SET_TRIGGER_OFF) &&
                (is_CaptureVideo(cam_handle_, IS_GET_LIVE) == TRUE));
    }

    inline bool extTriggerModeActive()
    {
        return ((cam_handle_ != HIDS(0)) &&
                (is_SetExternalTrigger(cam_handle_, IS_GET_EXTERNALTRIGGER) == IS_SET_TRIGGER_HI_LO) &&
                (is_CaptureVideo(cam_handle_, IS_GET_LIVE) == TRUE));
    }

    inline bool isCapturing() {
      return ((cam_handle_ != HIDS(0)) &&
          (is_CaptureVideo(cam_handle_, IS_GET_LIVE) == TRUE));
    }

    static bool isSupportedColorMode(INT mode);

    INT connectCam();
    INT disconnectCam();

    INT setFlashParams(INT& delay_us, UINT& duration_us);
    INT setGpioMode(const INT& gpio, INT& mode, double& pwm_freq, double& pwm_duty_cycle);
    INT setFreeRunMode();
    INT setExtTriggerMode();
    INT setStandbyMode();

    const char *processNextFrame(UINT timeout_ms);
    long int getFrameBufferSize();

    bool isColorModeSupported(std::string mode);
    INT setColorMode(std::string &mode, bool reallocate_buffer = true);
    INT setResolution(INT &image_width, INT &image_height, INT &image_left, INT &image_top,
                      bool reallocate_buffer = true);
    INT setSubsampling(int &rate, bool reallocate_buffer = true);
    bool isBinningSupported(int rate);
    INT setBinning(int &rate, bool reallocate_buffer = true);
    INT setSensorScaling(double &rate, bool reallocate_buffer = true);
    INT setGain(bool &auto_gain, INT &master_gain_prc, INT &red_gain_prc, INT &green_gain_prc, INT &blue_gain_prc,
                bool &gain_boost);
    INT setSoftwareGamma(INT &software_gamma);
    INT setExposure(bool &auto_exposure, double &auto_exposure_reference, double &exposure_ms);
    INT setWhiteBalance(bool &auto_white_balance, INT &red_offset, INT &blue_offset);
    INT setFrameRate(double &rate);

    void getSensorInfo(double *pixel_size, int *max_width, int *max_height);
    std::string getColorMode();
    INT getSensorColorMode();

    INT getGain(bool &auto_gain, INT &master_gain_prc, INT &red_gain_prc, INT &green_gain_prc, INT &blue_gain_prc,
      bool &gain_boost);
    INT getMinMaxExposure(double *minExposure, double *maxExposure);
    INT getMinMaxLongExposure(double *minExposure, double *maxExposure);
    INT getResolution(INT &image_width, INT &image_height, INT &image_left, INT &image_top);
    INT getExposure(double &exposure_ms);
    INT getWhiteBalance(bool &auto_white_balance, INT &red_offset, INT &blue_offset);

  protected:
    virtual void handleTimeout() {};

    INT reallocateCamBuffer();

  private:
    HIDS cam_handle_;
    int cam_id_;
    std::string cam_name_;

    SENSORINFO cam_sensor_info_;
    char *cam_buffer_;
    int cam_buffer_id_;
    INT cam_buffer_pitch_;
    unsigned int cam_buffer_size_;

    IS_RECT cam_aoi_;
    unsigned int cam_subsampling_rate_;
    unsigned int cam_binning_rate_;
    double cam_sensor_scaling_rate_;

    INT color_mode_;
    INT bits_per_pixel_;

    int m_Width;
    int m_Height;
    long m_StartTime;
    char *m_LockedMemory                 = nullptr;
    char *m_ImageMemoryAddr[NUM_BUFFERS] = { nullptr };
    int m_ImageMemoryId[NUM_BUFFERS];
};

