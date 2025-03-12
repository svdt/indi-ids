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

#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <thread>

#include <config.h>
#include <indilogger.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic pop

#include "ids_driver.h"
#include <iostream>
#include <cmath>

static char device[64];

void ids_set_debug(const char *name)
{
    strncpy(device, name, 64);
}

const char* err2str(INT error) {
#define CASE(s) case s: return #s; break
  switch (error) {
  CASE(IS_NO_SUCCESS);
  CASE(IS_SUCCESS);
  CASE(IS_INVALID_CAMERA_HANDLE);
  CASE(IS_IO_REQUEST_FAILED);
  CASE(IS_CANT_OPEN_DEVICE);
  CASE(IS_CANT_OPEN_REGISTRY);
  CASE(IS_CANT_READ_REGISTRY);
  CASE(IS_NO_IMAGE_MEM_ALLOCATED);
  CASE(IS_CANT_CLEANUP_MEMORY);
  CASE(IS_CANT_COMMUNICATE_WITH_DRIVER);
  CASE(IS_FUNCTION_NOT_SUPPORTED_YET);
  CASE(IS_INVALID_CAPTURE_MODE);
  CASE(IS_INVALID_MEMORY_POINTER);
  CASE(IS_FILE_WRITE_OPEN_ERROR);
  CASE(IS_FILE_READ_OPEN_ERROR);
  CASE(IS_FILE_READ_INVALID_BMP_ID);
  CASE(IS_FILE_READ_INVALID_BMP_SIZE);
  CASE(IS_NO_ACTIVE_IMG_MEM);
  CASE(IS_SEQUENCE_LIST_EMPTY);
  CASE(IS_CANT_ADD_TO_SEQUENCE);
  CASE(IS_SEQUENCE_BUF_ALREADY_LOCKED);
  CASE(IS_INVALID_DEVICE_ID);
  CASE(IS_INVALID_BOARD_ID);
  CASE(IS_ALL_DEVICES_BUSY);
  CASE(IS_TIMED_OUT);
  CASE(IS_NULL_POINTER);
  CASE(IS_INVALID_PARAMETER);
  CASE(IS_OUT_OF_MEMORY);
  CASE(IS_ACCESS_VIOLATION);
  CASE(IS_NO_USB20);
  CASE(IS_CAPTURE_RUNNING);
  CASE(IS_IMAGE_NOT_PRESENT);
  CASE(IS_TRIGGER_ACTIVATED);
  CASE(IS_CRC_ERROR);
  CASE(IS_NOT_YET_RELEASED);
  CASE(IS_WAITING_FOR_KERNEL);
  CASE(IS_NOT_SUPPORTED);
  CASE(IS_TRIGGER_NOT_ACTIVATED);
  CASE(IS_OPERATION_ABORTED);
  CASE(IS_BAD_STRUCTURE_SIZE);
  CASE(IS_INVALID_BUFFER_SIZE);
  CASE(IS_INVALID_PIXEL_CLOCK);
  CASE(IS_INVALID_EXPOSURE_TIME);
  CASE(IS_AUTO_EXPOSURE_RUNNING);
  CASE(IS_CANNOT_CREATE_BB_SURF);
  CASE(IS_CANNOT_CREATE_BB_MIX);
  CASE(IS_BB_OVLMEM_NULL);
  CASE(IS_CANNOT_CREATE_BB_OVL);
  CASE(IS_NOT_SUPP_IN_OVL_SURF_MODE);
  CASE(IS_INVALID_SURFACE);
  CASE(IS_SURFACE_LOST);
  CASE(IS_RELEASE_BB_OVL_DC);
  CASE(IS_BB_TIMER_NOT_CREATED);
  CASE(IS_BB_OVL_NOT_EN);
  CASE(IS_ONLY_IN_BB_MODE);
  CASE(IS_INVALID_COLOR_FORMAT);
  CASE(IS_INVALID_WB_BINNING_MODE);
  CASE(IS_INVALID_I2C_DEVICE_ADDRESS);
  CASE(IS_COULD_NOT_CONVERT);
  CASE(IS_TRANSFER_ERROR);
  CASE(IS_PARAMETER_SET_NOT_PRESENT);
  CASE(IS_INVALID_CAMERA_TYPE);
  CASE(IS_INVALID_HOST_IP_HIBYTE);
  CASE(IS_CM_NOT_SUPP_IN_CURR_DISPLAYMODE);
  CASE(IS_NO_IR_FILTER);
  CASE(IS_STARTER_FW_UPLOAD_NEEDED);
  CASE(IS_DR_LIBRARY_NOT_FOUND);
  CASE(IS_DR_DEVICE_OUT_OF_MEMORY);
  CASE(IS_DR_CANNOT_CREATE_SURFACE);
  CASE(IS_DR_CANNOT_CREATE_VERTEX_BUFFER);
  CASE(IS_DR_CANNOT_CREATE_TEXTURE);
  CASE(IS_DR_CANNOT_LOCK_OVERLAY_SURFACE);
  CASE(IS_DR_CANNOT_UNLOCK_OVERLAY_SURFACE);
  CASE(IS_DR_CANNOT_GET_OVERLAY_DC);
  CASE(IS_DR_CANNOT_RELEASE_OVERLAY_DC);
  CASE(IS_DR_DEVICE_CAPS_INSUFFICIENT);
  CASE(IS_INCOMPATIBLE_SETTING);
  CASE(IS_DR_NOT_ALLOWED_WHILE_DC_IS_ACTIVE);
  CASE(IS_DEVICE_ALREADY_PAIRED);
  CASE(IS_SUBNETMASK_MISMATCH);
  CASE(IS_SUBNET_MISMATCH);
  CASE(IS_INVALID_IP_CONFIGURATION);
  CASE(IS_DEVICE_NOT_COMPATIBLE);
  CASE(IS_NETWORK_FRAME_SIZE_INCOMPATIBLE);
  CASE(IS_NETWORK_CONFIGURATION_INVALID);
  CASE(IS_ERROR_CPU_IDLE_STATES_CONFIGURATION);
  default:
    return "UNKNOWN ERROR";
  }
  return "UNKNOWN ERROR";
#undef CASE
}


INT name2colormode(const std::string& name) {
  const std::map<std::string, INT>::const_iterator iter = COLOR_DICTIONARY.find(name);
  if (iter!=COLOR_DICTIONARY.end()) {
    return iter->second;
  }
  else {
    return 0;
  }
}

std::string colormode2name(const INT &mode) {
  for (auto cm = COLOR_DICTIONARY.cbegin(); cm != COLOR_DICTIONARY.cend(); cm++) {
    if (cm->second == mode)
      return cm->first;
  }
  return "";
}

INT colormode2bpp(INT mode) {
  switch (mode) {
    case IS_CM_SENSOR_RAW8:
    case IS_CM_MONO8:
      return 8;
    case IS_CM_SENSOR_RAW10:
    case IS_CM_SENSOR_RAW12:
    case IS_CM_SENSOR_RAW16:
    case IS_CM_MONO10:
    case IS_CM_MONO12:
    case IS_CM_MONO16:
    case IS_CM_BGR5_PACKED:
    case IS_CM_BGR565_PACKED:
    case IS_CM_UYVY_PACKED:
    case IS_CM_UYVY_MONO_PACKED:
    case IS_CM_UYVY_BAYER_PACKED:
    case IS_CM_CBYCRY_PACKED:
      return 16;
    case IS_CM_RGB8_PACKED:
    case IS_CM_BGR8_PACKED:
    case IS_CM_RGB8_PLANAR:
      return 24;
    case IS_CM_RGBA8_PACKED:
    case IS_CM_BGRA8_PACKED:
    case IS_CM_RGBY8_PACKED:
    case IS_CM_BGRY8_PACKED:
    case IS_CM_RGB10_PACKED:
    case IS_CM_BGR10_PACKED:
      return 32;
    case IS_CM_RGB10_UNPACKED:
    case IS_CM_BGR10_UNPACKED:
    case IS_CM_RGB12_UNPACKED:
    case IS_CM_BGR12_UNPACKED:
      return 48;
    case IS_CM_RGBA12_UNPACKED:
    case IS_CM_BGRA12_UNPACKED:
      return 64;
//   case IS_CM_JPEG:
    default:
      return 0;
  }
}

INT getBinFlag(int rate) {
  INT rate_flag;
  switch (rate) {
    case 1:
      rate_flag = IS_BINNING_DISABLE;
      break;
    case 2:
      rate_flag = IS_BINNING_2X;
      break;
    case 3:
      rate_flag = IS_BINNING_3X;
      break;
    case 4:
      rate_flag = IS_BINNING_4X;
      break;
    case 5:
      rate_flag = IS_BINNING_5X;
      break;
    case 6:
      rate_flag = IS_BINNING_6X;
      break;
    case 8:
      rate_flag = IS_BINNING_8X;
      break;
    case 16:
      rate_flag = IS_BINNING_16X;
      break;
    default:
      rate = 1;
      rate_flag = IS_BINNING_DISABLE;
      break;
  }
  return rate_flag;
}

int getNumCameras()
{
    int n = 0;
    if (is_GetNumberOfCameras(&n) == IS_SUCCESS)
        return n;
    else
    {
        DEBUGDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to get numer of cameras...");
        return 0;
    }
}

std::map<uint32_t, CameraWidget> getCameras()
{
    UEYE_CAMERA_LIST ucl;
    ucl.dwCount = getNumCameras();
    std::map<uint32_t, CameraWidget> cameras;
    if (is_GetCameraList(&ucl) == IS_SUCCESS)
    {
        for (size_t i = 0; i < ucl.dwCount; i++)
        {
            auto &info = ucl.uci[i];
            DEBUGFDEVICE(device, INDI::Logger::DBG_SESSION, "Camera DeviceID: %d, CameraID: %d, SensorID: %d, inUse: %d", info.dwDeviceID, info.dwCameraID, info.dwSensorID, info.dwInUse);
            std::string serial(info.SerNo);
            CameraWidget cam{ info.dwCameraID, serial };
            cameras.emplace(info.dwCameraID, cam);
        }
    }
    else
    {
        DEBUGDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to get camera list.");
    }
    return cameras;
}

IDSCamera::IDSCamera(int cam_id, std::string cam_name)
 : cam_handle_(0), cam_id_(cam_id), cam_name_(cam_name),
 cam_buffer_(nullptr),
 cam_buffer_id_(0),
 cam_buffer_pitch_(0),
 cam_buffer_size_(0),
 cam_subsampling_rate_(1),
 cam_binning_rate_(1),
 cam_sensor_scaling_rate_(1),
 color_mode_(IS_CM_MONO8),
 bits_per_pixel_(8) {}

IDSCamera::~IDSCamera()
{
  disconnectCam();
}

INT IDSCamera::connectCam()
{
    INT is_err = IS_SUCCESS;
    int numCameras;

    // Terminate any existing opened cameras
    is_err = setStandbyMode();

    // Initialize camera handle
    numCameras = getNumCameras();
    if (numCameras == 0)
    {
      DEBUGDEVICE(device, INDI::Logger::DBG_ERROR, "No cameras found.");
      return IS_NO_SUCH_DEVICE;
    }

    cam_handle_ = static_cast<HIDS>(cam_id_);
    is_err = is_InitCamera(&cam_handle_, NULL);
    if (is_err == IS_STARTER_FW_UPLOAD_NEEDED)
    {
      DEBUGDEVICE(device, INDI::Logger::DBG_DEBUG, "Camera needs firmware update.");

        INT uploadTimeMSEC = 25000;
        is_GetDuration(cam_handle_, IS_STARTER_FW_UPLOAD, &uploadTimeMSEC);

        DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Uploading new firmware to [%s]; please wait for about %f seconds", cam_name_.c_str(), uploadTimeMSEC / 1000.0);

        // Attempt to re-open camera handle while triggering automatic firmware upload
        cam_handle_ = static_cast<HIDS>(static_cast<INT>(cam_handle_) | IS_ALLOW_STARTER_FW_UPLOAD);
        is_err      = is_InitCamera(&cam_handle_, nullptr); // Will block for N seconds
        return IS_NO_SUCCESS;
    }
    else if (is_err != IS_SUCCESS)
    {
        DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to init camera. Error: %d", is_err);
        return IS_NO_SUCCESS;
    }
    // Set display mode to Device Independent Bitmap (DIB)
    is_err = is_SetDisplayMode(cam_handle_, IS_SET_DM_DIB);
    if (is_err != IS_SUCCESS)
    {
      DEBUGDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to set display mode.");
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "UEye camera ID %s does not support Device Independent Bitmap mode; driver wrapper not compatible with OpenGL/DirectX modes (%s)", cam_id_, err2str(is_err));
        return is_err;
    }

    // Fetch sensor parameters
    is_err = is_GetSensorInfo(cam_handle_, &cam_sensor_info_);
    if (is_err != IS_SUCCESS)
    {
      DEBUGDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to get sensor info.");
        return is_err;
    }

    // // Validate camera's configuration to ensure compatibility with driver wrapper
    // // (note that this function also initializes the internal frame buffer)
    // if ((is_err = syncCamConfig()) != IS_SUCCESS) return is_err;

    DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Connected to [%s]", cam_name_.c_str());
    return is_err;
}

INT IDSCamera::disconnectCam()
{
    INT is_err = IS_SUCCESS;

    if (isConnected())
    {
        setStandbyMode();

        // Release existing camera buffers
        if (cam_buffer_ != nullptr)
        {
            is_err = is_FreeImageMem(cam_handle_, cam_buffer_, cam_buffer_id_);
        }
        cam_buffer_ = nullptr;

        // Release camera handle
        is_err      = is_ExitCamera(cam_handle_);
        cam_handle_ = HIDS(0);

        DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Disconnected from [%s]", cam_name_.c_str());
    }

    return is_err;
}


INT IDSCamera::setFlashParams(INT& delay_us, UINT& duration_us) {
  INT is_err = IS_SUCCESS;

  // Make sure parameters are within range supported by camera
  IO_FLASH_PARAMS minFlashParams, maxFlashParams, newFlashParams;
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_GET_PARAMS_MIN,
      (void*) &minFlashParams, sizeof(IO_FLASH_PARAMS))) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to retrieve flash parameter info (min) for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
    return is_err;
  }
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_GET_PARAMS_MAX,
      (void*) &maxFlashParams, sizeof(IO_FLASH_PARAMS))) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to retrieve flash parameter info (max) for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
    return is_err;
  }
  delay_us = (delay_us < minFlashParams.s32Delay) ? minFlashParams.s32Delay :
      ((delay_us > maxFlashParams.s32Delay) ? maxFlashParams.s32Delay : delay_us);
  duration_us = (duration_us < minFlashParams.u32Duration && duration_us != 0) ? minFlashParams.u32Duration :
      ((duration_us > maxFlashParams.u32Duration) ? maxFlashParams.u32Duration : duration_us);
  newFlashParams.s32Delay = delay_us;
  newFlashParams.u32Duration = duration_us;
  // WARNING: Setting s32Duration to 0, according to documentation, means
  //          setting duration to total exposure time. If non-ext-triggered
  //          camera is operating at fastest grab rate, then the resulting
  //          flash signal will APPEAR as active LO when set to active HIGH,
  //          and vice versa. This is why the duration is set manually.
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_SET_PARAMS,
      (void*) &newFlashParams, sizeof(IO_FLASH_PARAMS))) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to set flash parameter info for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
    return is_err;
  }

  return is_err;
}


INT IDSCamera::setGpioMode(const int& gpio, int& mode, double& pwm_freq, double& pwm_duty_cycle) {
  /* Set GPIO to a specific mode. */
  INT is_err = IS_SUCCESS;
  IO_GPIO_CONFIGURATION gpioConfiguration;
  gpioConfiguration.u32State = 0;
  IO_PWM_PARAMS m_pwmParams;
  
  if (gpio == 1) 
    gpioConfiguration.u32Gpio = IO_GPIO_1; // GPIO1 (pin 5).
  else if (gpio == 2) 
    gpioConfiguration.u32Gpio = IO_GPIO_2; // GPIO2 (pin 6).

  switch (mode) {
  case 0: gpioConfiguration.u32Configuration = IS_GPIO_INPUT; break;  
  case 1: gpioConfiguration.u32Configuration = IS_GPIO_OUTPUT; break;
  case 2:
          gpioConfiguration.u32Configuration = IS_GPIO_OUTPUT;
          gpioConfiguration.u32State = 1;
          break;
  case 3: gpioConfiguration.u32Configuration = IS_GPIO_FLASH; break;
  case 4:
          gpioConfiguration.u32Configuration = IS_GPIO_PWM;  
          m_pwmParams.dblFrequency_Hz = pwm_freq;
          m_pwmParams.dblDutyCycle = pwm_duty_cycle;
          break;
  case 5: gpioConfiguration.u32Configuration = IS_GPIO_TRIGGER;  break;
  }
  
  // set GPIO regarding the selected pind and mode
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_GPIOS_SET_CONFIGURATION, (void*)&gpioConfiguration, sizeof(gpioConfiguration))) != IS_SUCCESS) { 
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to set GPIO configuration for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
  }

  // Set PWM details if prior change of GPIO was successfull and mode is PWM 
  if ((is_err == IS_SUCCESS) && (mode == 4)) {
    if ((is_err = is_IO(cam_handle_, IS_IO_CMD_PWM_SET_PARAMS, (void*)&m_pwmParams, sizeof(m_pwmParams))) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to set PWM parameters for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
    }  
  }

  return is_err;
}


INT IDSCamera::setFreeRunMode() {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  if (!freeRunModeActive()) {
    setStandbyMode(); // No need to check for success

    // Set the flash to a high active pulse for each image in the trigger mode
    INT flash_delay = 0;
    UINT flash_duration = 1000;
    setFlashParams(flash_delay, flash_duration);
    UINT nMode = IO_FLASH_MODE_FREERUN_HI_ACTIVE;
    if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_SET_MODE,
        (void*) &nMode, sizeof(nMode))) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Could not set free-run active-high flash output for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
      DEBUGDEVICE(device, INDI::Logger::DBG_WARNING, "WARNING: camera hardware does not support ueye_cam's master-slave synchronization method");
    }
    IS_INIT_EVENT init_events[] = {{IS_SET_EVENT_FRAME, FALSE, FALSE}};
    if ((is_err = is_Event(cam_handle_, IS_EVENT_CMD_INIT, init_events, sizeof(init_events))) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not init frame event for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
      return is_err;
    }
    UINT events[] = {IS_SET_EVENT_FRAME};
    if ((is_err = is_Event(cam_handle_,IS_EVENT_CMD_ENABLE, events, sizeof(events))) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not enable frame event for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
      return is_err;
    }
    if ((is_err = is_CaptureVideo(cam_handle_, IS_WAIT)) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not start free-run live video mode for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
      return is_err;
    }
    DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Started free-run live video mode for [%s]", cam_name_.c_str());
  }

  return is_err;
}


INT IDSCamera::setExtTriggerMode() {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  if (!extTriggerModeActive()) {
    setStandbyMode(); // No need to check for success

    IS_INIT_EVENT init_events[] = {{IS_SET_EVENT_FRAME, FALSE, FALSE}};
    if ((is_err = is_Event(cam_handle_, IS_EVENT_CMD_INIT, init_events, sizeof(init_events))) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not init frame event for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
      return is_err;
    }
    UINT events[] = {IS_SET_EVENT_FRAME};
    if ((is_err = is_Event(cam_handle_,IS_EVENT_CMD_ENABLE, events, sizeof(events))) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not enable frame event for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
      return is_err;
    }

    if ((is_err = is_SetExternalTrigger(cam_handle_, IS_SET_TRIGGER_HI_LO)) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not enable rising-edge external trigger mode for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
      return is_err;
    }
    if ((is_err = is_CaptureVideo(cam_handle_, IS_DONT_WAIT)) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not start external trigger live video mode for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
      return is_err;
    }
    DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Started rising-edge external trigger live video mode for [%s]", cam_name_.c_str());
  }

  return is_err;
}

INT IDSCamera::setStandbyMode()
{
    if (!isConnected())
        return IS_INVALID_CAMERA_HANDLE;

    INT is_err = IS_SUCCESS;

    UINT events[] = { IS_SET_EVENT_FRAME };

    if (extTriggerModeActive())
    {
        if ((is_err = is_Event(cam_handle_, IS_EVENT_CMD_DISABLE, events, sizeof(events))) != IS_SUCCESS)
        {
          DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not disable frame event for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
            return is_err;
        }
        if ((is_err = is_Event(cam_handle_, IS_EVENT_CMD_EXIT, events, sizeof(events))) != IS_SUCCESS)
        {
          
            DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not exit frame event for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
            return is_err;
        }
        if ((is_err = is_SetExternalTrigger(cam_handle_, IS_SET_TRIGGER_OFF)) != IS_SUCCESS)
        {
            DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not disable external trigger mode for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
            return is_err;
        }
        is_SetExternalTrigger(
            cam_handle_,
            IS_GET_TRIGGER_STATUS); // documentation seems to suggest that this is needed to disable external trigger mode (to go into free-run mode)
        if ((is_err = is_StopLiveVideo(cam_handle_, IS_WAIT)) != IS_SUCCESS)
        {
            DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not stop live video mode for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
            return is_err;
        }
        DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Stopped external trigger mode for [%s]", cam_name_.c_str());
    }
    else if (freeRunModeActive())
    {
        UINT nMode = IO_FLASH_MODE_OFF;
        if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_SET_MODE, (void *)&nMode, sizeof(nMode))) != IS_SUCCESS)
        {
            DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not disable flash output for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
            return is_err;
        }
        if ((is_err = is_Event(cam_handle_, IS_EVENT_CMD_DISABLE, events, sizeof(events))) != IS_SUCCESS)
        {
            DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not disable frame event for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
            return is_err;
        }
        if ((is_err = is_Event(cam_handle_, IS_EVENT_CMD_EXIT, events, sizeof(events))) != IS_SUCCESS)
        {
            DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not exit frame event for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
            return is_err;
        }
        if ((is_err = is_StopLiveVideo(cam_handle_, IS_WAIT)) != IS_SUCCESS)
        {
            DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not stop live video mode for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
            return is_err;
        }
        DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Stopped free-run live video mode for [%s]", cam_name_.c_str());
    }
    if ((is_err = static_cast<int>(is_CameraStatus(cam_handle_, IS_STANDBY, IS_GET_STATUS))) != IS_SUCCESS)
    {
        DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not get setStandbyMode mode status for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
        return is_err;
    }

    return is_err;
}

bool IDSCamera::isSupportedColorMode(INT mode) {
    switch (mode) {
      case IS_CM_SENSOR_RAW8:
      case IS_CM_SENSOR_RAW10:
      case IS_CM_SENSOR_RAW12:
      case IS_CM_SENSOR_RAW16:
      case IS_CM_MONO8:
      case IS_CM_MONO10:
      case IS_CM_MONO12:
      case IS_CM_MONO16:
      case IS_CM_RGB8_PACKED:
      case IS_CM_BGR8_PACKED:
      case IS_CM_RGB8_PLANAR:
      case IS_CM_RGB10_PACKED:
      case IS_CM_BGR10_PACKED:
      case IS_CM_RGB10_UNPACKED:
      case IS_CM_BGR10_UNPACKED:
      case IS_CM_RGB12_UNPACKED:
      case IS_CM_BGR12_UNPACKED:
        return true;
  //    case IS_CM_BGR5_PACKED:
  //    case IS_CM_BGR565_PACKED:
  //    case IS_CM_UYVY_PACKED:
  //    case IS_CM_UYVY_MONO_PACKED:
  //    case IS_CM_UYVY_BAYER_PACKED:
  //    case IS_CM_CBYCRY_PACKED:
  //    case IS_CM_RGBA8_PACKED:
  //    case IS_CM_BGRA8_PACKED:
  //    case IS_CM_RGBY8_PACKED:
  //    case IS_CM_BGRY8_PACKED:
  //    case IS_CM_RGBA12_UNPACKED:
  //    case IS_CM_BGRA12_UNPACKED:
  //    case IS_CM_JPEG:
      default:
        return false;
    }
  }


INT IDSCamera::reallocateCamBuffer() {
  INT is_err = IS_SUCCESS;

  // Stop capture to prevent access to memory buffer
  setStandbyMode();

  // Free existing memory from previous calls to reallocateCamBuffer()
  if (cam_buffer_ != nullptr) {
    is_err = is_FreeImageMem(cam_handle_, cam_buffer_, cam_buffer_id_);
    cam_buffer_ = nullptr;
  }

  // Query camera's current resolution settings, for redundancy
  if ((is_err = is_AOI(cam_handle_, IS_AOI_IMAGE_GET_AOI,
      (void*) &cam_aoi_, sizeof(cam_aoi_))) != IS_SUCCESS) {
        DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not retrieve Area Of Interest (AOI) information for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
    return is_err;
  }
  
  // Allocate new memory section for IDS driver to use as frame buffer
  if ((is_err = is_AllocImageMem(cam_handle_, cam_aoi_.s32Width , cam_aoi_.s32Height,
    bits_per_pixel_, &cam_buffer_, &cam_buffer_id_)) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not allocate %d x %d image buffer for [%s] (%s)", cam_aoi_.s32Width, cam_aoi_.s32Height, cam_name_.c_str(), err2str(is_err));
    return is_err;
  }

  // Tell IDS driver to use allocated memory section as frame buffer
  if ((is_err = is_SetImageMem(cam_handle_, cam_buffer_, cam_buffer_id_)) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not associate image buffer to IDS driver for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
    return is_err;
  }

  // Synchronize internal settings for buffer step size and overall buffer size
  // NOTE: assume that sensor_scaling_rate, subsampling_rate, and cam_binning_rate_
  //       have all been previously validated and synchronized by syncCamConfig()
  if ((is_err = is_GetImageMemPitch(cam_handle_, &cam_buffer_pitch_)) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not query buffer step size / pitch / stride for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
    return is_err;
  }
  if (cam_buffer_pitch_ < cam_aoi_.s32Width * bits_per_pixel_/8) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Frame buffer's queried step size (%d) is smaller than buffer's expected stride [= width (%d) * bits per pixel (%d) /8] for [%s]\n(THIS IS A CODING ERROR, PLEASE CONTACT PACKAGE AUTHOR)", cam_buffer_pitch_, cam_aoi_.s32Width, bits_per_pixel_, cam_name_.c_str());
  }
  cam_buffer_size_ = static_cast<unsigned int>(cam_buffer_pitch_ * cam_aoi_.s32Height);

  // Report updated settings
  DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Allocated internal memory for [%s]:\n  buffer width: %d\n  buffer height: %d\n  buffer step/pitch/stride: %d\n  expected bits per pixel: %d\n  expected buffer size: %d",
    cam_name_.c_str(), cam_aoi_.s32Width, cam_aoi_.s32Height, cam_buffer_pitch_, bits_per_pixel_, cam_buffer_size_);
  return is_err;
}

const char* IDSCamera::processNextFrame(UINT timeout_ms) {
  if (!freeRunModeActive() && !extTriggerModeActive()) return nullptr;

  INT is_err = IS_SUCCESS;

  // Wait for frame event
  UINT events[] = {IS_SET_EVENT_FRAME};
  IS_WAIT_EVENTS wait_events = {events, 1, FALSE, timeout_ms, 0, 0};
  if ((is_err = is_Event(cam_handle_, IS_EVENT_CMD_WAIT, &wait_events, sizeof(wait_events))) != IS_SUCCESS) {
    if (is_err == IS_TIMED_OUT) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Timed out (%dms) while acquiring image from [%s] (%s)", timeout_ms, cam_name_.c_str(), err2str(is_err));
      DEBUGDEVICE(device, INDI::Logger::DBG_ERROR, "If this is occurring frequently, see https://github.com/anqixu/ueye_cam/issues/6#issuecomment-49925549");
      handleTimeout();
    } else {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to acquire image from [%s] (%s)", cam_name_.c_str(), err2str(is_err));
    }
    return nullptr;
  }

  return cam_buffer_;
}

long int IDSCamera::getFrameBufferSize() {
  return cam_buffer_size_;
}

bool IDSCamera::isColorModeSupported(std::string mode) {
  auto tmp_color_mode_ = name2colormode(mode);
  INT is_err = is_SetColorMode(cam_handle_, tmp_color_mode_);
  is_SetColorMode(cam_handle_, color_mode_);
  return is_err == IS_SUCCESS;
}

INT IDSCamera::setColorMode(std::string& mode, bool reallocate_buffer) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // Stop capture to prevent access to memory buffer
  setStandbyMode();

  // Set to specified color mode
  color_mode_ = name2colormode(mode);
  if (!isSupportedColorMode(color_mode_)) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Could not set color mode of [%s] to %s (not supported by this wrapper). Switching to default mode: mono8", cam_name_.c_str(), mode.c_str());
    color_mode_ = IS_CM_MONO8;
    mode = "mono8";
  }
  if ((is_err = is_SetColorMode(cam_handle_, color_mode_)) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not set color mode of [%s] to %s (%s: %d / '%s'). Switching to default mode: mono8", cam_name_.c_str(), mode.c_str(), err2str(is_err), color_mode_, mode.c_str());
        
    color_mode_ = IS_CM_MONO8;
    mode = "mono8";
    if ((is_err = is_SetColorMode(cam_handle_, color_mode_)) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not set color mode of [%s] to %s (%s: %d / '%s').", cam_name_.c_str(), mode.c_str(), err2str(is_err), color_mode_, mode.c_str());
      return is_err;
    }
  }
  bits_per_pixel_ = colormode2bpp(color_mode_);

  DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Updated color mode to %s for [%s]", mode.c_str(), cam_name_.c_str());

  return (reallocate_buffer ? reallocateCamBuffer() : IS_SUCCESS);
}

INT IDSCamera::setResolution(INT& image_width, INT& image_height, INT& image_left, INT& image_top, bool reallocate_buffer) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // Validate arguments
  CAP(image_width, 8, static_cast<INT>(cam_sensor_info_.nMaxWidth));
  CAP(image_height, 4, static_cast<INT>(cam_sensor_info_.nMaxHeight));
  if (image_left >= 0 && static_cast<int>(cam_sensor_info_.nMaxWidth) - image_width - image_left < 0) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Cannot set AOI left index to %d with a frame width of %d and sensor max width of %d for [%s]", image_left, image_width, cam_sensor_info_.nMaxWidth, cam_name_.c_str());
    image_left = -1;
  }
  if (image_top >= 0 &&
      static_cast<int>(cam_sensor_info_.nMaxHeight) - image_height - image_top < 0) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Cannot set AOI top index to %d with a frame height of %d and sensor max height of %d for [%s]", image_top, image_height, cam_sensor_info_.nMaxHeight, cam_name_.c_str());
    image_top = -1;
  }
  cam_aoi_.s32X = (image_left < 0) ?
      (cam_sensor_info_.nMaxWidth - static_cast<unsigned int>(image_width)) / 2 : image_left;
  cam_aoi_.s32Y = (image_top < 0) ?
      (cam_sensor_info_.nMaxHeight - static_cast<unsigned int>(image_height)) / 2 : image_top;
  cam_aoi_.s32Width = image_width;
  cam_aoi_.s32Height = image_height;

  const double s = cam_binning_rate_*cam_subsampling_rate_*cam_sensor_scaling_rate_;
  cam_aoi_.s32X /= s;
  cam_aoi_.s32Y /= s;
  cam_aoi_.s32Width /= s;
  cam_aoi_.s32Height /= s;

  if ((is_err = is_AOI(cam_handle_, IS_AOI_IMAGE_SET_AOI, &cam_aoi_, sizeof(cam_aoi_))) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to set Area Of Interest (AOI) to %d x %d with top-left corner at (%d, %d) for [%s]", cam_aoi_.s32Width, cam_aoi_.s32Height, cam_aoi_.s32X, cam_aoi_.s32Y, cam_name_.c_str());
    return is_err;
  }

  DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Updated Area Of Interest (AOI) to %d x %d with top-left corner at (%d, %d) for [%s]", cam_aoi_.s32Width, cam_aoi_.s32Height, cam_aoi_.s32X, cam_aoi_.s32Y, cam_name_.c_str());
  return (reallocate_buffer ? reallocateCamBuffer() : IS_SUCCESS);
}


INT IDSCamera::setSubsampling(int& rate, bool reallocate_buffer) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // Stop capture to prevent access to memory buffer
  setStandbyMode();

  INT rate_flag;
  INT supportedRates;

  supportedRates = is_SetSubSampling(cam_handle_, IS_GET_SUPPORTED_SUBSAMPLING);
  switch (rate) {
    case 1:
      rate_flag = IS_SUBSAMPLING_DISABLE;
      break;
    case 2:
      rate_flag = IS_SUBSAMPLING_2X;
      break;
    case 4:
      rate_flag = IS_SUBSAMPLING_4X;
      break;
    case 8:
      rate_flag = IS_SUBSAMPLING_8X;
      break;
    case 16:
      rate_flag = IS_SUBSAMPLING_16X;
      break;
    default:
      DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Currentlythe subsampling rate: %dX is not supported for [%s], resetting to 1X", rate, cam_name_.c_str());
      rate = 1;
      rate_flag = IS_SUBSAMPLING_DISABLE;
      break;
  }

  if ((supportedRates & rate_flag) == rate_flag) {
    if ((is_err = is_SetSubSampling(cam_handle_, rate_flag)) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not set subsampling rate to %dX for [%s] (%s)", rate, cam_name_.c_str(), err2str(is_err));
      return is_err;
    }
  } else {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Requested subsampling rate of %dX is not supported for [%s]", rate, cam_name_.c_str());

    // Query current rate
    INT currRate = is_SetSubSampling(cam_handle_, IS_GET_SUBSAMPLING);
    if (currRate == IS_SUBSAMPLING_DISABLE) { rate = 1; }
    else if (currRate == IS_SUBSAMPLING_2X) { rate = 2; }
    else if (currRate == IS_SUBSAMPLING_4X) { rate = 4; }
    else if (currRate == IS_SUBSAMPLING_8X) { rate = 8; }
    else if (currRate == IS_SUBSAMPLING_16X) { rate = 16; }
    else {
      DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Currently unsupported subsampling rate: %d for [%s], resetting to 1X", currRate, cam_name_.c_str());
      if ((is_err = is_SetSubSampling(cam_handle_, IS_SUBSAMPLING_DISABLE)) != IS_SUCCESS) {
        DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to set subsampling rate to 1X for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
        return is_err;
      }
    }
    return IS_SUCCESS;
  }

  DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Updated subsampling rate to %dX for [%s]", rate, cam_name_.c_str());

  cam_subsampling_rate_ = static_cast<unsigned int>(rate);

  return (reallocate_buffer ? reallocateCamBuffer() : IS_SUCCESS);
}

bool IDSCamera::isBinningSupported(int rate) {
  INT supportedRates = is_SetBinning(cam_handle_, IS_GET_SUPPORTED_BINNING);
  INT rate_flag = getBinFlag(rate);
  if (rate != 1 && rate_flag == IS_SUBSAMPLING_DISABLE)
    return false;
  else
    return (supportedRates & rate_flag) == rate_flag;
}

INT IDSCamera::setBinning(int& rate, bool reallocate_buffer) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // Stop capture to prevent access to memory buffer
  setStandbyMode();

  INT supportedRates = is_SetBinning(cam_handle_, IS_GET_SUPPORTED_BINNING);
  INT rate_flag = getBinFlag(rate);
  if (rate != 1 && rate_flag == IS_BINNING_DISABLE) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Requested binning rate of %dX is not supported for [%s], resetting to 1X", rate, cam_name_.c_str());
  }

  if ((supportedRates & rate_flag) == rate_flag) {
    if ((is_err = is_SetBinning(cam_handle_, rate_flag)) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Could not set binning rate to %dX for [%s] (%s)", rate, cam_name_.c_str(), err2str(is_err));
      return is_err;
    }
  } else {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Requested binning rate of %dX is not supported for [%s]", rate, cam_name_.c_str());

    // Query current rate
    INT currRate = is_SetBinning(cam_handle_, IS_GET_BINNING);
    if (currRate == IS_BINNING_DISABLE) { rate = 1; }
    else if (currRate == IS_BINNING_2X) { rate = 2; }
    else if (currRate == IS_BINNING_4X) { rate = 4; }
    else if (currRate == IS_BINNING_8X) { rate = 8; }
    else if (currRate == IS_BINNING_16X) { rate = 16; }
    else {
      DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Currently unsupported binning rate: %d for [%s], resetting to 1X", currRate, cam_name_.c_str());
      if ((is_err = is_SetBinning(cam_handle_, IS_BINNING_DISABLE)) != IS_SUCCESS) {
        DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to set binning rate to 1X for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
        return is_err;
      }
    }
    return IS_SUCCESS;
  }

  DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Updated binning rate to %dX for [%s]", rate, cam_name_.c_str());

  cam_binning_rate_ = static_cast<unsigned int>(rate);

  return (reallocate_buffer ? reallocateCamBuffer() : IS_SUCCESS);
}


INT IDSCamera::setSensorScaling(double& rate, bool reallocate_buffer) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // Stop capture to prevent access to memory buffer
  setStandbyMode();

  SENSORSCALERINFO sensorScalerInfo;
  is_err = is_GetSensorScalerInfo(cam_handle_, &sensorScalerInfo, sizeof(sensorScalerInfo));
  if (is_err == IS_NOT_SUPPORTED) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "[%s] does not support internal image scaling", cam_name_.c_str());
    rate = 1.0;
    cam_sensor_scaling_rate_ = 1.0;
    return IS_SUCCESS;
  } else if (is_err != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to obtain supported internal image scaling information for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
    rate = 1.0;
    cam_sensor_scaling_rate_ = 1.0;
    return is_err;
  } else {
    if (rate < sensorScalerInfo.dblMinFactor || rate > sensorScalerInfo.dblMaxFactor) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Requested internal image scaling rate of %f is not within supported bounds for [%s]: %f, %f; not updating current rate of %f", rate, cam_name_.c_str(), sensorScalerInfo.dblMinFactor, sensorScalerInfo.dblMaxFactor, sensorScalerInfo.dblCurrFactor);
      rate = sensorScalerInfo.dblCurrFactor;
      return IS_SUCCESS;
    }
  }

  if ((is_err = is_SetSensorScaler(cam_handle_, IS_ENABLE_SENSOR_SCALER, rate)) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Failed to set internal image scaling rate to %fX for [%s] (%s)", rate, cam_name_.c_str(), err2str(is_err));
    rate = 1.0;
    if ((is_err = is_SetSensorScaler(cam_handle_, IS_ENABLE_SENSOR_SCALER, rate)) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to reset internal image scaling rate to 1X for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
      return is_err;
    }
  }

  DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Updated internal image scaling rate to %fX for [%s]", rate, cam_name_.c_str());

  cam_sensor_scaling_rate_ = rate;

  return (reallocate_buffer ? reallocateCamBuffer() : IS_SUCCESS);
}


INT IDSCamera::setGain(bool& auto_gain, INT& master_gain_prc, INT& red_gain_prc,
    INT& green_gain_prc, INT& blue_gain_prc, bool& gain_boost) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // Validate arguments
  CAP(master_gain_prc, 0, 100);
  CAP(red_gain_prc, 0, 100);
  CAP(green_gain_prc, 0, 100);
  CAP(blue_gain_prc, 0, 100);

  double pval1 = 0, pval2 = 0;

  if (auto_gain) {
    // Set auto gain
    pval1 = 1;
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_SENSOR_GAIN,
        &pval1, &pval2)) != IS_SUCCESS) {
      if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_GAIN,
          &pval1, &pval2)) != IS_SUCCESS) {
        DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "[%s] does not support auto gain mode (%s)", cam_name_.c_str(), err2str(is_err));
        auto_gain = false;
      }
    }
  } else {
    // Disable auto gain
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_SENSOR_GAIN,
        &pval1, &pval2)) != IS_SUCCESS) {
      if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_GAIN,
          &pval1, &pval2)) != IS_SUCCESS) {
        DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "[%s] does not support auto gain mode (%s)", cam_name_.c_str(), err2str(is_err));
      }
    }

    // Set gain boost
    if (is_SetGainBoost(cam_handle_, IS_GET_SUPPORTED_GAINBOOST) != IS_SET_GAINBOOST_ON) {
      gain_boost = false;
    } else {
      if ((is_err = is_SetGainBoost(cam_handle_,
          (gain_boost) ? IS_SET_GAINBOOST_ON : IS_SET_GAINBOOST_OFF))
          != IS_SUCCESS) {
        std::string gain_boost_enabled = (gain_boost) ? "enable" : "disable";
        DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Failed to %s gain boost for [%s] (%s)", gain_boost_enabled.c_str(), cam_name_.c_str(), err2str(is_err));
      }
    }

    // Set manual gain parameters
    if ((is_err = is_SetHardwareGain(cam_handle_, master_gain_prc,
        red_gain_prc, green_gain_prc, blue_gain_prc)) != IS_SUCCESS) {
        // Set manual gain parameters
        if ((is_err = is_SetHardwareGain(cam_handle_, master_gain_prc, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER)) != IS_SUCCESS) {
            DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to set manual gains (master: %d; red: %d; green: %d; blue: %d) for [%s] (%s)", master_gain_prc, red_gain_prc, green_gain_prc, blue_gain_prc, cam_name_.c_str(), err2str(is_err));
        }
    }
  }

  if (auto_gain) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Updated gain for [%s]: auto", cam_name_.c_str());
  } else {
    DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Updated gain for [%s]: manual\n  master gain: %d\n  red gain: %d\n  green gain: %d\n  blue gain: %d\n  gain boost: %s",
      cam_name_.c_str(), master_gain_prc, red_gain_prc, green_gain_prc, blue_gain_prc, (gain_boost) ? "enabled" : "disabled");
  }

  return is_err;
}


INT IDSCamera::setSoftwareGamma(INT& software_gamma) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // According to ids this is only possible when the color mode is debayered by the ids driver 
  if ((color_mode_ == IS_CM_SENSOR_RAW8)  ||
      (color_mode_ == IS_CM_SENSOR_RAW10) ||
      (color_mode_ == IS_CM_SENSOR_RAW12) ||
      (color_mode_ == IS_CM_SENSOR_RAW16)) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Software gamma only possible when the color mode is debayered, could not set software gamma for [%s]", cam_name_.c_str());
    return IS_NO_SUCCESS;    
  }

  // set software gamma 
  if ((is_err = is_Gamma(cam_handle_, IS_GAMMA_CMD_SET, (void*) &software_gamma, sizeof(software_gamma))) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Software gamma could not be set for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
  }  

  return is_err;
}


INT IDSCamera::setExposure(bool& auto_exposure, double& auto_exposure_reference, double& exposure_ms) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  double minExposure = 1e-5, maxExposure = 500 * 1e3;

  // Set auto exposure
  double pval1 = auto_exposure, pval2 = 0;
  if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER,
      &pval1, &pval2)) != IS_SUCCESS) {
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_SHUTTER,
        &pval1, &pval2)) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Auto exposure mode is not supported for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
      auto_exposure = false;
    }
  }

  // Set exposure reference for auto exposure controller 
  if ((is_err = is_SetAutoParameter (cam_handle_, IS_SET_AUTO_REFERENCE, 
    &auto_exposure_reference, 0)) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Auto exposure reference value could not be set for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
    }

  // Set manual exposure timing
  if (!auto_exposure) {
    // Make sure that user-requested exposure rate is achievable
    if (((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN,
        (void*) &minExposure, sizeof(minExposure))) != IS_SUCCESS) ||
        ((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX,
        (void*) &maxExposure, sizeof(maxExposure))) != IS_SUCCESS)) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to query valid exposure range from [%s]", cam_name_.c_str());
      return is_err;
    }
    CAP(exposure_ms, minExposure, maxExposure);

    // Update exposure
    if ((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_SET_EXPOSURE,
        (void*) &(exposure_ms), sizeof(exposure_ms))) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to set exposure to %f ms for [%s] (%s)", exposure_ms, cam_name_.c_str(), err2str(is_err));
      return is_err;
    }
  }

  DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Updated exposure: %s for [%s]",
    ((auto_exposure) ? "auto" : std::to_string(exposure_ms).c_str()), cam_name_.c_str());

  return is_err;
}


INT IDSCamera::setWhiteBalance(bool& auto_white_balance, INT& red_offset, INT& blue_offset) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  CAP(red_offset, -50, 50);
  CAP(blue_offset, -50, 50);

  // Set auto white balance mode and parameters
  double pval1 = auto_white_balance, pval2 = 0;
  // TODO: 9 bug: enabling auto white balance does not seem to have an effect; in ueyedemo it seems to change R/G/B gains automatically
  if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE,
      &pval1, &pval2)) != IS_SUCCESS) {
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_AUTO_WB_ONCE,
        &pval1, &pval2)) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Auto white balance mode is not supported for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
      auto_white_balance = false;
    }
  }
  if (auto_white_balance) {
    pval1 = red_offset;
    pval2 = blue_offset;
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_SET_AUTO_WB_OFFSET,
        &pval1, &pval2)) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Auto white balance offsets could not be set for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
    }
  }

  DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Updated white balance for [%s]: %s\n  red offset: %d\n  blue offset: %d",
    cam_name_.c_str(), ((auto_white_balance) ? "auto" : "manual"), red_offset, blue_offset);
  return is_err;
}

INT IDSCamera::setFrameRate(double& rate) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  double new_fps = 0;
  if ((is_err = is_SetFrameRate(cam_handle_, rate, &new_fps)) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Could not set frame rate to %f [%s] (%s)", rate, cam_name_.c_str(), err2str(is_err));
  }

  DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Updated frame rate for [%s]:  fps: %f", cam_name_.c_str(), new_fps);
  return is_err;
}

void IDSCamera::getSensorInfo(double *pixel_size, int *max_width, int *max_height) {
  *pixel_size = double(cam_sensor_info_.wPixelSize) / 100.0;
  *max_height = cam_sensor_info_.nMaxHeight;
  *max_width = cam_sensor_info_.nMaxWidth;
}

std::string IDSCamera::getColorMode() {
  INT mode = is_SetColorMode(cam_handle_, IS_GET_COLOR_MODE);
  return colormode2name(mode);
}

INT IDSCamera::getSensorColorMode() {
  return cam_sensor_info_.nColorMode;
}


INT IDSCamera::getGain(bool& auto_gain, INT& master_gain_prc, INT& red_gain_prc,
  INT& green_gain_prc, INT& blue_gain_prc, bool& gain_boost) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  double pval1 = 0, pval2 = 0;
  // Get auto gain
  if ((is_err = is_SetAutoParameter(cam_handle_, IS_GET_ENABLE_AUTO_SENSOR_GAIN,
      &pval1, &pval2)) != IS_SUCCESS) {
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_GET_ENABLE_AUTO_GAIN,
        &pval1, &pval2)) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "[%s] does not support auto gain mode (%s)", cam_name_.c_str(), err2str(is_err));
      auto_gain = false;
    }
  }
  if (pval1 > 0)
    auto_gain = true;

  // Get gain boost
  if (is_SetGainBoost(cam_handle_, IS_GET_SUPPORTED_GAINBOOST) == IS_SET_GAINBOOST_ON && is_SetGainBoost(cam_handle_, IS_GET_GAINBOOST) == IS_SET_GAINBOOST_ON) {
    gain_boost = true;
  } else {
    gain_boost = false;
  }

  // Set manual gain parameters
  master_gain_prc = is_SetHardwareGain(cam_handle_, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  red_gain_prc = is_SetHardwareGain(cam_handle_, IS_GET_RED_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  green_gain_prc = is_SetHardwareGain(cam_handle_, IS_GET_GREEN_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  blue_gain_prc = is_SetHardwareGain(cam_handle_, IS_GET_BLUE_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  
  DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Got gain settings for [%s]: auto_gain: %s\n manual\n  master gain: %d\n  red gain: %d\n  green gain: %d\n  blue gain: %d\n  gain boost: %s",
    cam_name_.c_str(), (auto_gain) ? "enabled" : "disabled", master_gain_prc, red_gain_prc, green_gain_prc, blue_gain_prc, (gain_boost) ? "enabled" : "disabled");
  return is_err;
}

INT IDSCamera::getMinMaxExposure(double *minExposure, double *maxExposure) {
  INT is_err = IS_SUCCESS;
  // Make sure that user-requested exposure rate is achievable
  if (((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN,
      (void*) minExposure, sizeof(*minExposure))) != IS_SUCCESS) ||
      ((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX,
      (void*) maxExposure, sizeof(*maxExposure))) != IS_SUCCESS)) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to query valid exposure range from [%s]", cam_name_.c_str());
    return is_err;
  }
  return is_err;
}

INT IDSCamera::getMinMaxLongExposure(double *minExposure, double *maxExposure) {
  INT is_err = IS_SUCCESS;
  // Make sure that user-requested exposure rate is achievable
  if (((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_LONG_EXPOSURE_RANGE_MIN,
      (void*) minExposure, sizeof(*minExposure))) != IS_SUCCESS) ||
      ((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_LONG_EXPOSURE_RANGE_MAX,
      (void*) maxExposure, sizeof(*maxExposure))) != IS_SUCCESS)) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to query valid exposure range from [%s]", cam_name_.c_str());
    return is_err;
  }
  return is_err;
}


INT IDSCamera::getResolution(INT &image_width, INT &image_height, INT &image_left, INT &image_top) {
  INT is_err = IS_SUCCESS;
  // image_width, image_height, cam_aoi_.s32X, cam_aoi_.s32Y, cam_name_.c_str()
  if ((is_err = is_AOI(cam_handle_, IS_AOI_IMAGE_GET_AOI, &cam_aoi_, sizeof(cam_aoi_))) != IS_SUCCESS) {
    DEBUGDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to get Area Of Interest (AOI)");
    return is_err;
  }
  image_height = cam_aoi_.s32Height;
  image_width = cam_aoi_.s32Width;
  image_left = cam_aoi_.s32X;
  image_top = cam_aoi_.s32Y;
  
  DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Area Of Interest (AOI) is %d x %d with top-left corner at (%d, %d) for [%s]", image_width, image_height, cam_aoi_.s32X, cam_aoi_.s32Y, cam_name_.c_str());
  return is_err;
}

INT IDSCamera::getExposure(double& exposure_ms) {
    if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;
  
    INT is_err = IS_SUCCESS;
      // Get exposure
    if ((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE, (void*) &(exposure_ms), sizeof(exposure_ms))) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_ERROR, "Failed to get exposure to %f ms for [%s] (%s)", exposure_ms, cam_name_.c_str(), err2str(is_err));
      return is_err;
    }

    return is_err;
}

INT IDSCamera::getWhiteBalance(bool& auto_white_balance, INT& red_offset, INT& blue_offset) {
  if (!isConnected()) return IS_INVALID_CAMERA_HANDLE;

  INT is_err = IS_SUCCESS;

  // Set auto white balance mode and parameters
  double pval1 = 0, pval2 = 0;
  // TODO: 9 bug: enabling auto white balance does not seem to have an effect; in ueyedemo it seems to change R/G/B gains automatically
  if ((is_err = is_SetAutoParameter(cam_handle_, IS_GET_ENABLE_AUTO_SENSOR_WHITEBALANCE,
      &pval1, &pval2)) != IS_SUCCESS) {
    if ((is_err = is_SetAutoParameter(cam_handle_, IS_GET_AUTO_WB_ONCE,
        &pval1, &pval2)) != IS_SUCCESS) {
      DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Auto white balance mode is not supported for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
      auto_white_balance = false;
    }
  }
  if (pval1 > 0)
      auto_white_balance = true;

  if ((is_err = is_SetAutoParameter(cam_handle_, IS_GET_AUTO_WB_OFFSET,
      &pval1, &pval2)) != IS_SUCCESS) {
    DEBUGFDEVICE(device, INDI::Logger::DBG_WARNING, "Auto white balance offsets could not be set for [%s] (%s)", cam_name_.c_str(), err2str(is_err));
  } else {
    red_offset = pval1;
    blue_offset = pval2;
  }

  DEBUGFDEVICE(device, INDI::Logger::DBG_DEBUG, "Got white balance for [%s]: %s\n  red offset: %d\n  blue offset: %d",
    cam_name_.c_str(), ((auto_white_balance) ? "auto" : "manual"), red_offset, blue_offset);
  return is_err;
}
