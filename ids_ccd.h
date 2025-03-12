/*
    Driver type: IDS Camera INDI Driver

    Copyright (C) 2025 Sebastian von der Thannen

*/

#pragma once

#include <indiccd.h>

#include "ids_driver.h"

#include <map>
#include <future>
#include <string>

class IDSCCD : public INDI::CCD
{
    public:
        IDSCCD(int hid_, std::string serial_);

        const char * getDefaultName() override;

        bool initProperties() override;
        bool updateProperties() override;

        bool Connect() override;
        bool Disconnect() override;

        bool StartExposure(float duration) override;
        bool AbortExposure() override;
        bool UpdateCCDFrame(int x, int y, int w, int h) override;

        // enable binning
        bool UpdateCCDBin(int hor, int ver) override;

        virtual bool ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n) override;
        virtual bool ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n) override;

        static void ExposureUpdate(void * vp);
        void ExposureUpdate();

    protected:
        // Misc.
        bool saveConfigItems(FILE * fp) override;
        void addFITSKeywords(INDI::CCDChip * targetChip, std::vector<INDI::FITSRecord> &fitsKeywords) override;
        void TimerHit() override;

        // Capture format
        bool SetCaptureFormat(uint8_t index) override;
        void initSwitch(INDI::PropertySwitch &switchSP, int n, const char **names, int set_on = 0);

        // Streaming
        bool StartStreaming() override;
        bool StopStreaming() override;
        void streamLiveView();

        std::mutex liveStreamMutex;
        bool m_RunLiveStream;

    private:
        double CalcTimeLeft();
        bool grabImage();

        int hid;
        std::string serial;
        char name[MAXINDIDEVICE];
        char model[MAXINDINAME];

        struct timeval ExpStart;
        double ExposureRequest;

        // ids driver
        IDSCamera::UniquePtr ids_camera;

        int timerID;
        bool isTemperatureSupported { false };

        // binning ?
        bool binning { false };
        std::vector<int> binModes {};
        std::vector<std::string> colorModes {};

        INDI::PropertySwitch ColorModeSP {0};
        
        enum
        {
            RedOffset = 0, BlueOffset
        };

        INDI::PropertySwitch AWBSwitch {0};
        INDI::PropertyNumber AWBSettings {2};

        INDI::PropertySwitch AutoGainSwitch {0};
        enum
        {
            MasterGain = 0, RGain, GGain, BGain
        };
        INDI::PropertyNumber GainNP {BGain+1};

        // Threading
        std::thread m_LiveViewThread;

        // std::map <uint8_t, uint8_t> m_CaptureFormatMap;

        static constexpr double MINUMUM_CAMERA_TEMPERATURE = -60.0;

        // Do not accept switches more than this
        static constexpr uint8_t MAX_SWITCHES = 200;

        friend void ::ISSnoopDevice(XMLEle * root);
        // friend void ::ISGetProperties(const char * dev);
        friend void ::ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int num);
        friend void ::ISNewText(const char * dev, const char * name, char * texts[], char * names[], int num);
        friend void ::ISNewNumber(const char * dev, const char * name, double values[], char * names[], int num);
        friend void ::ISNewBLOB(const char * dev, const char * name, int sizes[], int blobsizes[], char * blobs[],
                                char * formats[], char * names[], int n);
};

