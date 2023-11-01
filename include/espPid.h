#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#define strLogTag "PID"

// get default configuration from Kconfig
#define PID_COEFFS_TIME_BASED 0x1
#define PID_COEFFS_INDEP_BASED 0x0

#if CONFIG_PID_REL_TIME
#define PID_DEFAULT_BASE 0x1
#elif CONFIG_PID_REL_INDEPENDENT
#define PID_DEFAULT_BASE 0x0
#endif



class espPid
{
    public:
        espPid(nvs_handle_t * ptr_nvs_handle);
        void begin();
        void begin(float *);
        void begin(float *, float *);
        void activate(bool, bool, bool);
        void changePidCoeff(const float f_kp, const float f_ki, const float f_kd, const bool b_time_rel);
        void addOutputLimits(float f_lower_lim, float f_upper_lim);
        void changeTargetValue(float f_target_value);
        void changeOperatingRange(float f_operating_lower, float f_operating_higher);
        void compute();
        void compute(const float &, float &);

    private:
        float _fTargetValue;
        float _fSumIntegrator;
        float * _ptrActualValue;
        float * _ptrManipValue;
        float _fPropFactor;
        float _fIntFactor;
        float _fDiffFactor;
        int8_t _bTimeRelation;
        float _fLoLim;
        float _fUpLim;
        float _fOpLoLim;
        float _fOpUpLim;
        bool _bOpUpLimActive = false;
        bool _bOpLoLimActive = false;
        float _fErrDiff;
        float _fLastControlDev;
        nvs_handle_t * _ptrNvsHandle;
        unsigned long _iLastComputeMillis;
        int _iLinkMode;
        bool _bKpActivate;
        bool _bKiActivate;
        bool _bKdActivate;
        void _init(void);
        void _calcControlEquation();
};