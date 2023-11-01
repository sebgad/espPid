#include "espPid.h"

espPid::espPid(nvs_handle_t * ptr_nvs_handle){
  _ptrNvsHandle = ptr_nvs_handle;
}

void espPid::_init(void){
  /**
   * @brief 
   * 
   */

  size_t i_float_size = 4;
  size_t i_bool_size = 1;

  ESP_LOGI(strLogTag, "Initialize PID controller...");

  if (nvs_get_blob(*_ptrNvsHandle, "PROP_ACTIVATE", &_bKpActivate, &i_bool_size) != ESP_OK){
    #ifdef CONFIG_PID_PROP_ACTIVATE
      _bKpActivate = true;
    #else
      _bKpActivate = false;
    #endif
    nvs_set_blob(*_ptrNvsHandle, "PROP_ACTIVATE", &_bKpActivate, i_bool_size);
  }

  if (nvs_get_blob(*_ptrNvsHandle, "FAC_PROP", &_fPropFactor, &i_float_size) != ESP_OK){
    _fPropFactor = (float)CONFIG_PID_FACTOR_PROP_MULT / (float)CONFIG_PID_GLOBAL_DIVISOR;
    nvs_set_blob(*_ptrNvsHandle, "FAC_PROP", &_fPropFactor, i_float_size);
  }

  if (nvs_get_blob(*_ptrNvsHandle, "INT_ACTIVATE", &_bKiActivate, &i_bool_size) != ESP_OK){
    #ifdef CONFIG_PID_INT_ACTIVATE
      _bKiActivate = true;
    #else
      _bKiActivate = false;
    #endif
    nvs_set_blob(*_ptrNvsHandle, "INT_ACTIVATE", &_bKiActivate, i_bool_size);
  }
  
  if (nvs_get_blob(*_ptrNvsHandle, "FAC_INT", &_fIntFactor, &i_float_size) != ESP_OK){
    _fIntFactor = (float)CONFIG_PID_FACTOR_INT_MULT / (float)CONFIG_PID_GLOBAL_DIVISOR;
    nvs_set_blob(*_ptrNvsHandle, "FAC_INT", &_fIntFactor, i_float_size);
  }

  if (nvs_get_blob(*_ptrNvsHandle, "DIFF_ACTIVATE", &_bKdActivate, &i_bool_size) != ESP_OK){
    #ifdef CONFIG_PID_DIFF_ACTIVATE
      _bKdActivate = true;
    #else
      _bKdActivate = false;
    #endif
    nvs_set_blob(*_ptrNvsHandle, "DIFF_ACTIVATE", &_bKdActivate, i_bool_size);
  }

  if (nvs_get_blob(*_ptrNvsHandle, "FAC_DIFF", &_fDiffFactor, &i_float_size) != ESP_OK){
    _fDiffFactor = (float)CONFIG_PID_FACTOR_DIFF_MULT / (float)CONFIG_PID_GLOBAL_DIVISOR;
    nvs_set_blob(*_ptrNvsHandle, "FAC_DIFF", &_fDiffFactor, i_float_size);
  }

  if (nvs_get_i8(*_ptrNvsHandle, "FAC_TIME_REL", &_bTimeRelation) != ESP_OK){
    _bTimeRelation = PID_DEFAULT_BASE;
    nvs_set_i8(*_ptrNvsHandle, "FAC_TIME_REL", _bTimeRelation);
  }

  if (nvs_get_blob(*_ptrNvsHandle, "LO_LIM", &_fLoLim, &i_float_size) != ESP_OK){
    _fLoLim = (float)CONFIG_PID_LO_LIM_MULT / (float)CONFIG_PID_GLOBAL_DIVISOR;
    nvs_set_blob(*_ptrNvsHandle, "LO_LIM", &_fLoLim, i_float_size);
  }

  if (nvs_get_blob(*_ptrNvsHandle, "UP_LIM", &_fUpLim, &i_float_size) != ESP_OK){
    _fUpLim = (float)CONFIG_PID_UP_LIM_MULT / (float)CONFIG_PID_GLOBAL_DIVISOR;
    nvs_set_blob(*_ptrNvsHandle, "UP_LIM", &_fUpLim, i_float_size);
  }

  if (nvs_get_blob(*_ptrNvsHandle, "TGT_VAL", &_fTargetValue, &i_float_size) != ESP_OK){
    _fTargetValue = (float)CONFIG_PID_TARGET_VAL_MULT / (float)CONFIG_PID_GLOBAL_DIVISOR;
    nvs_set_blob(*_ptrNvsHandle, "TGT_VAL", &_fTargetValue, i_float_size);
  }

  if (nvs_get_blob(*_ptrNvsHandle, "OP_LO_LIM", &_fOpLoLim, &i_float_size) != ESP_OK){
    _fOpLoLim = (float)CONFIG_PID_OP_RNG_LO_LIM_MULT / (float)CONFIG_PID_GLOBAL_DIVISOR;
    nvs_set_blob(*_ptrNvsHandle, "OP_LO_LIM", &_fOpLoLim, i_float_size);
  }

  if (nvs_get_blob(*_ptrNvsHandle, "OP_HI_LIM", &_fOpUpLim, &i_float_size) != ESP_OK){
    _fOpUpLim = (float)CONFIG_PID_OP_RNG_UP_LIM_MULT / (float)CONFIG_PID_GLOBAL_DIVISOR;
    nvs_set_blob(*_ptrNvsHandle, "OP_HI_LIM", &_fOpUpLim, i_float_size);
  }

  if (nvs_commit(*_ptrNvsHandle) == ESP_OK) {
    ESP_LOGI(strLogTag, "Done.");
  }
}

void espPid::begin(){
    /** 
     * Start PID controler.
     * Actual value variable and manipulation value variable is initialized and not linked. 
     * Compute() method takes actual value and manipulation value as parameter.
     */
    
    _ptrActualValue = new float;
    _ptrManipValue = new float;
    
    _iLastComputeMillis = esp_timer_get_time();
    _iLinkMode = 1;
    _init();
}

void espPid::begin(float * ptr_actual_value){
    /** 
     * Start PID controler.
     * Actual value variable is linked, manipulation value variable is initialized and not linked. 
     * Compute() method takes the manipulation variale as parameter.
     * @param ptr_actual_value     Actual value or measurement value as pointer
    */
    _ptrActualValue = ptr_actual_value;
    _ptrManipValue = new float;
    _iLastComputeMillis = esp_timer_get_time();
    _iLinkMode = 2;
    _init();
}

void espPid::begin(float * ptr_actual_value, float * ptr_manip_value){
    /**
     * Start PID controler. 
     * Actual value variable and manipulation value variable is linked. Compute() method will directly change the output.
     * @param ptr_actual_value       Actual value or measurement as pointer
     * @param ptr_manip_value        Manipulation value as pointer
    */
    _ptrActualValue =  ptr_actual_value;
    _ptrManipValue = ptr_manip_value;
    _iLastComputeMillis = esp_timer_get_time();
    _iLinkMode = 3;
    _init();
}

void espPid::activate(bool b_kp_activate, bool b_ki_activate, bool b_kd_activate){
    /**
     * @brief Define which Parameters should be activated
     * 
     */
    size_t i_bool_size = 1;
    
    _bKpActivate = b_kp_activate;
    nvs_set_blob(*_ptrNvsHandle, "PROP_ACTIVATE", &_bKpActivate, i_bool_size);
    
    _bKiActivate = b_ki_activate;
    nvs_set_blob(*_ptrNvsHandle, "INT_ACTIVATE", &_bKiActivate, i_bool_size);

    _bKdActivate = b_kd_activate;
    nvs_set_blob(*_ptrNvsHandle, "DIFF_ACTIVATE", &_bKdActivate, i_bool_size);
    nvs_commit(*_ptrNvsHandle);
}

void espPid::changePidCoeff(const float f_kp, const float f_ki, const float f_kd, const bool b_time_rel){
  /**
   * @brief 
   * 
   */
  size_t i_float_size = 4;

  _fPropFactor = f_kp;
  nvs_set_blob(*_ptrNvsHandle, "FAC_PROP", &_fPropFactor, i_float_size);

  _fIntFactor = f_ki;
  nvs_set_blob(*_ptrNvsHandle, "FAC_INT", &_fIntFactor, i_float_size);

  _fDiffFactor = f_kd;
  nvs_set_blob(*_ptrNvsHandle, "FAC_DIFF", &_fDiffFactor, i_float_size);

  _bTimeRelation = b_time_rel;
  nvs_set_i8(*_ptrNvsHandle, "FAC_TIME_REL", (int8_t)b_time_rel);
  nvs_commit(*_ptrNvsHandle);
}

void espPid::addOutputLimits(float f_lower_lim, float f_upper_lim){
    /**
     * lower and upper limit
     * TODO param
     */
    
    size_t i_float_size = 4;
    _fLoLim = f_lower_lim;
    _fUpLim = f_upper_lim;
    nvs_set_blob(*_ptrNvsHandle, "UP_LIM", &_fLoLim, i_float_size);
    nvs_set_blob(*_ptrNvsHandle, "LO_LIM", &_fLoLim, i_float_size);
    nvs_commit(*_ptrNvsHandle);
}

void espPid::changeTargetValue(float f_target_value){
    /** TODO
     * 
     */
    size_t i_float_size = 4;
    _fTargetValue = f_target_value;
    nvs_set_blob(*_ptrNvsHandle, "TGT_VAL", &_fTargetValue, i_float_size);
    nvs_commit(*_ptrNvsHandle);
}

void espPid::changeOperatingRange(float f_operating_lower, float f_operating_higher) {
  /**
  * Set a threshold were the PID output is hard set to on when the actual value is lower f_tresh_on 
  * or hard set off when the actual value is higher f_tresh_off
  * @param f_thres_on     lower threshold to be used
  * @param f_thres_off    upper threshold to be used
  */
  size_t i_float_size = 4;
  _fOpLoLim = f_operating_lower;
  _fOpUpLim = f_operating_higher;
  nvs_set_blob(*_ptrNvsHandle, "OP_LO_LIM", &_fOpLoLim, i_float_size);
  nvs_set_blob(*_ptrNvsHandle, "OP_HI_LIM", &_fOpUpLim, i_float_size);
  nvs_commit(*_ptrNvsHandle);
}

void espPid::compute() {
    /** 
     * Compute and calculate controler equitation. Actual value variable and manipulation value variable have to 
     * be linked in advance.
     */
    
    _calcControlEquation();
}

void espPid::compute(const float & f_actual, float & f_manip){
    /**
     * Compute and calculate controler equitation. Actual value variable and manipulation value variable are NOT
     * linked in advance.
     * @param f_actual  Actual value variable
     * @param f_manip   Manipulation value variable
     */    

    *_ptrActualValue = f_actual;
    _calcControlEquation();
    f_manip = *_ptrManipValue;
}

void espPid::_calcControlEquation(){
    /**
     * Calculate control equitation.
     */

    float f_delta_sec;
    float f_control_deviation;
    float f_d_control_deviation;
    float f_int_based;
    float f_diff_based;

    if (_bTimeRelation == PID_COEFFS_TIME_BASED){
      // Kp/Tn = 1/Ti = Ki
      f_int_based = _fPropFactor / _fIntFactor;
      // Kp*Tv = Td = Kd
      f_diff_based = _fPropFactor * _fDiffFactor;
    } else {
      f_int_based =  _fIntFactor;
      f_diff_based = _fDiffFactor;
    }
    
    f_delta_sec = (float)(esp_timer_get_time() - _iLastComputeMillis)/1000.F;
    f_control_deviation = _fTargetValue - *_ptrActualValue; // error

    *_ptrManipValue = 0.F;
    
    // setValue = Kp* (error + 1/Ti * integal(error) * dt + Td * diff(error)/dt)
    if (_bKpActivate){
        // Proportional component of the controler
        *_ptrManipValue += _fPropFactor * f_control_deviation;


        if (_bKdActivate){
            // Differential component of the controler
            _fErrDiff = (f_control_deviation - _fLastControlDev); // deviation of error
            *_ptrManipValue += f_diff_based * _fErrDiff / f_delta_sec;
        }
    
        if (_bKiActivate){
            // Integral part of the controler
            f_d_control_deviation = f_delta_sec * f_control_deviation;
            *_ptrManipValue += f_int_based * (_fSumIntegrator + f_d_control_deviation);
            
            if ((*_ptrManipValue<_fUpLim) && (_fSumIntegrator+f_d_control_deviation>=_fLoLim)){
              // Antiwindup measure: only sum error if manipulation value is lower then upper limit and error sum is greater than 0 (no cooling possible)
              _fSumIntegrator += f_d_control_deviation; // integrate deviation over time
            }
        }
    }

    // Check lower and upper limit of manipulation variable
    if (*_ptrManipValue<_fLoLim) { *_ptrManipValue = _fLoLim; };
    if (*_ptrManipValue>_fUpLim) { *_ptrManipValue = _fUpLim; };

    // Check operating range
    if (( *_ptrActualValue < _fOpLoLim ) || ( *_ptrActualValue > _fOpUpLim )){ 
      *_ptrManipValue = _fLoLim; 
    };
    
    _fLastControlDev = f_control_deviation;
    _iLastComputeMillis = esp_timer_get_time();
}



