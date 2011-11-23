/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mlcontrol.c 4324 2010-12-15 20:07:49Z kkeal $
 *
 * $Date: 2010-12-15 12:07:49 -0800 (Wed, 15 Dec 2010) $
 *
 * $Author: kkeal $
 *
 * $Revision: 4324 $
 *
 *******************************************************************************/

/**
 *  @defgroup ML_CONTROL
 *  @brief  Motion Library - Control Engine.
 *          The Control Library processes gyroscopes, accelerometers, and 
 *          compasses to provide control signals that can be used in user 
 *          interfaces.
 *          These signals can be used to manipulate objects such as documents,
 *          images, cursors, menus, etc.
 *
 *  @{
 *      @file mlcontrol.c
 *      @brief The Control Library.
 *
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#include "mltypes.h"
#include "mlinclude.h"
#include "mltypes.h"
#include "ml.h"
#include "mlos.h"
#include "mlsl.h"
#include "mldl.h"
#include "mlcontrol.h"
#include "dmpKey.h"
#include "mlstates.h"
#include "mlFIFO.h"
#include "string.h"

/* - Global Vars. - */
tMLCTRLParams mlCtrlParams = { 
    {
        MLCTRL_SENSITIVITY_0_DEFAULT,
        MLCTRL_SENSITIVITY_1_DEFAULT,
        MLCTRL_SENSITIVITY_2_DEFAULT,
        MLCTRL_SENSITIVITY_3_DEFAULT
    },                                              // sensitivity
    MLCTRL_FUNCTIONS_DEFAULT,                       // functions
    {
        MLCTRL_PARAMETER_ARRAY_0_DEFAULT,
        MLCTRL_PARAMETER_ARRAY_1_DEFAULT,
        MLCTRL_PARAMETER_ARRAY_2_DEFAULT,
        MLCTRL_PARAMETER_ARRAY_3_DEFAULT
    },                                              // parameterArray
    {
        MLCTRL_PARAMETER_AXIS_0_DEFAULT,
        MLCTRL_PARAMETER_AXIS_1_DEFAULT,
        MLCTRL_PARAMETER_AXIS_2_DEFAULT,
        MLCTRL_PARAMETER_AXIS_3_DEFAULT
    },                                              // parameterAxis
    {
        MLCTRL_GRID_THRESHOLD_0_DEFAULT,
        MLCTRL_GRID_THRESHOLD_1_DEFAULT,
        MLCTRL_GRID_THRESHOLD_2_DEFAULT,
        MLCTRL_GRID_THRESHOLD_3_DEFAULT
    },                                              // gridThreshold
    {
        MLCTRL_GRID_MAXIMUM_0_DEFAULT,
        MLCTRL_GRID_MAXIMUM_1_DEFAULT,
        MLCTRL_GRID_MAXIMUM_2_DEFAULT,
        MLCTRL_GRID_MAXIMUM_3_DEFAULT
    },                                              // gridMaximum
    MLCTRL_GRID_CALLBACK_DEFAULT                    // gridCallback
};                                                  

/* - Extern Vars. - */
tMLCTRLXData mlCtrlxData;
tMLCTRLParams mlCtrlParams;
extern const unsigned char *dmpConfig1;

/* -------------- */
/* - Functions. - */
/* -------------- */

/**
 *  @brief  MLSetControlSensitivity is used to set the sensitivity for a control
 *          signal.
 *
 *  @pre    MLDmpOpen() Must be called with MLDmpDefaultOpen() or 
 *          MLDmpPedometerStandAloneOpen().
 *
 *  @param controlSignal    Indicates which control signal is being modified.
 *                          Must be one of:
 *                          - ML_CONTROL_1,
 *                          - ML_CONTROL_2,
 *                          - ML_CONTROL_3 or
 *                          - ML_CONTROL_4.
 *
 *  @param sensitivity      The sensitivity of the control signal.
 *
 *  @return error code
 */
tMLError MLSetControlSensitivity(unsigned short controlSignal, long sensitivity)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[2];
    long finalSens = 0;
    tMLError result;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE; 

    finalSens = sensitivity * 100;
    if (finalSens > 16384) {
        finalSens = 16384;
    }
    regs[0] = (unsigned char)(finalSens/256);
    regs[1] = (unsigned char)(finalSens%256);
    switch (controlSignal) {
        case ML_CONTROL_1:
            result = MLDLSetMemoryMPU(KEY_D_0_224, 2, regs );
            ERROR_CHECK(result);
            mlCtrlParams.sensitivity[0] = (unsigned short)sensitivity;
            break;
        case ML_CONTROL_2:
            result = MLDLSetMemoryMPU(KEY_D_0_228, 2, regs );
            ERROR_CHECK(result);
            mlCtrlParams.sensitivity[1] = (unsigned short)sensitivity;
            break;
        case ML_CONTROL_3:
            result = MLDLSetMemoryMPU(KEY_D_0_232, 2, regs );
            ERROR_CHECK(result);
            mlCtrlParams.sensitivity[2] = (unsigned short)sensitivity;
            break;
        case ML_CONTROL_4:
            result = MLDLSetMemoryMPU(KEY_D_0_236, 2, regs );
            ERROR_CHECK(result);
            mlCtrlParams.sensitivity[3] = (unsigned short)sensitivity;
            break;
        default:
            break;
    }
    if (finalSens != sensitivity * 100) {
        return ML_ERROR_INVALID_PARAMETER;
    } else {
        return ML_SUCCESS;
    }
}


/**
 *  @brief  MLSetControlFunc allows the user to choose how the sensor data will
 *          be processed in order to provide a control parameter.
 *          MLSetControlFunc allows the user to choose which control functions
 *          will be incorporated in the sensor data processing.
 *          The control functions are:
 *          - ML_GRID
 *          Indicates that the user will be controlling a system that
 *          has discrete steps, such as icons, menu entries, pixels, etc.
 *          - ML_SMOOTH
 *          Indicates that noise from unintentional motion should be filtered out.
 *          - ML_DEAD_ZONE
 *          Indicates that a dead zone should be used, below which sensor
 *          data is set to zero.
 *          - ML_HYSTERESIS
 *          Indicates that, when ML_GRID is selected, hysteresis should
 *          be used to prevent the control signal from switching rapidly across
 *          elements of the grid.
 *
 *  @pre    MLDmpOpen() Must be called with MLDmpDefaultOpen() or 
 *          MLDmpPedometerStandAloneOpen().
 *
 *  @param  function    Indicates what functions will be used.
 *                      Can be a bitwise OR of several values.
 *
 *  @return Zero if the command is successful; an ML error code otherwise.
 */
tMLError MLSetControlFunc(unsigned short function)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[8] = { DINA06, DINA26,
                              DINA46, DINA66,
                              DINA0E, DINA2E,
                              DINA4E, DINA6E };
    unsigned char i;
    tMLError result;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    if ((function & ML_SMOOTH)==0) {
        for (i=0; i<8; i++) {
            regs[i] = DINA80+3;
        }
    }
    result = MLDLSetMemoryMPU(KEY_CFG_4, 8, regs );
    ERROR_CHECK(result);
    mlCtrlParams.functions = function;
    result = MLSetDeadZone();

    return result;
}


/**
 *  @brief  MLGetControlSignal is used to get the current control signal with
 *          high precision.
 *          MLGetControlSignal is used to acquire the current data of a control signal.
 *          If ML_GRID is being used, MLGetGridNumber will probably be preferrable.
 *
 *  @param  controlSignal   Indicates which control signal is being queried.
 *          Must be one of:
 *          - ML_CONTROL_1,
 *          - ML_CONTROL_2,
 *          - ML_CONTROL_3 or
 *          - ML_CONTROL_4.
 *
 *  @param  reset   Indicates whether the control signal should be reset to zero.
 *                  Options are ML_RESET or ML_NO_RESET
 *  @param  data    A pointer to the current control signal data.
 *
 *  @return Zero if the command is successful; an ML error code otherwise.
 */
tMLError MLGetControlSignal(unsigned short controlSignal, unsigned short reset, long *data)
{
    INVENSENSE_FUNC_START;

    if ( MLGetState() != ML_STATE_DMP_STARTED )
        return ML_ERROR_SM_IMPROPER_STATE; 
  
    switch (controlSignal) {
        case ML_CONTROL_1:
            *data = mlCtrlxData.controlInt[0];
            if (reset == ML_RESET) {
                mlCtrlxData.controlInt[0] = 0;
            }
            break;
        case ML_CONTROL_2:
            *data = mlCtrlxData.controlInt[1];
            if (reset == ML_RESET) {
                mlCtrlxData.controlInt[1] = 0;
            }
            break;
        case ML_CONTROL_3:
            *data = mlCtrlxData.controlInt[2];
            if (reset == ML_RESET) {
                mlCtrlxData.controlInt[2] = 0;
            }
            break;
        case ML_CONTROL_4:
            *data = mlCtrlxData.controlInt[3];
            if (reset == ML_RESET) {
                mlCtrlxData.controlInt[3] = 0;
            }
            break;
        default:
            break;
    }
    return ML_SUCCESS;
}

/**
 *  @brief  MLGetGridNum is used to get the current grid location for a certain
 *          control signal.
 *          MLGetGridNum is used to acquire the current grid location.
 *
 *  @pre    MLDmpOpen() Must be called with MLDmpDefaultOpen() or 
 *          MLDmpPedometerStandAloneOpen().
 *
 *  @param  controlSignal   Indicates which control signal is being queried.
 *          Must be one of:
 *          - ML_CONTROL_1,
 *          - ML_CONTROL_2,
 *          - ML_CONTROL_3 or
 *          - ML_CONTROL_4.
 *
 *  @param  reset   Indicates whether the control signal should be reset to zero.
 *                  Options are ML_RESET or ML_NO_RESET
 *  @param  data    A pointer to the current grid number.
 *
 *  @return Zero if the command is successful; an ML error code otherwise.
 */

tMLError MLGetGridNum(unsigned short controlSignal, unsigned short reset,long *data)
{
    INVENSENSE_FUNC_START;
    
    if ( MLGetState() != ML_STATE_DMP_STARTED )
        return ML_ERROR_SM_IMPROPER_STATE;  

    switch (controlSignal) {
        case ML_CONTROL_1:
            *data = mlCtrlxData.gridNum[0];
            if (reset == ML_RESET) {
                mlCtrlxData.gridNum[0] = 0;
            }
            break;
        case ML_CONTROL_2:
            *data = mlCtrlxData.gridNum[1];
            if (reset == ML_RESET) {
                mlCtrlxData.gridNum[1] = 0;
            }
            break;
        case ML_CONTROL_3:
            *data = mlCtrlxData.gridNum[2];
            if (reset == ML_RESET) {
                mlCtrlxData.gridNum[2] = 0;
            }
            break;
        case ML_CONTROL_4:
            *data = mlCtrlxData.gridNum[3];
            if (reset == ML_RESET) {
                mlCtrlxData.gridNum[3] = 0;
            }
            break;
        default:
            break;
    }

    return ML_SUCCESS;
}


/**
 *  @brief  MLSetGridThresh is used to set the grid size for a control signal.
 *          MLSetGridThresh is used to adjust the size of the grid being controlled.
 *  @param  controlSignal   Indicates which control signal is being modified.
 *                          Must be one of:
 *                          - ML_CONTROL_1,
 *                          - ML_CONTROL_2,
 *                          - ML_CONTROL_3 and
 *                          - ML_CONTROL_4.
 *  @param  threshold       The threshold of the control signal at which the grid
 *                          number will be incremented or decremented.
 *  @return Zero if the command is successful; an ML error code otherwise.
 */

tMLError MLSetGridThresh(unsigned short controlSignal, long threshold)
{
    INVENSENSE_FUNC_START;
    
    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;  
  

    switch (controlSignal) {
        case ML_CONTROL_1:
            mlCtrlParams.gridThreshold[0] = threshold;
            break;
        case ML_CONTROL_2:
            mlCtrlParams.gridThreshold[1] = threshold;
            break;
        case ML_CONTROL_3:
            mlCtrlParams.gridThreshold[2] = threshold;
            break;
        case ML_CONTROL_4:
            mlCtrlParams.gridThreshold[3] = threshold;
            break;
        default:
            return ML_ERROR_INVALID_PARAMETER;
            break;
    }

    return ML_SUCCESS;
}


/**
 *  @brief  MLSetGridMax is used to set the maximum grid number for a control signal.
 *          MLSetGridMax is used to adjust the maximum allowed grid number, above
 *          which the grid number will not be incremented.
 *          The minimum grid number is always zero.
 *
 *  @pre    MLDmpOpen() Must be called with MLDmpDefaultOpen() or 
 *          MLDmpPedometerStandAloneOpen().
 *
 *  @param controlSignal    Indicates which control signal is being modified.
 *                          Must be one of:
 *                          - ML_CONTROL_1,
 *                          - ML_CONTROL_2,
 *                          - ML_CONTROL_3 and
 *                          - ML_CONTROL_4.
 *
 *  @param  maximum         The maximum grid number for a control signal.
 *  @return Zero if the command is successful; an ML error code otherwise.
 */

tMLError MLSetGridMax(unsigned short controlSignal, long maximum)
{
    INVENSENSE_FUNC_START;

    if ( MLGetState() != ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;    

    switch (controlSignal) {
        case ML_CONTROL_1:
            mlCtrlParams.gridMaximum[0] = maximum;
            break;
        case ML_CONTROL_2:
            mlCtrlParams.gridMaximum[1] = maximum;
            break;
        case ML_CONTROL_3:
            mlCtrlParams.gridMaximum[2] = maximum;
            break;
        case ML_CONTROL_4:
            mlCtrlParams.gridMaximum[3] = maximum;
            break;
        default:
            return ML_ERROR_INVALID_PARAMETER;
            break;
    }

    return ML_SUCCESS;
}


/**
 *  @brief  GridCallback function pointer type, to be passed as argument of 
 *          MLSetGridCallback.
 *
 *  @param  controlSignal   Indicates which control signal crossed a grid threshold.
 *                          Must be one of:
 *                          - ML_CONTROL_1,
 *                          - ML_CONTROL_2,
 *                          - ML_CONTROL_3 and
 *                          - ML_CONTROL_4.
 *
 *  @param  gridNumber  An array of four numbers representing the grid number for each
 *                      control signal.
 *  @param  gridChange  An array of four numbers representing the change in grid number
 *                      for each control signal.
**/
typedef void (*fpGridCb)(unsigned short controlSignal, long *gridNum, long *gridChange);

/**
 *  @brief  MLSetGridCallback is used to register a callback function that
 *          will trigger when the grid location changes.
 *          MLSetGridCallback allows a user to define a callback function that will
 *          run when a control signal crosses a grid threshold.

 *  @pre    MLDmpOpen() Must be called with MLDmpDefaultOpen() or 
 *          MLDmpPedometerStandAloneOpen().  MLDmpStart must <b>NOT</b> have 
 *          been called.
 *
 *  @param  func    A user defined callback function
 *  @return Zero if the command is successful; an ML error code otherwise.
**/
tMLError MLSetGridCallback(fpGridCb func)
{
    INVENSENSE_FUNC_START;

    if ( MLGetState() != ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;   

    mlCtrlParams.gridCallback = func;
    return ML_SUCCESS;
}

/**
 *  @brief  MLSetControlData is used to assign physical parameters to control signals.
 *          MLSetControlData allows flexibility in assigning physical parameters to
 *          control signals. For example, the user is allowed to use raw gyroscope data
 *          as an input to the control algorithm.
 *          Alternatively, angular velocity can be used, which combines gyroscopes and
 *          accelerometers to provide a more robust physical parameter. Finally, angular
 *          velocity in world coordinates can be used, providing a control signal in
 *          which pitch and yaw are provided relative to gravity.
 *
 *  @pre    MLDmpOpen() Must be called with MLDmpDefaultOpen() or 
 *          MLDmpPedometerStandAloneOpen().
 *
 *  @param  controlSignal   Indicates which control signal is being modified.
 *                          Must be one of:
 *                          - ML_CONTROL_1,
 *                          - ML_CONTROL_2,
 *                          - ML_CONTROL_3 or
 *                          - ML_CONTROL_4.
 *
 *  @param  parameterArray   Indicates which parameter array is being assigned to a
 *                          control signal. Must be one of:
 *                          - ML_GYROS,
 *                          - ML_ANGULAR_VELOCITY, or
 *
 *  @param  parameterAxis   Indicates which axis of the parameter array will be used.
 *                          Must be:
 *                          - ML_ROLL,
 *                          - ML_PITCH, or
 *                          - ML_YAW.
 */

tMLError MLSetControlData(unsigned short controlSignal, unsigned short parameterArray, unsigned short parameterAxis)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[2] = {DINA80+10, DINA20};
    tMLError result;

    if ( MLGetState() != ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    if (parameterArray == ML_ANGULAR_VELOCITY) {
        regs[0] = DINA80+5;
        regs[1] = DINA00;
    }
    switch (controlSignal) {
        case ML_CONTROL_1:
            mlCtrlParams.parameterArray[0] = parameterArray;
            switch (parameterAxis) {
                case ML_PITCH:
                    regs[1] += 0x02;
                    mlCtrlParams.parameterAxis[0] = 0;
                    break;
                case ML_ROLL:
                    regs[1] = DINA22;
                    mlCtrlParams.parameterAxis[0] = 1;
                    break;
                case ML_YAW:
                    regs[1] = DINA42;
                    mlCtrlParams.parameterAxis[0] = 2;
                    break;
                default:
                    return ML_ERROR_INVALID_PARAMETER;
            }
            result = MLDLSetMemoryMPU(KEY_CFG_3, 2, regs );
            ERROR_CHECK(result);
            break;
        case ML_CONTROL_2:
            mlCtrlParams.parameterArray[1] = parameterArray;
            switch (parameterAxis) {
                case ML_PITCH:
                    regs[1] += DINA0E;
                    mlCtrlParams.parameterAxis[1] = 0;
                    break;
                case ML_ROLL:
                    regs[1] += DINA2E;
                    mlCtrlParams.parameterAxis[1] = 1;
                    break;
                case ML_YAW:
                    regs[1] += DINA4E;
                    mlCtrlParams.parameterAxis[1] = 2;
                    break;
                default:
                    return ML_ERROR_INVALID_PARAMETER;
            }
            result = MLDLSetMemoryMPU(KEY_CFG_3B, 2, regs );
            ERROR_CHECK(result);
            break;
        case ML_CONTROL_3:
            mlCtrlParams.parameterArray[2] = parameterArray;
            switch (parameterAxis) {
                case ML_PITCH:
                    regs[1] += DINA0E;
                    mlCtrlParams.parameterAxis[2] = 0;
                    break;
                case ML_ROLL:
                    regs[1] += DINA2E;
                    mlCtrlParams.parameterAxis[2] = 1;
                    break;
                case ML_YAW:
                    regs[1] += DINA4E;
                    mlCtrlParams.parameterAxis[2] = 2;
                    break;
                default:
                    return ML_ERROR_INVALID_PARAMETER;
            }
            result = MLDLSetMemoryMPU(KEY_CFG_3C, 2, regs );
            ERROR_CHECK(result);
            break;
        case ML_CONTROL_4:
            mlCtrlParams.parameterArray[3] = parameterArray;
            switch (parameterAxis) {
                case ML_PITCH:
                    regs[1] += DINA0E;
                    mlCtrlParams.parameterAxis[3] = 0;
                    break;
                case ML_ROLL:
                    regs[1] += DINA2E;
                    mlCtrlParams.parameterAxis[3] = 1;
                    break;
                case ML_YAW:
                    regs[1] += DINA4E;
                    mlCtrlParams.parameterAxis[3] = 2;
                    break;
                default:
                    return ML_ERROR_INVALID_PARAMETER;
            }
            result = MLDLSetMemoryMPU(KEY_CFG_3D, 2, regs );
            ERROR_CHECK(result);
            break;
        default:
            result = ML_ERROR_INVALID_PARAMETER;
            break;
    }
    return result;
}


/**
 *  @brief  MLGetControlData is used to get the current control data.
 *
 *  @pre    MLDmpOpen() Must be called with MLDmpDefaultOpen() or 
 *          MLDmpPedometerStandAloneOpen().
 *
 *  @param  controlSignal   Indicates which control signal is being queried.
 *                          Must be one of:
 *                          - ML_CONTROL_1,
 *                          - ML_CONTROL_2,
 *                          - ML_CONTROL_3 or
 *                          - ML_CONTROL_4.
 *
 *  @param  gridNum     A pointer to pass gridNum info back to the user.
 *  @param  gridChange  A pointer to pass gridChange info back to the user.
 *
 *  @return Zero if the command is successful; an ML error code otherwise.
 */

tMLError MLGetControlData(long *controlSignal, long *gridNum, long *gridChange)
{
    INVENSENSE_FUNC_START;
    int_fast8_t i=0;
    
    if ( MLGetState() != ML_STATE_DMP_STARTED )
        return ML_ERROR_SM_IMPROPER_STATE;

    for (i=0; i<4; i++) {
        controlSignal[i] = mlCtrlxData.controlInt[i];
        gridNum[i] = mlCtrlxData.gridNum[i];
        gridChange[i] = mlCtrlxData.gridChange[i];
    }
    return ML_SUCCESS;
}

/** 
 * @internal
 * @brief   Update the ML Control engine.  This function should be called 
 *          every time new data from the MPU becomes available.
 *          Control engine outputs are written to the mlCtrlxData data 
 *          structure.
 * @return  ML_SUCCESS or an error code.
**/
tMLError MLControlUpdate( tMLXData *mlxData )
{
    INVENSENSE_FUNC_START;
    unsigned char i;
    long gridTmp;
    long tmp;

    FIFOGetControlData( mlCtrlxData.mlGridNumDMP );
    if ((mlxData->mlEngineMask & ML_CONTROL)) {
        for (i=0; i<4; i++) {
            if (mlCtrlParams.functions & ML_GRID) {
                if (mlCtrlParams.functions & ML_HYSTERESIS) {
                    mlCtrlxData.mlGridNumDMP[i] += mlCtrlxData.gridNumOffset[i];
                }
                mlCtrlxData.mlGridNumDMP[i] = mlCtrlxData.mlGridNumDMP[i]/2 + 1073741824L;
                mlCtrlxData.controlInt[i] = (mlCtrlxData.mlGridNumDMP[i]%(128*mlCtrlParams.gridThreshold[i]))/128;
                gridTmp = mlCtrlxData.mlGridNumDMP[i]/(128*mlCtrlParams.gridThreshold[i]);
                tmp = 1+16777216L/mlCtrlParams.gridThreshold[i];
                mlCtrlxData.gridChange[i] = gridTmp-mlCtrlxData.lastGridNum[i];
                if (mlCtrlxData.gridChange[i]>tmp/2) {
                    mlCtrlxData.gridChange[i] = gridTmp-tmp-mlCtrlxData.lastGridNum[i];
                } else if (mlCtrlxData.gridChange[i]<-tmp/2) {
                    mlCtrlxData.gridChange[i] = gridTmp+tmp-mlCtrlxData.lastGridNum[i];
                }
                if ((mlCtrlParams.functions & ML_HYSTERESIS) && (mlCtrlxData.gridChange[i]!=0)) {
                    if (mlCtrlxData.gridChange[i]>0) {
                        mlCtrlxData.gridNumOffset[i] += 128*mlCtrlParams.gridThreshold[i];
                        mlCtrlxData.controlInt[i] = mlCtrlParams.gridThreshold[i]/2;
                    }
                    if (mlCtrlxData.gridChange[i]<0) {
                        mlCtrlxData.gridNumOffset[i] -= 128*mlCtrlParams.gridThreshold[i];
                        mlCtrlxData.controlInt[i] = mlCtrlParams.gridThreshold[i]/2;
                    }
                }
                mlCtrlxData.gridNum[i]+=mlCtrlxData.gridChange[i];
                if (mlCtrlxData.gridNum[i]>=mlCtrlParams.gridMaximum[i]) {
                    mlCtrlxData.gridNum[i] = mlCtrlParams.gridMaximum[i];
                    if (mlCtrlxData.controlInt[i]>=mlCtrlParams.gridThreshold[i]/2) {
                        mlCtrlxData.controlInt[i]=mlCtrlParams.gridThreshold[i]/2;
                    }
                } else if (mlCtrlxData.gridNum[i]<=0) {
                    mlCtrlxData.gridNum[i] = 0;
                    if (mlCtrlxData.controlInt[i]<mlCtrlParams.gridThreshold[i]/2) {
                        mlCtrlxData.controlInt[i]=mlCtrlParams.gridThreshold[i]/2;
                    }
                }
                mlCtrlxData.lastGridNum[i] = gridTmp;
                if ((mlCtrlParams.gridCallback) && (mlCtrlxData.gridChange[i]!=0)) {
                    mlCtrlParams.gridCallback((ML_CONTROL_1<<i), mlCtrlxData.gridNum, mlCtrlxData.gridChange);
                }

            } else {
                mlCtrlxData.controlInt[i] = mlCtrlxData.mlGridNumDMP[i];
            }

        }
    }

    return ML_SUCCESS;
}


/**
 * @brief Enables the ML_CONTROL engine.
 *
 * @note  This function replaces MLEnable(ML_CONTROL)
 *
 * @pre MLDmpOpen() with MLDmpDefaultOpen or MLDmpPedometerStandAlone() must 
 *      have been called.
 *
 * @return ML_SUCCESS or non-zero error code
 */
tMLError MLEnableControl(void)
{
    INVENSENSE_FUNC_START;
    
    if ( MLGetState() != ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    memset( &mlCtrlxData, 0, sizeof(mlCtrlxData) );

    RegisterHighRateProcess( MLControlUpdate ); // fixme, someone needs to send control data to the fifo
    mlxData.mlEngineMask |= ML_CONTROL;
    return ML_SUCCESS;
}



/**
 * @brief Disables the ML_CONTROL engine.
 *
 * @note  This function replaces MLDisable(ML_CONTROL)
 *
 * @pre MLDmpOpen() with MLDmpDefaultOpen or MLDmpPedometerStandAlone() must 
 *      have been called.
 *
 * @return ML_SUCCESS or non-zero error code
 */
tMLError MLDisableControl(void)
{
    INVENSENSE_FUNC_START;
    
    if ( MLGetState() < ML_STATE_DMP_STARTED )
        return ML_ERROR_SM_IMPROPER_STATE;    

    mlxData.mlEngineMask &= (~ML_CONTROL);
    return ML_SUCCESS;
}

/**
 * @}
 */

