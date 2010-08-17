/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup Camera Sensor Drivers
 */

/*!
 * @file mt9v111.h
 *
 * @brief MT9V111 Camera Header file
 *
 * This header file contains defines and structures for the iMagic mi8012
 * aka the Micron mt9v111 camera.
 *
 * @ingroup Camera
 */

#ifndef MT9V111_H_
#define MT9V111_H_

/*!
 * Basic camera values
 */
#define MT9V111_FRAME_RATE        30
#define MT9V111_MCLK              27000000 /* Desired clock rate */
#define MT9V111_CLK_MIN           12000000 /* This clock rate yields 15 fps */
#define MT9V111_CLK_MAX           27000000
#define MT9V111_MAX_WIDTH         640      /* Max width for this camera */
#define MT9V111_MAX_HEIGHT        480      /* Max height for this camera */

/*!
 * mt9v111 IFP REGISTER BANK MAP
 */
#define MT9V111I_ADDR_SPACE_SEL           0x1
#define MT9V111I_BASE_MAXTRIX_SIGN        0x2
#define MT9V111I_BASE_MAXTRIX_SCALE15     0x3
#define MT9V111I_BASE_MAXTRIX_SCALE69     0x4
#define MT9V111I_APERTURE_GAIN            0x5
#define MT9V111I_MODE_CONTROL             0x6
#define MT9V111I_SOFT_RESET               0x7
#define MT9V111I_FORMAT_CONTROL           0x8
#define MT9V111I_BASE_MATRIX_CFK1         0x9
#define MT9V111I_BASE_MATRIX_CFK2         0xa
#define MT9V111I_BASE_MATRIX_CFK3         0xb
#define MT9V111I_BASE_MATRIX_CFK4         0xc
#define MT9V111I_BASE_MATRIX_CFK5         0xd
#define MT9V111I_BASE_MATRIX_CFK6         0xe
#define MT9V111I_BASE_MATRIX_CFK7         0xf
#define MT9V111I_BASE_MATRIX_CFK8         0x10
#define MT9V111I_BASE_MATRIX_CFK9         0x11
#define MT9V111I_AWB_POSITION             0x12
#define MT9V111I_AWB_RED_GAIN             0x13
#define MT9V111I_AWB_BLUE_GAIN            0x14
#define MT9V111I_DELTA_MATRIX_CF_SIGN     0x15
#define MT9V111I_DELTA_MATRIX_CF_D1       0x16
#define MT9V111I_DELTA_MATRIX_CF_D2       0x17
#define MT9V111I_DELTA_MATRIX_CF_D3       0x18
#define MT9V111I_DELTA_MATRIX_CF_D4       0x19
#define MT9V111I_DELTA_MATRIX_CF_D5       0x1a
#define MT9V111I_DELTA_MATRIX_CF_D6       0x1b
#define MT9V111I_DELTA_MATRIX_CF_D7       0x1c
#define MT9V111I_DELTA_MATRIX_CF_D8       0x1d
#define MT9V111I_DELTA_MATRIX_CF_D9       0x1e
#define MT9V111I_LUMINANCE_LIMIT_WB       0x20
#define MT9V111I_RBG_MANUUAL_WB           0x21
#define MT9V111I_AWB_RED_LIMIT            0x22
#define MT9V111I_AWB_BLUE_LIMIT           0x23
#define MT9V111I_MATRIX_ADJUST_LIMIT      0x24
#define MT9V111I_AWB_SPEED                0x25
#define MT9V111I_H_BOUND_AE               0x26
#define MT9V111I_V_BOUND_AE               0x27
#define MT9V111I_H_BOUND_AE_CEN_WIN       0x2b
#define MT9V111I_V_BOUND_AE_CEN_WIN       0x2c
#define MT9V111I_BOUND_AWB_WIN            0x2d
#define MT9V111I_AE_PRECISION_TARGET      0x2e
#define MT9V111I_AE_SPEED                 0x2f
#define MT9V111I_RED_AWB_MEASURE          0x30
#define MT9V111I_LUMA_AWB_MEASURE         0x31
#define MT9V111I_BLUE_AWB_MEASURE         0x32
#define MT9V111I_LIMIT_SHARP_SATU_CTRL    0x33
#define MT9V111I_LUMA_OFFSET              0x34
#define MT9V111I_CLIP_LIMIT_OUTPUT_LUMI   0x35
#define MT9V111I_GAIN_LIMIT_AE            0x36
#define MT9V111I_SHUTTER_WIDTH_LIMIT_AE   0x37
#define MT9V111I_UPPER_SHUTTER_DELAY_LIM  0x39
#define MT9V111I_OUTPUT_FORMAT_CTRL2      0x3a
#define MT9V111I_IPF_BLACK_LEVEL_SUB      0x3b
#define MT9V111I_IPF_BLACK_LEVEL_ADD      0x3c
#define MT9V111I_ADC_LIMIT_AE_ADJ         0x3d
#define MT9V111I_GAIN_THRE_CCAM_ADJ       0x3e
#define MT9V111I_LINEAR_AE                0x3f
#define MT9V111I_THRESHOLD_EDGE_DEFECT    0x47
#define MT9V111I_LUMA_SUM_MEASURE         0x4c
#define MT9V111I_TIME_ADV_SUM_LUMA        0x4d
#define MT9V111I_MOTION                   0x52
#define MT9V111I_GAMMA_KNEE_Y12           0x53
#define MT9V111I_GAMMA_KNEE_Y34           0x54
#define MT9V111I_GAMMA_KNEE_Y56           0x55
#define MT9V111I_GAMMA_KNEE_Y78           0x56
#define MT9V111I_GAMMA_KNEE_Y90           0x57
#define MT9V111I_GAMMA_VALUE_Y0           0x58
#define MT9V111I_SHUTTER_60               0x59
#define MT9V111I_SEARCH_FLICK_60          0x5c
#define MT9V111I_RATIO_IMAGE_GAIN_BASE    0x5e
#define MT9V111I_RATIO_IMAGE_GAIN_DELTA   0x5f
#define MT9V111I_SIGN_VALUE_REG5F         0x60
#define MT9V111I_AE_GAIN                  0x62
#define MT9V111I_MAX_GAIN_AE              0x67
#define MT9V111I_LENS_CORRECT_CTRL        0x80
#define MT9V111I_SHADING_PARAMETER1       0x81
#define MT9V111I_SHADING_PARAMETER2       0x82
#define MT9V111I_SHADING_PARAMETER3       0x83
#define MT9V111I_SHADING_PARAMETER4       0x84
#define MT9V111I_SHADING_PARAMETER5       0x85
#define MT9V111I_SHADING_PARAMETER6       0x86
#define MT9V111I_SHADING_PARAMETER7       0x87
#define MT9V111I_SHADING_PARAMETER8       0x88
#define MT9V111I_SHADING_PARAMETER9       0x89
#define MT9V111I_SHADING_PARAMETER10      0x8A
#define MT9V111I_SHADING_PARAMETER11      0x8B
#define MT9V111I_SHADING_PARAMETER12      0x8C
#define MT9V111I_SHADING_PARAMETER13      0x8D
#define MT9V111I_SHADING_PARAMETER14      0x8E
#define MT9V111I_SHADING_PARAMETER15      0x8F
#define MT9V111I_SHADING_PARAMETER16      0x90
#define MT9V111I_SHADING_PARAMETER17      0x91
#define MT9V111I_SHADING_PARAMETER18      0x92
#define MT9V111I_SHADING_PARAMETER19      0x93
#define MT9V111I_SHADING_PARAMETER20      0x94
#define MT9V111I_SHADING_PARAMETER21      0x95
#define MT9V111i_FLASH_CTRL               0x98
#define MT9V111i_LINE_COUNTER             0x99
#define MT9V111i_FRAME_COUNTER            0x9A
#define MT9V111i_H_PAN                    0xA5
#define MT9V111i_H_ZOOM                   0xA6
#define MT9V111i_H_SIZE                   0xA7
#define MT9V111i_V_PAN                    0xA8
#define MT9V111i_V_ZOOM                   0xA9
#define MT9V111i_V_SIZE                   0xAA

#define MT9V111I_SEL_IFP                  0x1
#define MT9V111I_SEL_SCA                  0x4
#define MT9V111I_FC_RGB_OR_YUV            0x1000

/*!
 * Mt9v111 SENSOR CORE REGISTER BANK MAP
 */
#define MT9V111S_ADDR_SPACE_SEL           0x1
#define MT9V111S_COLUMN_START             0x2
#define MT9V111S_WIN_HEIGHT               0x3
#define MT9V111S_WIN_WIDTH                0x4
#define MT9V111S_HOR_BLANKING             0x5
#define MT9V111S_VER_BLANKING             0x6
#define MT9V111S_OUTPUT_CTRL              0x7
#define MT9V111S_ROW_START                0x8
#define MT9V111S_SHUTTER_WIDTH            0x9
#define MT9V111S_PIXEL_CLOCK_SPEED        0xa
#define MT9V111S_RESTART                  0xb
#define MT9V111S_SHUTTER_DELAY            0xc
#define MT9V111S_RESET                    0xd
#define MT9V111S_COLUMN_START_IN_ZOOM     0x12
#define MT9V111S_ROW_START_IN_ZOOM        0x13
#define MT9V111S_DIGITAL_ZOOM             0x1e
#define MT9V111S_READ_MODE                0x20
#define MT9V111S_DAC_CTRL                 0x27
#define MT9V111S_GREEN1_GAIN              0x2b
#define MT9V111S_BLUE_GAIN                0x2c
#define MT9V111S_READ_GAIN                0x2d
#define MT9V111S_GREEN2_GAIN              0x2e
#define MT9V111S_ROW_NOISE_CTRL           0x30
#define MT9V111S_DARK_TARGET_W            0x31
#define MT9V111S_TEST_DATA                0x32
#define MT9V111S_GLOBAL_GAIN              0x35
#define MT9V111S_SENSOR_CORE_VERSION      0x36
#define MT9V111S_DARK_TARGET_WO           0x37
#define MT9V111S_VERF_DAC                 0x41
#define MT9V111S_VCM_VCL                  0x42
#define MT9V111S_DISABLE_BYPASS           0x58
#define MT9V111S_CALIB_MEAN_TEST          0x59
#define MT9V111S_DARK_G1_AVE              0x5B
#define MT9V111S_DARK_G2_AVE              0x5C
#define MT9V111S_DARK_R_AVE               0x5D
#define MT9V111S_DARK_B_AVE               0x5E
#define MT9V111S_CAL_THRESHOLD            0x5f
#define MT9V111S_CAL_G1                   0x60
#define MT9V111S_CAL_G2                   0x61
#define MT9V111S_CAL_CTRL                 0x62
#define MT9V111S_CAL_R                    0x63
#define MT9V111S_CAL_B                    0x64
#define MT9V111S_CHIP_ENABLE              0xF1
#define MT9V111S_CHIP_VERSION             0xFF

/* OUTPUT_CTRL */
#define MT9V111S_OUTCTRL_SYNC             0x1
#define MT9V111S_OUTCTRL_CHIP_ENABLE      0x2
#define MT9V111S_OUTCTRL_TEST_MODE        0x40

/* READ_MODE */
#define MT9V111S_RM_NOBADFRAME            0x1
#define MT9V111S_RM_NODESTRUCT            0x2
#define MT9V111S_RM_COLUMNSKIP            0x4
#define MT9V111S_RM_ROWSKIP               0x8
#define MT9V111S_RM_BOOSTEDRESET          0x1000
#define MT9V111S_RM_COLUMN_LATE           0x10
#define MT9V111S_RM_ROW_LATE              0x80
#define MT9V111S_RM_RIGTH_TO_LEFT         0x4000
#define MT9V111S_RM_BOTTOM_TO_TOP         0x8000

/*! I2C Slave Address */
#define MT9V111_I2C_ADDRESS	0x48

/*!
 * The image resolution enum for the mt9v111 sensor
 */
typedef enum {
	MT9V111_OutputResolution_VGA = 0,	/*!< VGA size */
	MT9V111_OutputResolution_QVGA,	/*!< QVGA size */
	MT9V111_OutputResolution_CIF,	/*!< CIF size */
	MT9V111_OutputResolution_QCIF,	/*!< QCIF size */
	MT9V111_OutputResolution_QQVGA,	/*!< QQVGA size */
	MT9V111_OutputResolution_SXGA	/*!< SXGA size */
} MT9V111_OutputResolution;

enum {
	MT9V111_WINWIDTH = 0x287,
	MT9V111_WINWIDTH_DEFAULT = 0x287,
	MT9V111_WINWIDTH_MIN = 0x9,

	MT9V111_WINHEIGHT = 0x1E7,
	MT9V111_WINHEIGHT_DEFAULT = 0x1E7,

	MT9V111_HORZBLANK_DEFAULT = 0x26,
	MT9V111_HORZBLANK_MIN = 0x9,
	MT9V111_HORZBLANK_MAX = 0x3FF,

	MT9V111_VERTBLANK_DEFAULT = 0x4,
	MT9V111_VERTBLANK_MIN = 0x3,
	MT9V111_VERTBLANK_MAX = 0xFFF,
};

/*!
 * Mt9v111 Core Register structure.
 */
typedef struct {
	u32 addressSelect;	/*!< select address bank for Core Register 0x4 */
	u32 columnStart;	/*!< Starting Column */
	u32 windowHeight;	/*!< Window Height */
	u32 windowWidth;	/*!< Window Width */
	u32 horizontalBlanking;	/*!< Horizontal Blank time, in pixels */
	u32 verticalBlanking;	/*!< Vertical Blank time, in pixels */
	u32 outputControl;	/*!< Register to control sensor output */
	u32 rowStart;		/*!< Starting Row */
	u32 shutterWidth;
	u32 pixelClockSpeed;	/*!< pixel date rate */
	u32 restart;		/*!< Abandon the readout of current frame */
	u32 shutterDelay;
	u32 reset;		/*!< reset the sensor to the default mode */
	u32 zoomColStart;	/*!< Column start in the Zoom mode */
	u32 zomRowStart;	/*!< Row start in the Zoom mode */
	u32 digitalZoom;	/*!< 1 means zoom by 2 */
	u32 readMode;		/*!< Readmode: aspects of the readout of the sensor */
	u32 dACStandbyControl;
	u32 green1Gain;		/*!< Gain Settings */
	u32 blueGain;
	u32 redGain;
	u32 green2Gain;
	u32 rowNoiseControl;
	u32 darkTargetwNC;
	u32 testData;		/*!< test mode */
	u32 globalGain;
	u32 chipVersion;
	u32 darkTargetwoNC;
	u32 vREFDACs;
	u32 vCMandVCL;
	u32 disableBypass;
	u32 calibMeanTest;
	u32 darkG1average;
	u32 darkG2average;
	u32 darkRaverage;
	u32 darkBaverage;
	u32 calibThreshold;
	u32 calibGreen1;
	u32 calibGreen2;
	u32 calibControl;
	u32 calibRed;
	u32 calibBlue;
	u32 chipEnable;		/*!< Image core Registers written by image flow processor */
} mt9v111_coreReg;

/*!
 * Mt9v111 IFP Register structure.
 */
typedef struct {
	u32 addrSpaceSel;	/*!< select address bank for Core Register 0x1 */
	u32 baseMaxtrixSign;	/*!< sign of coefficient for base color correction matrix */
	u32 baseMaxtrixScale15;	/*!< scaling of color correction coefficient K1-5 */
	u32 baseMaxtrixScale69;	/*!< scaling of color correction coefficient K6-9 */
	u32 apertureGain;	/*!< sharpening */
	u32 modeControl;	/*!< bit 7 CCIR656 sync codes are embedded in the image */
	u32 softReset;		/*!< Image processing mode: 1 reset mode, 0 operational mode */
	u32 formatControl;	/*!< bit12 1 for RGB565, 0 for YcrCb */
	u32 baseMatrixCfk1;	/*!< K1 Color correction coefficient */
	u32 baseMatrixCfk2;	/*!< K2 Color correction coefficient */
	u32 baseMatrixCfk3;	/*!< K3 Color correction coefficient */
	u32 baseMatrixCfk4;	/*!< K4 Color correction coefficient */
	u32 baseMatrixCfk5;	/*!< K5 Color correction coefficient */
	u32 baseMatrixCfk6;	/*!< K6 Color correction coefficient */
	u32 baseMatrixCfk7;	/*!< K7 Color correction coefficient */
	u32 baseMatrixCfk8;	/*!< K8 Color correction coefficient */
	u32 baseMatrixCfk9;	/*!< K9 Color correction coefficient */
	u32 awbPosition;	/*!< Current position of AWB color correction matrix */
	u32 awbRedGain;		/*!< Current value of AWB red channel gain */
	u32 awbBlueGain;	/*!< Current value of AWB blue channel gain */
	u32 deltaMatrixCFSign;	/*!< Sign of coefficients of delta color correction matrix register */
	u32 deltaMatrixCFD1;	/*!< D1 Delta coefficient */
	u32 deltaMatrixCFD2;	/*!< D2 Delta coefficient */
	u32 deltaMatrixCFD3;	/*!< D3 Delta coefficient */
	u32 deltaMatrixCFD4;	/*!< D4 Delta coefficient */
	u32 deltaMatrixCFD5;	/*!< D5 Delta coefficient */
	u32 deltaMatrixCFD6;	/*!< D6 Delta coefficient */
	u32 deltaMatrixCFD7;	/*!< D7 Delta coefficient */
	u32 deltaMatrixCFD8;	/*!< D8 Delta coefficient */
	u32 deltaMatrixCFD9;	/*!< D9 Delta coefficient */
	u32 lumLimitWB;		/*!< Luminance range of pixels considered in WB statistics */
	u32 RBGManualWB;	/*!< Red and Blue color channel gains for manual white balance */
	u32 awbRedLimit;	/*!< Limits on Red channel gain adjustment through AWB */
	u32 awbBlueLimit;	/*!< Limits on Blue channel gain adjustment through AWB */
	u32 matrixAdjLimit;	/*!< Limits on color correction matrix adjustment through AWB */
	u32 awbSpeed;		/*!< AWB speed and color saturation control */
	u32 HBoundAE;		/*!< Horizontal boundaries of AWB measurement window */
	u32 VBoundAE;		/*!< Vertical boundaries of AWB measurement window */
	u32 HBoundAECenWin;	/*!< Horizontal boundaries of AE measurement window for backlight compensation */
	u32 VBoundAECenWin;	/*!< Vertical boundaries of AE measurement window for backlight compensation */
	u32 boundAwbWin;	/*!< Boundaries of AWB measurement window */
	u32 AEPrecisionTarget;	/*!< Auto exposure target and precision control */
	u32 AESpeed;		/*!< AE speed and sensitivity control register */
	u32 redAWBMeasure;	/*!< Measure of the red channel value used by AWB */
	u32 lumaAWBMeasure;	/*!< Measure of the luminance channel value used by AWB */
	u32 blueAWBMeasure;	/*!< Measure of the blue channel value used by AWB */
	u32 limitSharpSatuCtrl;	/*!< Automatic control of sharpness and color saturation */
	u32 lumaOffset;		/*!< Luminance offset control (brightness control) */
	u32 clipLimitOutputLumi;	/*!< Clipping limits for output luminance */
	u32 gainLimitAE;	/*!< Imager gain limits for AE adjustment */
	u32 shutterWidthLimitAE;	/*!< Shutter width (exposure time) limits for AE adjustment */
	u32 upperShutterDelayLi;	/*!< Upper Shutter Delay Limit */
	u32 outputFormatCtrl2;	/*!< Output Format Control 2
				   00 = 16-bit RGB565.
				   01 = 15-bit RGB555.
				   10 = 12-bit RGB444x.
				   11 = 12-bit RGBx444.       */
	u32 ipfBlackLevelSub;	/*!< IFP black level subtraction */
	u32 ipfBlackLevelAdd;	/*!< IFP black level addition */
	u32 adcLimitAEAdj;	/*!< ADC limits for AE adjustment */
	u32 agimnThreCamAdj;	/*!< Gain threshold for CCM adjustment */
	u32 linearAE;
	u32 thresholdEdgeDefect;	/*!< Edge threshold for interpolation and defect correction */
	u32 lumaSumMeasure;	/*!< Luma measured by AE engine */
	u32 timeAdvSumLuma;	/*!< Time-averaged luminance value tracked by auto exposure */
	u32 motion;		/*!< 1 when motion is detected */
	u32 gammaKneeY12;	/*!< Gamma knee points Y1 and Y2 */
	u32 gammaKneeY34;	/*!< Gamma knee points Y3 and Y4 */
	u32 gammaKneeY56;	/*!< Gamma knee points Y5 and Y6 */
	u32 gammaKneeY78;	/*!< Gamma knee points Y7 and Y8 */
	u32 gammaKneeY90;	/*!< Gamma knee points Y9 and Y10 */
	u32 gammaKneeY0;	/*!< Gamma knee point Y0 */
	u32 shutter_width_60;
	u32 search_flicker_60;
	u32 ratioImageGainBase;
	u32 ratioImageGainDelta;
	u32 signValueReg5F;
	u32 aeGain;
	u32 maxGainAE;
	u32 lensCorrectCtrl;
	u32 shadingParameter1;	/*!< Shade Parameters */
	u32 shadingParameter2;
	u32 shadingParameter3;
	u32 shadingParameter4;
	u32 shadingParameter5;
	u32 shadingParameter6;
	u32 shadingParameter7;
	u32 shadingParameter8;
	u32 shadingParameter9;
	u32 shadingParameter10;
	u32 shadingParameter11;
	u32 shadingParameter12;
	u32 shadingParameter13;
	u32 shadingParameter14;
	u32 shadingParameter15;
	u32 shadingParameter16;
	u32 shadingParameter17;
	u32 shadingParameter18;
	u32 shadingParameter19;
	u32 shadingParameter20;
	u32 shadingParameter21;
	u32 flashCtrl;		/*!< Flash control */
	u32 lineCounter;	/*!< Line counter */
	u32 frameCounter;	/*!< Frame counter */
	u32 HPan;		/*!< Horizontal pan in decimation */
	u32 HZoom;		/*!< Horizontal zoom in decimation */
	u32 HSize;		/*!< Horizontal output size iIn decimation */
	u32 VPan;		/*!< Vertical pan in decimation */
	u32 VZoom;		/*!< Vertical zoom in decimation */
	u32 VSize;		/*!< Vertical output size in decimation */
} mt9v111_IFPReg;

/*!
 * mt9v111 Config structure
 */
typedef struct {
	mt9v111_coreReg *coreReg;	/*!< Sensor Core Register Bank */
	mt9v111_IFPReg *ifpReg;	/*!< IFP Register Bank */
} mt9v111_conf;

typedef struct {
	u8 index;
	u16 width;
	u16 height;
} mt9v111_image_format;

#endif				/* MT9V111_H_  */
