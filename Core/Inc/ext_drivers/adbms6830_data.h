/*
 * adbms6830.h
 *
 *  Created on: May 13, 2025
 *      Author: realb
 */

#ifndef INC_EXT_DRIVERS_ADBMS6830_DATA_H_
#define INC_EXT_DRIVERS_ADBMS6830_DATA_H_

#include "stm32f7xx_hal.h"

#define CELL 16                 /* Bms ic number of cell              */
#define AUX  12                 /* Bms ic number of Aux               */
#define RAUX 10                 /* Bms ic number of RAux              */
#define PWMA 12                 /* Bms ic number of PWMA              */
#define PWMB 4                  /* Bms ic number of PWMB              */
#define COMM 3                  /* GPIO communication comm reg        */
#define RSID 6                  /* Bms ic number of SID byte          */
#define TX_DATA 6               /* Bms tx data byte                   */
#define RX_DATA 8               /* Bms rx data byte                   */
#define RDCVALL_SIZE    34      /* RDCVALL data byte size             */
#define RDSALL_SIZE     34      /* RDSALL data byte size              */
#define RDACALL_SIZE    34      /* RDACALL data byte size             */
#define RDFCALL_SIZE    34      /* RDFCALL data byte size             */
#define RDCSALL_SIZE    66      /* RDCSALL data byte size             */
#define RDASALL_SIZE    70      /* RDASALL data byte size             */
#define RDACSALL_SIZE   66      /* RDACSALL data byte size            */

/* shared buffer */
#define ADBMS6830_BSIZ 512
unsigned char shared_buf[ADBMS6830_BSIZ] = {0};

/*!
*  \enum Loop Measurement ENABLED or DISABLED.
*/
typedef enum { DISABLED = 0X0, ENABLED = 0X1} LOOP_MEASURMENT;

/**************************************** CMDEnums *************************************************/
/*!
*  \enum GPIO CHANNEL
* CH: GPIO Channels.
*/
/* Channel selection */
typedef enum
{
  AUX_ALL = 0,
  GPIO1,
  GPIO2,
  GPIO3,
  GPIO4,
  GPIO5,
  GPIO6,
  GPIO7,
  GPIO8,
  GPIO9,
  GPIO10,
  VREF2,
  LD03V,
  LD05V,
  TEMP,
  V_POSTIVE_2_NAGATIVE,
  V_NAGATIVE,
  VR4K,
  VREF3
}CH;

/*!
*  \enum RD
* RD: Read Device.
*/
typedef enum { RD_OFF = 0X0, RD_ON = 0X1} RD;

/*!
*  \enum CONT
* CONT: Continuous or single measurement.
*/
/* Continuous or single measurement */
typedef enum { SINGLE = 0X0, CONTINUOUS = 0X1} CONT;

/*!
*  \enum OW_C_S
* OW_C_S: Open wire c/s.
*/
/* Open wire c/s adcs */
typedef enum { OW_OFF_ALL_CH = 0X0, OW_ON_EVEN_CH, OW_ON_ODD_CH, OW_ON_ALL_CH} OW_C_S;

/*!
*  \enum OW_AUX
* OW_AUX: Open wire Aux.
*/
/* Open wire AUX */
typedef enum { AUX_OW_OFF = 0X0, AUX_OW_ON = 0X1} OW_AUX;

/*!
*  \enum PUP
* PUP: Pull Down current during aux conversion.
*/
/* Pull Down current during aux conversion (if OW = 1) */
typedef enum { PUP_DOWN = 0X0, PUP_UP = 0X1 } PUP;

/*!
*  \enum DCP
* DCP: Discharge permitted.
*/
/* Discharge permitted */
typedef enum { DCP_OFF = 0X0, DCP_ON = 0X1} DCP;

/*!
*  \enum RSTF
* RSTF: Reset Filter.
*/
/* Reset filter */
typedef enum  { RSTF_OFF = 0x0, RSTF_ON = 0x1 } RSTF;

/*!
*  \enum ERR
* ERR: Inject error is spi read out.
*/
/* Inject error is spi read out */
typedef enum  { WITHOUT_ERR = 0x0, WITH_ERR = 0x1 } ERR;

/**************************************** Mem bits *************************************************/
/* Configuration Register A */

/*!
*  \enum REFON
* REFON: Refernece remains power up/down.
*/
/* Refernece remains power up/down */
typedef enum  { PWR_DOWN = 0x0, PWR_UP = 0x1 } REFON;

/*!
*  \enum CTH
* CTH: Comparison voltages threshold C vs S.
*/
/* Comparison voltages threshold C vs S*/
typedef enum
{
  CVT_5_1mV = 0,        /* 5.1mV                */
  CVT_8_1mV,            /* 8.1mV (Default)      */
  CVT_10_05mV,          /* 10.05mV              */
  CVT_15mV,             /* 15mV                 */
  CVT_22_5mV,           /* 22.5mV               */
  CVT_45mV,             /* 45mV                 */
  CVT_75mV,             /* 75mV                 */
  CVT_135mV,            /* 135mV                */
}CTH;

/*!
*  \enum FLAG_D
* FLAG_D: Fault flags.
*/
/* Fault flags */
typedef enum
{
  FLAG_D0 = 0,          /* Force oscillator counter fast */
  FLAG_D1,              /* Force oscillator counter slow */
  FLAG_D2,              /* Force Supply Error detection  */
  FLAG_D3,              /* FLAG_D[3]: 1--> Select Supply OV and delta detection, 0 --> Selects UV */
  FLAG_D4,              /* Set THSD */
  FLAG_D5,              /* Force Fuse ED */
  FLAG_D6,              /* Force Fuse MED */
  FLAG_D7,              /* Force TMODCHK  */
} FLAG_D;

typedef enum  { FLAG_CLR = 0x0, FLAG_SET = 0x1 } CFGA_FLAG;

/*!
*  \enum CL FLAG
* FLAG: Fault Clear.
*/
typedef enum  { CL_FLAG_CLR = 0x0, CL_FLAG_SET = 0x1 } FLAG;

/*!
*  \enum SOAKON
* SOAKON: Enables or disable soak time for all commands.
*/
/* Enables or disable soak time for all commands */
typedef enum  { SOAKON_CLR = 0x0, SOAKON_SET = 0x1 } SOAKON;


/* Open wire sokon time owa */
typedef enum  {OWA0 = 0x0, OWA1, OWA2, OWA3, OWA4, OWA5, OWA6, OWA7} OWA;

/*!
*  \enum OWRNG
* OWRNG: Set soak time range Long/Short.
*/
/* Set soak time range Long/Short */
typedef enum  { SHORT = 0x0, LONG = 0x1 } OWRNG;


/*!
*  \enum OW_TIME
* OW_TIME:Open Wire Soak times
*          For Aux commands. If OWRNG=0, Soak time = 2^(6 +OWA[2:0]) Clocks (32 us 4.1 ms)
*          For Aux commands. If OWRNG=1, Soak time = 2^(13+OWA[2:0]) Clocks (41 ms 524 ms)
*/
typedef enum  { TIME_32US_TO_4_1MS = 0x0, TIME_41MS_TO_524MS = 0x1 } OW_TIME;

/*!
*  \enum GPO
* GPO: GPO Pins.
*/
/* GPO Pins */
typedef enum
{
  GPO1 = 0,
  GPO2,
  GPO3,
  GPO4,
  GPO5,
  GPO6,
  GPO7,
  GPO8,
  GPO9,
  GPO10,
} GPO;

/*!
*  \enum GPIO
* GPIO: GPIO Pin Control.
*/
/* GPO Pin Control */
typedef enum  { GPO_CLR = 0x0, GPO_SET = 0x1 } CFGA_GPO;

/*!
*  \enum IIR_FPA
* IIR_FPA: IIR Filter Parameter.
*/
/* IIR Filter Parameter */
typedef enum
{
  IIR_FPA_OFF = 0,              /* Filter Disabled          */
  IIR_FPA2,                     /* 110   Hz -3dB Frequency  */
  IIR_FPA4,                     /* 45    Hz -3dB Frequency  */
  IIR_FPA8,                     /* 21    Hz -3dB Frequency  */
  IIR_FPA16,                    /* 10    Hz -3dB Frequency  */
  IIR_FPA32,                    /* 5     Hz -3dB Frequency  */
  IIR_FPA128,                   /* 1.25  Hz -3dB Frequency  */
  IIR_FPA256,                   /* 0.625 Hz -3dB Frequency  */
}IIR_FPA;

/*!
*  \enum COMM_BK
* COMM_BK: Communication Break.
*/
/* Communication Break */
typedef enum  { COMM_BK_OFF = 0x0, COMM_BK_ON = 0x1 } COMM_BK;

/*!
*  \enum SNAPSHOT
* SNAPSHOT: Snapshot.
*/
/* Snapshot */
typedef enum  { SNAP_OFF = 0x0, SNAP_ON = 0x1 } SNAPSHOT;

/* Configuration Register B */

/*!
*  \enum DTMEN
* DTMEN: Enable Dis-charge Timer Monitor.
*/
/* Enable Dis-charge Timer Monitor */
typedef enum  { DTMEN_OFF = 0x0, DTMEN_ON = 0x1 } DTMEN;

/*!
*  \enum DTRNG
* DTRNG: Discharge Timer Range Setting.
*/
/* Discharge Timer Range Setting */
typedef enum  { RANG_0_TO_63_MIN = 0x0, RANG_0_TO_16_8_HR = 0x1 } DTRNG;

/*!
*  \enum DCTO
* DCTO: DCTO timeout values.
*/
typedef enum
{
  DCTO_TIMEOUT = 0,
  TIME_1MIN_OR_0_26HR,
  TIME_2MIN_OR_0_53HR,
  TIME_3MIN_OR_0_8HR,
  TIME_4MIN_OR_1_06HR,
  TIME_5MIN_OR_1_33HR,
  TIME_6MIN_OR_1_6HR,
  /* If required more time out value add here */
} DCTO;

/*!
*  \enum PWM
* PWM: PWM Duty cycle.
*/
typedef enum
{
  PWM_0_0_PCT = 0,      /* 0.0%  (default) */
  PWM_6_6_PCT,          /* 6.6%            */
  PWM_13_2_PCT,         /* 13.2%           */
  PWM_19_8_PCT,         /* 19.8%           */
  PWM_26_4_PCT,         /* 26.4%           */
  PWM_33_0_PCT,         /* 33.0%           */
  PWM_39_6_PCT,         /* 39.6%           */
  PWM_46_2_PCT,         /* 46.2%           */
  PWM_52_8_PCT,         /* 52.8%           */
  PWM_59_4_PCT,         /* 59.4%           */
  PWM_66_0_PCT,         /* 66.0%           */
  PWM_72_6_PCT,         /* 72.6%           */
  PWM_79_2_PCT,         /* 79.2%           */
  PWM_85_8_PCT,         /* 85.8%           */
  PWM_92_4_PCT,         /* 92.4%           */
  PWM_100_0_PCT,        /* ~100.0%         */
} PWM_DUTY;

/*!
*  \enum DCC
* DCC: DCC bits.
*/
/* DCC bits */
typedef enum
{
  DCC1 = 0,
  DCC2,
  DCC3,
  DCC4,
  DCC5,
  DCC6,
  DCC7,
  DCC8,
  DCC9,
  DCC10,
  DCC11,
  DCC12,
  DCC13,
  DCC14,
  DCC15,
  DCC16,
} DCC;

/*!
*  \enum DCC_BIT
* DCC_BIT: Discharge cell set and claer.
*/
/* Discharge cell set and claer  */
typedef enum  { DCC_BIT_CLR = 0x0, DCC_BIT_SET = 0x1 } DCC_BIT;

/* General Enums */
typedef enum
{
	ALL_GRP = 0x0,
	A,
	B,
	C,
	D,
	E,
	F,
	NONE
} 	GRP;

typedef enum
{
	Cell = 0x0,
	Aux,
	RAux,
	Status,
	Pwm,
	AvgCell,
	S_volt,
	F_volt,
	Config,
	Comm,
	Sid,
	Clrflag,
	Rdcvall,
	Rdacall,
	Rdsall,
	Rdcsall,
	Rdacsall,
	Rdfcall,
	Rdasall
}   TYPE;

typedef enum { PASS , FAIL } RESULT;
typedef enum { OSC_MISMATCH = 0x0, SUPPLY_ERROR, THSD, FUSE_ED, FUSE_MED, TMODCHK} DIAGNOSTIC_TYPE;

/* configuration registers commands */
uint8_t WRCFGA[2]        = { 0x00, 0x01 };
uint8_t WRCFGB[2]        = { 0x00, 0x24 };
uint8_t RDCFGA[2]        = { 0x00, 0x02 };
uint8_t RDCFGB[2]        = { 0x00, 0x26 };

/* Read cell voltage result registers commands */
uint8_t RDCVA[2]         = { 0x00, 0x04 };
uint8_t RDCVB[2]         = { 0x00, 0x06 };
uint8_t RDCVC[2]         = { 0x00, 0x08 };
uint8_t RDCVD[2]         = { 0x00, 0x0A };
uint8_t RDCVE[2]         = { 0x00, 0x09 };
uint8_t RDCVF[2]         = { 0x00, 0x0B };
uint8_t RDCVALL[2]       = { 0x00, 0x0C };

/* Read average cell voltage result registers commands commands */
uint8_t RDACA[2]         = { 0x00, 0x44 };
uint8_t RDACB[2]         = { 0x00, 0x46 };
uint8_t RDACC[2]         = { 0x00, 0x48 };
uint8_t RDACD[2]         = { 0x00, 0x4A };
uint8_t RDACE[2]         = { 0x00, 0x49 };
uint8_t RDACF[2]         = { 0x00, 0x4B };
uint8_t RDACALL[2]       = { 0x00, 0x4C };

/* Read s voltage result registers commands */
uint8_t RDSVA[2]         = { 0x00, 0x03 };
uint8_t RDSVB[2]         = { 0x00, 0x05 };
uint8_t RDSVC[2]         = { 0x00, 0x07 };
uint8_t RDSVD[2]         = { 0x00, 0x0D };
uint8_t RDSVE[2]         = { 0x00, 0x0E };
uint8_t RDSVF[2]         = { 0x00, 0x0F };
uint8_t RDSALL[2]        = { 0x00, 0x10 };

/* Read c and s results */
uint8_t RDCSALL[2]       = { 0x00, 0x11 };
uint8_t RDACSALL[2]      = { 0x00, 0x51 };

/* Read all AUX and all Status Registers */
uint8_t RDASALL[2]       = { 0x00, 0x35 };

/* Read filtered cell voltage result registers*/
uint8_t RDFCA[2]         = { 0x00, 0x12 };
uint8_t RDFCB[2]         = { 0x00, 0x13 };
uint8_t RDFCC[2]         = { 0x00, 0x14 };
uint8_t RDFCD[2]         = { 0x00, 0x15 };
uint8_t RDFCE[2]         = { 0x00, 0x16 };
uint8_t RDFCF[2]         = { 0x00, 0x17 };
uint8_t RDFCALL[2]       = { 0x00, 0x18 };

/* Read aux results */
uint8_t RDAUXA[2]        = { 0x00, 0x19 };
uint8_t RDAUXB[2]        = { 0x00, 0x1A };
uint8_t RDAUXC[2]        = { 0x00, 0x1B };
uint8_t RDAUXD[2]        = { 0x00, 0x1F };

/* Read redundant aux results */
uint8_t RDRAXA[2]        = { 0x00, 0x1C };
uint8_t RDRAXB[2]        = { 0x00, 0x1D };
uint8_t RDRAXC[2]        = { 0x00, 0x1E };
uint8_t RDRAXD[2]        = { 0x00, 0x25 };

/* Read status registers */
uint8_t RDSTATA[2]       = { 0x00, 0x30 };
uint8_t RDSTATB[2]       = { 0x00, 0x31 };
uint8_t RDSTATC[2]       = { 0x00, 0x32 };
uint8_t RDSTATCERR[2]    = { 0x00, 0x72 };              /* ERR */
uint8_t RDSTATD[2]       = { 0x00, 0x33 };
uint8_t RDSTATE[2]       = { 0x00, 0x34 };

/* Pwm registers commands */
uint8_t WRPWM1[2]        = { 0x00, 0x20 };
uint8_t RDPWM1[2]        = { 0x00, 0x22 };

uint8_t WRPWM2[2]        = { 0x00, 0x21 };
uint8_t RDPWM2[2]        = { 0x00, 0x23 };

/* Clear commands */
uint8_t CLRCELL[2]       = { 0x07, 0x11 };
uint8_t CLRAUX [2]       = { 0x07, 0x12 };
uint8_t CLRSPIN[2]       = { 0x07, 0x16 };
uint8_t CLRFLAG[2]       = { 0x07, 0x17 };
uint8_t CLRFC[2]         = { 0x07, 0x14 };
uint8_t CLOVUV[2]        = { 0x07, 0x15 };

/* Poll adc command */
uint8_t PLADC[2]         = { 0x07, 0x18 };
uint8_t PLAUT[2]         = { 0x07, 0x19 };
uint8_t PLCADC[2]        = { 0x07, 0x1C };
uint8_t PLSADC[2]        = { 0x07, 0x1D };
uint8_t PLAUX1[2]        = { 0x07, 0x1E };
uint8_t PLAUX2[2]        = { 0x07, 0x1F };

/* Diagn command */
uint8_t DIAGN[2]         = {0x07 , 0x15};

/* GPIOs Comm commands */
uint8_t WRCOMM[2]        = { 0x07, 0x21 };
uint8_t RDCOMM[2]        = { 0x07, 0x22 };
uint8_t STCOMM[13]       = { 0x07, 0x23, 0xB9, 0xE4 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};

/* Mute and Unmute commands */
uint8_t MUTE[2]          = { 0x00, 0x28 };
uint8_t UNMUTE[2]        = { 0x00, 0x29 };

uint8_t RSTCC[2]         = { 0x00, 0x2E };
uint8_t SNAP[2]          = { 0x00, 0x2D };
uint8_t UNSNAP[2]        = { 0x00, 0x2F };
uint8_t SRST[2]          = { 0x00, 0x27 };

/* Read SID command */
uint8_t RDSID[2]         = { 0x00, 0x2C };

/* For ADBMS6830 config register structure */
typedef struct
{
  uint8_t       refon;
  uint8_t       cth;
  uint8_t       flag_d;
  uint8_t       soakon;
  uint8_t       owrng;
  uint8_t       owa;
  uint16_t      gpo;
  uint8_t       snap;
  uint8_t       mute_st;
  uint8_t       comm_bk;
  uint8_t       fc;
}cfa_;

/* For ADBMS6830 config register structure */
typedef struct
{
  uint16_t  vuv;
  uint16_t  vov;
  uint8_t   dtmen;
  uint8_t   dtrng;
  uint8_t   dcto;
  uint16_t  dcc;
}cfb_;

/* Cell Voltage Data structure */
typedef struct
{
  int16_t c_codes[CELL]; /* Cell Voltage Codes */
} cv_;

typedef struct
{
  int16_t ac_codes[CELL]; /* Average Cell Voltage Codes */
} acv_;

/* S Voltage Data structure */
typedef struct
{
  int16_t sc_codes[CELL]; /* S Voltage Codes */
} scv_;

/* Filtered Cell Voltage Data structure */
typedef struct
{
  int16_t fc_codes[CELL]; /* filtered Cell Voltage Codes */
} fcv_;

/* Aux Voltage Data Structure*/
typedef struct
{
  int16_t a_codes[AUX]; /* Aux Voltage Codes */
} ax_;

/* Redundant Aux Voltage Data Structure*/
typedef struct
{
  int16_t ra_codes[RAUX]; /* Aux Voltage Codes */
} rax_;

/* Status A register Data structure*/
typedef struct
{
  uint16_t  vref2;
  uint16_t  itmp;
  uint16_t  vref3;
} sta_;

/* Status B register Data structure*/
typedef struct
{
  uint16_t vd;
  uint16_t va;
  uint16_t vr4k;
} stb_;

/* Status C register Data structure*/
typedef struct
{
  uint16_t      cs_flt;
  uint8_t       va_ov;
  uint8_t       va_uv;
  uint8_t       vd_ov;
  uint8_t       vd_uv;
  uint8_t       otp1_ed;
  uint8_t       otp1_med;
  uint8_t       otp2_ed;
  uint8_t       otp2_med;
  uint8_t       vde;
  uint8_t       vdel;
  uint8_t       comp;
  uint8_t       spiflt;
  uint8_t       sleep;
  uint8_t       thsd;
  uint8_t       tmodchk;
  uint8_t       oscchk;
} stc_;

/* ClrFlag register Data structure*/
typedef struct
{
  uint16_t      cl_csflt;
  uint8_t       cl_smed;
  uint8_t       cl_sed;
  uint8_t       cl_cmed;
  uint8_t       cl_ced;
  uint8_t       cl_vduv;
  uint8_t       cl_vdov;
  uint8_t       cl_vauv;
  uint8_t       cl_vaov;
  uint8_t       cl_oscchk;
  uint8_t       cl_tmode;
  uint8_t       cl_thsd;
  uint8_t       cl_sleep;
  uint8_t       cl_spiflt;
  uint8_t       cl_vdel;
  uint8_t       cl_vde;
} clrflag_;

/* Status D register Data structure*/
typedef struct
{
  uint8_t c_ov[CELL];
  uint8_t c_uv[CELL];
  uint8_t ct;
  uint8_t cts;
  uint8_t oc_cntr;
} std_;

/* Status E register Data structure*/
typedef struct
{
  uint16_t gpi;
  uint8_t rev;
} ste_;

/* Pwm register Data structure*/
typedef struct
{
  uint8_t pwma[PWMA];
} pwma_;

/*PWMB Register Structure */
typedef struct
{
  uint8_t pwmb[PWMB];
} pwmb_;

/* COMM register Data structure*/
typedef struct
{
  uint8_t fcomm[COMM];
  uint8_t icomm[COMM];
  uint8_t data[COMM];
} com_;

/*SID Register Structure */
typedef struct
{
  uint8_t sid[RSID];
} sid_;

/* Transmit byte and recived byte data structure */
typedef struct
{
  uint8_t tx_data[TX_DATA];
  uint8_t rx_data[RX_DATA];
} ic_register_;

/* Command counter and pec error data Structure */
typedef struct
{
  uint8_t cmd_cntr;
  uint8_t cfgr_pec;
  uint8_t cell_pec;
  uint8_t acell_pec;
  uint8_t scell_pec;
  uint8_t fcell_pec;
  uint8_t aux_pec;
  uint8_t raux_pec;
  uint8_t stat_pec;
  uint8_t comm_pec;
  uint8_t pwm_pec;
  uint8_t sid_pec;
} cmdcnt_pec_;

/* Diagnostic test result data structure */
typedef struct
{
  uint8_t osc_mismatch;
  uint8_t supply_error;
  uint8_t supply_ovuv;
  uint8_t thsd;
  uint8_t fuse_ed;
  uint8_t fuse_med;
  uint8_t tmodchk;
  uint8_t cell_ow[CELL];
  uint8_t cellred_ow[CELL];
  uint8_t aux_ow[(AUX-2)];
} diag_test_;

/* Aux open wire data structure */
typedef struct
{
  int cell_ow_even[CELL];
  int cell_ow_odd[CELL];
} cell_ow_;

/* Aux open wire data structure */
typedef struct
{
  int aux_pup_up[(AUX-2)];
  int aux_pup_down[(AUX-2)];
} aux_ow_;

/* BMS ic main structure */
typedef struct
{
  cfa_ tx_cfga;
  cfa_ rx_cfga;
  cfb_ tx_cfgb;
  cfb_ rx_cfgb;
  clrflag_ clflag;
  cv_  cell;
  acv_ acell;
  scv_ scell;
  fcv_ fcell;
  ax_  aux;
  rax_ raux;
  sta_ stata;
  stb_ statb;
  stc_ statc;
  std_ statd;
  ste_ state;
  com_ comm;
  pwma_ PwmA;
  pwmb_ PwmB;
  sid_ sid;
  ic_register_ configa;
  ic_register_ configb;
  ic_register_ clrflag;
  ic_register_ stat;
  ic_register_ com;
  ic_register_ pwma;
  ic_register_ pwmb;
  ic_register_ rsid;
  cmdcnt_pec_ cccrc;
  aux_ow_ gpio;
  cell_ow_ owcell;
  diag_test_ diag_result;
} adbms6830_asic;

/* adc command config - holds the command configuration for the adbms */
typedef struct
{
  RD REDUNDANT_MEASUREMENT;
  CH AUX_CH_TO_CONVERT;
  CONT CONTINUOUS_MEASUREMENT;
  OW_C_S CELL_OPEN_WIRE_DETECTION;
  OW_AUX AUX_OPEN_WIRE_DETECTION;
  PUP OPEN_WIRE_CURRENT_SOURCE;
  DCP DISCHARGE_PERMITTED;
  RSTF RESET_FILTER;
  ERR  INJECT_ERR_SPI_READ;
} adc_command_config_t;

typedef struct
{
  float OV_THRESHOLD; /* Volt */
  float UV_THRESHOLD; /* Volt */
  int OWC_Threshold; /* Cell Open wire threshold(mili volt) */
  int OWA_Threshold; /* Aux Open wire threshold(mili volt) */
} threshold_config_t;

/* loop_manager_t - holds loop measurement variables
* LOOP_MEASUREMENT variables are either ENABLED or DISABLED (all caps)
*/
typedef struct
{
  uint32_t LOOP_MEASUREMENT_COUNT;
  uint16_t MEASUREMENT_LOOP_TIME;
  uint32_t loop_count;
  uint32_t pladc_count;
  LOOP_MEASURMENT MEASURE_CELL;
  LOOP_MEASURMENT MEASURE_AVG_CELL;
  LOOP_MEASURMENT MEASURE_F_CELL;
  LOOP_MEASURMENT MEASURE_S_VOLTAGE;
  LOOP_MEASURMENT MEASURE_AUX;
  LOOP_MEASURMENT MEASURE_RAUX;
  LOOP_MEASURMENT MEASURE_STAT;
} loop_manager_t;

typedef struct
{
  int num_ics;
  adbms6830_asic *ics;
  SPI_HandleTypeDef *hspi_a;
  SPI_HandleTypeDef *hspi_b;
  GPIO_TypeDef *cs_port_a;
  GPIO_TypeDef *cs_port_b;
  uint16_t cs_pin_a;
  uint16_t cs_pin_b;
  adc_command_config_t adc_config;
  threshold_config_t thresholds;
  loop_manager_t loop_manager;
} adbms6830_driver_t;

#endif /* INC_EXT_DRIVERS_ADBMS6830_DATA_H_ */
