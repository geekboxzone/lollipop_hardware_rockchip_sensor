/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
#ifndef DMPMAP_H
#define DMPMAP_H

#ifdef __cplusplus
extern "C"
{
#endif

#define DMP_PTAT    0
#define DMP_XGYR    2
#define DMP_YGYR    4
#define DMP_ZGYR    6
#define DMP_XACC    8
#define DMP_YACC    10
#define DMP_ZACC    12
#define DMP_ADC1    14
#define DMP_ADC2    16
#define DMP_ADC3    18
#define DMP_FOOTER    30
#define DMP_0_H    32
#define DMP_0_L    34
#define DMP_0_h    36
#define DMP_1_l    38
#define DMP_0_5H    40
#define DMP_0_5L    42
#define DMP_ASR16H    44
#define DMP_ASR16L    46
#define DMP_LP_XH    52
#define DMP_LP_XL    54
#define DMP_LP_YH    56
#define DMP_LP_YL    58
#define DMP_LP_ZH    60
#define DMP_LP_ZL    62
#define DMP_TMPX    64
#define DMP_ZEROX    66
#define DMP_TMPY    68
#define DMP_ZEROY    70
#define DMP_TEMPZ    72
#define DMP_ZEROZ    74
#define DMP_SYNCH    76
#define DMP_SYNCL    78
#define DMP_HPX_H    84
#define DMP_HPX_L    86
#define DMP_HPY_H    88
#define DMP_HPY_L    90
#define DMP_HPZ_H    92
#define DMP_HPZ_L    94
#define DMP_TMPXAH    100
#define DMP_TMPXAL    102
#define DMP_TMPYAH    104
#define DMP_TMPYAL    106
#define DMP_TMPZAH    108
#define DMP_TMPZAL    110
#define DMP_A_INV_XH    116
#define DMP_A_INV_XL    118
#define DMP_A_INV_YH    120
#define DMP_A_INV_YL    122
#define DMP_A_INV_ZH    124
#define DMP_A_INV_ZL    126
#define DMP_PSYNC    130
#define DMP_INTX_HC    132
#define DMP_INTX_LC    134
#define DMP_INTY_HC    136
#define DMP_INTY_LC    138
#define DMP_INTZ_HC    140
#define DMP_INTZ_LC    142
#define DMP_TEMP1    144
#define DMP_TEMP2    146
#define DMP_INTX_H    148
#define DMP_INTX_L    150
#define DMP_INTY_H    152
#define DMP_INTY_L    154
#define DMP_INTZ_H    156
#define DMP_INTZ_L    158
#define DMP_STSQ_XH    160
#define DMP_STSQ_XL    162
#define DMP_CTSQ_XH    164
#define DMP_CTSQ_XL    166
#define DMP_CTSQ_YH    168
#define DMP_CTSQ_YL    170
#define DMP_CTSQ_ZH    172
#define DMP_CTSQ_ZL    174
#define DMP_AINV_PH    176
#define DMP_AINV_PL    178
#define DMP_INTX_PH    180
#define DMP_INTX_PL    182
#define DMP_INTY_PH    184
#define DMP_INTY_PL    186
#define DMP_INTZ_PH    188
#define DMP_INTZ_PL    190
#define DMP_AINV_SH    192
#define DMP_AINV_SL    194
#define DMP_INTX_SH    196
#define DMP_INTX_SL    198
#define DMP_INTY_SH    200
#define DMP_INTY_SL    202
#define DMP_INTZ_SH    204
#define DMP_INTZ_SL    206
#define DMP_CTHX_H    212
#define DMP_CTHX_L    214
#define DMP_CTHY_H    216
#define DMP_CTHY_L    218
#define DMP_CTHZ_H    220
#define DMP_CTHZ_L    222
#define DMP_NCTHX_H    228
#define DMP_NCTHX_L    230
#define DMP_NCTHY_H    232
#define DMP_NCTHY_L    234
#define DMP_NCTHZ_H    236
#define DMP_NCTHZ_L    238

#ifdef __cplusplus
}
#endif
#endif // DMPMAP_H
