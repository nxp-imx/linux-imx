#ifndef __INC_HAL8723S_FW_IMG_H
#define __INC_HAL8723S_FW_IMG_H

/*Created on  2011/11/17,  7:21*/

//#if (RTL8723S_HWIMG_SUPPORT == 1)
#if 1

#define Rtl8723SImgArrayLength 20864
extern const u8 Rtl8723SFwImgArray[Rtl8723SImgArrayLength];
#define Rtl8723SBTImgArrayLength 1
extern const u8 Rtl8723SFwBTImgArray[Rtl8723SBTImgArrayLength];
#define Rtl8723SUMCBCutImgArrayLength 20606
extern const u8 Rtl8723SFwUMCBCutImgArray[Rtl8723SUMCBCutImgArrayLength];
#define Rtl8723SPHY_REG_2TArrayLength 1
extern const u32 Rtl8723SPHY_REG_2TArray[Rtl8723SPHY_REG_2TArrayLength];
#define Rtl8723SPHY_REG_1TArrayLength 372
extern const u32 Rtl8723SPHY_REG_1TArray[Rtl8723SPHY_REG_1TArrayLength];
#define Rtl8723SPHY_ChangeTo_1T1RArrayLength 1
extern const u32 Rtl8723SPHY_ChangeTo_1T1RArray[Rtl8723SPHY_ChangeTo_1T1RArrayLength];
#define Rtl8723SPHY_ChangeTo_1T2RArrayLength 1
extern const u32 Rtl8723SPHY_ChangeTo_1T2RArray[Rtl8723SPHY_ChangeTo_1T2RArrayLength];
#define Rtl8723SPHY_ChangeTo_2T2RArrayLength 1
extern const u32 Rtl8723SPHY_ChangeTo_2T2RArray[Rtl8723SPHY_ChangeTo_2T2RArrayLength];
#define Rtl8723SPHY_REG_Array_PGLength 336
extern const u32 Rtl8723SPHY_REG_Array_PG[Rtl8723SPHY_REG_Array_PGLength];
#define Rtl8723SPHY_REG_Array_MPLength 4
extern const u32 Rtl8723SPHY_REG_Array_MP[Rtl8723SPHY_REG_Array_MPLength];
#define Rtl8723SRadioA_2TArrayLength 1
extern const u32 Rtl8723SRadioA_2TArray[Rtl8723SRadioA_2TArrayLength];
#define Rtl8723SRadioB_2TArrayLength 1
extern const u32 Rtl8723SRadioB_2TArray[Rtl8723SRadioB_2TArrayLength];
#define Rtl8723SRadioA_1TArrayLength 282
extern const u32 Rtl8723SRadioA_1TArray[Rtl8723SRadioA_1TArrayLength];
#define Rtl8723SRadioB_1TArrayLength 1
extern const u32 Rtl8723SRadioB_1TArray[Rtl8723SRadioB_1TArrayLength];
#define Rtl8723SRadioB_GM_ArrayLength 1
extern const u32 Rtl8723SRadioB_GM_Array[Rtl8723SRadioB_GM_ArrayLength];
#define Rtl8723SMAC_2T_ArrayLength 172
extern const u32 Rtl8723SMAC_2T_Array[Rtl8723SMAC_2T_ArrayLength];
#define Rtl8723SMACPHY_Array_PGLength 1
extern const u32 Rtl8723SMACPHY_Array_PG[Rtl8723SMACPHY_Array_PGLength];
#define Rtl8723SAGCTAB_2TArrayLength 1
extern const u32 Rtl8723SAGCTAB_2TArray[Rtl8723SAGCTAB_2TArrayLength];
#define Rtl8723SAGCTAB_1TArrayLength 320
extern const u32 Rtl8723SAGCTAB_1TArray[Rtl8723SAGCTAB_1TArrayLength];

#else

#define Rtl8723SImgArrayLength 1
extern const u8 Rtl8723SFwImgArray[Rtl8723SImgArrayLength];
#define Rtl8723SBTImgArrayLength 1
extern const u8 Rtl8723SFwBTImgArray[Rtl8723SBTImgArrayLength];
#define Rtl8723SUMCBCutImgArrayLength 1
extern const u8 Rtl8723SFwUMCBCutImgArray[Rtl8723SUMCBCutImgArrayLength];
#define Rtl8723SPHY_REG_2TArrayLength 1
extern const u32 Rtl8723SPHY_REG_2TArray[Rtl8723SPHY_REG_2TArrayLength];
#define Rtl8723SPHY_REG_1TArrayLength 1
extern const u32 Rtl8723SPHY_REG_1TArray[Rtl8723SPHY_REG_1TArrayLength];
#define Rtl8723SPHY_ChangeTo_1T1RArrayLength 1
extern const u32 Rtl8723SPHY_ChangeTo_1T1RArray[Rtl8723SPHY_ChangeTo_1T1RArrayLength];
#define Rtl8723SPHY_ChangeTo_1T2RArrayLength 1
extern const u32 Rtl8723SPHY_ChangeTo_1T2RArray[Rtl8723SPHY_ChangeTo_1T2RArrayLength];
#define Rtl8723SPHY_ChangeTo_2T2RArrayLength 1
extern const u32 Rtl8723SPHY_ChangeTo_2T2RArray[Rtl8723SPHY_ChangeTo_2T2RArrayLength];
#define Rtl8723SPHY_REG_Array_PGLength 1
extern const u32 Rtl8723SPHY_REG_Array_PG[Rtl8723SPHY_REG_Array_PGLength];
#define Rtl8723SPHY_REG_Array_MPLength 1
extern const u32 Rtl8723SPHY_REG_Array_MP[Rtl8723SPHY_REG_Array_MPLength];
#define Rtl8723SRadioA_2TArrayLength 1
extern const u32 Rtl8723SRadioA_2TArray[Rtl8723SRadioA_2TArrayLength];
#define Rtl8723SRadioB_2TArrayLength 1
extern const u32 Rtl8723SRadioB_2TArray[Rtl8723SRadioB_2TArrayLength];
#define Rtl8723SRadioA_1TArrayLength 1
extern const u32 Rtl8723SRadioA_1TArray[Rtl8723SRadioA_1TArrayLength];
#define Rtl8723SRadioB_1TArrayLength 1
extern const u32 Rtl8723SRadioB_1TArray[Rtl8723SRadioB_1TArrayLength];
#define Rtl8723SRadioB_GM_ArrayLength 1
extern const u32 Rtl8723SRadioB_GM_Array[Rtl8723SRadioB_GM_ArrayLength];
#define Rtl8723SMAC_2T_ArrayLength 1
extern const u32 Rtl8723SMAC_2T_Array[Rtl8723SMAC_2T_ArrayLength];
#define Rtl8723SMACPHY_Array_PGLength 1
extern const u32 Rtl8723SMACPHY_Array_PG[Rtl8723SMACPHY_Array_PGLength];
#define Rtl8723SAGCTAB_2TArrayLength 1
extern const u32 Rtl8723SAGCTAB_2TArray[Rtl8723SAGCTAB_2TArrayLength];
#define Rtl8723SAGCTAB_1TArrayLength 1
extern const u32 Rtl8723SAGCTAB_1TArray[Rtl8723SAGCTAB_1TArrayLength];

#endif //#if (RTL8723S_HWIMG_SUPPORT == 1)

#endif //__INC_HAL8723S_FW_IMG_H
