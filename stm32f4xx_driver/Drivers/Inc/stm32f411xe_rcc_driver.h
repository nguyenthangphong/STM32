#ifndef INC_STM32F411XE_RCC_DRIVER_H_
#define INC_STM32F411XE_RCC_DRIVER_H_

#include "stm32f411xe.h"
#include "stm32f411xe_gpio_driver.h"
#include "stm32f411xe_def.h"

#define RCC_HSI_CLOCK                           (0x00F42400U)                   /* !< 16000000 MHz */
#define RCC_HSE_CLOCK                           (0x007A1200U)                   /* !< 8000000 MHz */

/*
 * RCC Oscillator Type
 */

#define RCC_OSCILLATORTYPE_NONE                 (0x00000000U)
#define RCC_OSCILLATORTYPE_HSE                  (0x00000001U)
#define RCC_OSCILLATORTYPE_HSI                  (0x00000002U)
#define RCC_OSCILLATORTYPE_LSE                  (0x00000004U)
#define RCC_OSCILLATORTYPE_LSI                  (0x00000008U)

/*
 * RCC System Clock Type
 */

#define RCC_CLOCKTYPE_SYSCLK                    (0x00000001U)
#define RCC_CLOCKTYPE_HCLK                      (0x00000002U)
#define RCC_CLOCKTYPE_PCLK1                     (0x00000004U)
#define RCC_CLOCKTYPE_PCLK2                     (0x00000008U)

/*
 * RCC HSE Config
 */

#define RCC_HSE_OFF                             (0x00000000U)                       /*!< HSE oscillator OFF */
#define RCC_HSE_ON                              (0x00000001U)                       /*!< HSE oscillator ON */
#define RCC_HSE_BYPASS                          (RCC_CR_HSEON | RCC_CR_HSEBYP)      /*!< HSE oscillator bypassed with an external clock */

/*
 * RCC HSI Config
 */

#define RCC_HSI_OFF                             (0x00000000U)                       /*!< HSI oscillator OFF */
#define RCC_HSI_ON                              (0x00000001U)                       /*!< HSI oscillator ON */

#define RCC_HSICALIBRATION_DEFAULT              (0x00000010U)

/* LSE Config */
#define RCC_LSE_OFF                             (0x00000000U)                       /*!< LSE oscillator OFF */
#define RCC_LSE_ON                              (0x00000001U)                       /*!< LSE oscillator ON */
#define RCC_LSE_BYPASS                          (RCC_BDCR_LSEON | RCC_BDCR_LSEBYP)  /*!< LSE oscillator bypassed with an external clock */

/* LSI Config */
#define RCC_LSI_OFF                             (0x00000000U)                       /*!< LSI oscillator OFF */
#define RCC_LSI_ON                              (0x00000001U)                       /*!< LSI oscillator ON */

/* PLL Config */
#define RCC_PLL_NONE                            (0x00000000U)                       /*!< PLL None */
#define RCC_PLL_OFF                             (0x00000001U)                       /*!< PLL OFF */
#define RCC_PLL_ON                              (0x00000002U)                       /*!< PLL ON */

/* System Clock */
#define RCC_SYSTEM_CLOCK_DIV_2                  (0x00000008U)
#define RCC_SYSTEM_CLOCK_DIV_4                  (0x00000009U)
#define RCC_SYSTEM_CLOCK_DIV_8                  (0x0000000AU)
#define RCC_SYSTEM_CLOCK_DIV_16                 (0x0000000BU)
#define RCC_SYSTEM_CLOCK_DIV_64                 (0x0000000CU)
#define RCC_SYSTEM_CLOCK_DIV_128                (0x0000000DU)
#define RCC_SYSTEM_CLOCK_DIV_256                (0x0000000EU)
#define RCC_SYSTEM_CLOCK_DIV_512                (0x0000000FU)

/* AHB Clock */
#define RCC_AHB_CLOCK_DIV_2                     (0x00000004U)
#define RCC_AHB_CLOCK_DIV_4                     (0x00000005U)
#define RCC_AHB_CLOCK_DIV_8                     (0x00000006U)
#define RCC_AHB_CLOCK_DIV_16                    (0x00000007U)

/* Division factor for the main PLL (PLL) input clock */
#define RCC_PLLM_DIV_2                          (0x00000002U)                       /*!< PLLM = 2 */
#define RCC_PLLM_DIV_3                          (0x00000003U)                       /*!< PLLM = 3 */
#define RCC_PLLM_DIV_4                          (0x00000004U)                       /*!< PLLM = 4 */
#define RCC_PLLM_DIV_5                          (0x00000005U)                       /*!< PLLM = 5 */
#define RCC_PLLM_DIV_6                          (0x00000006U)                       /*!< PLLM = 6 */
#define RCC_PLLM_DIV_7                          (0x00000007U)                       /*!< PLLM = 7 */
#define RCC_PLLM_DIV_8                          (0x00000008U)                       /*!< PLLM = 8 */
#define RCC_PLLM_DIV_9                          (0x00000009U)                       /*!< PLLM = 9 */
#define RCC_PLLM_DIV_10                         (0x0000000AU)                       /*!< PLLM = 10 */
#define RCC_PLLM_DIV_11                         (0x0000000BU)                       /*!< PLLM = 11 */
#define RCC_PLLM_DIV_12                         (0x0000000CU)                       /*!< PLLM = 12 */
#define RCC_PLLM_DIV_13                         (0x0000000DU)                       /*!< PLLM = 13 */
#define RCC_PLLM_DIV_14                         (0x0000000EU)                       /*!< PLLM = 14 */
#define RCC_PLLM_DIV_15                         (0x0000000FU)                       /*!< PLLM = 15 */
#define RCC_PLLM_DIV_16                         (0x00000010U)                       /*!< PLLM = 16 */
#define RCC_PLLM_DIV_17                         (0x00000011U)                       /*!< PLLM = 17 */
#define RCC_PLLM_DIV_18                         (0x00000012U)                       /*!< PLLM = 18 */
#define RCC_PLLM_DIV_19                         (0x00000013U)                       /*!< PLLM = 19 */
#define RCC_PLLM_DIV_20                         (0x00000014U)                       /*!< PLLM = 20 */
#define RCC_PLLM_DIV_21                         (0x00000015U)                       /*!< PLLM = 21 */
#define RCC_PLLM_DIV_22                         (0x00000016U)                       /*!< PLLM = 22 */
#define RCC_PLLM_DIV_23                         (0x00000017U)                       /*!< PLLM = 23 */
#define RCC_PLLM_DIV_24                         (0x00000018U)                       /*!< PLLM = 24 */
#define RCC_PLLM_DIV_25                         (0x00000019U)                       /*!< PLLM = 25 */
#define RCC_PLLM_DIV_26                         (0x0000001AU)                       /*!< PLLM = 26 */
#define RCC_PLLM_DIV_27                         (0x0000001BU)                       /*!< PLLM = 27 */
#define RCC_PLLM_DIV_28                         (0x0000001CU)                       /*!< PLLM = 28 */
#define RCC_PLLM_DIV_29                         (0x0000001DU)                       /*!< PLLM = 29 */
#define RCC_PLLM_DIV_30                         (0x0000001EU)                       /*!< PLLM = 30 */
#define RCC_PLLM_DIV_31                         (0x0000001FU)                       /*!< PLLM = 31 */
#define RCC_PLLM_DIV_32                         (0x00000020U)                       /*!< PLLM = 32 */
#define RCC_PLLM_DIV_33                         (0x00000021U)                       /*!< PLLM = 33 */
#define RCC_PLLM_DIV_34                         (0x00000022U)                       /*!< PLLM = 34 */
#define RCC_PLLM_DIV_35                         (0x00000023U)                       /*!< PLLM = 35 */
#define RCC_PLLM_DIV_36                         (0x00000024U)                       /*!< PLLM = 36 */
#define RCC_PLLM_DIV_37                         (0x00000025U)                       /*!< PLLM = 37 */
#define RCC_PLLM_DIV_38                         (0x00000026U)                       /*!< PLLM = 38 */
#define RCC_PLLM_DIV_39                         (0x00000027U)                       /*!< PLLM = 39 */
#define RCC_PLLM_DIV_40                         (0x00000028U)                       /*!< PLLM = 40 */
#define RCC_PLLM_DIV_41                         (0x00000029U)                       /*!< PLLM = 41 */
#define RCC_PLLM_DIV_42                         (0x0000002AU)                       /*!< PLLM = 42 */
#define RCC_PLLM_DIV_43                         (0x0000002BU)                       /*!< PLLM = 43 */
#define RCC_PLLM_DIV_44                         (0x0000002CU)                       /*!< PLLM = 44 */
#define RCC_PLLM_DIV_45                         (0x0000002DU)                       /*!< PLLM = 45 */
#define RCC_PLLM_DIV_46                         (0x0000002EU)                       /*!< PLLM = 46 */
#define RCC_PLLM_DIV_47                         (0x0000002FU)                       /*!< PLLM = 47 */
#define RCC_PLLM_DIV_48                         (0x00000030U)                       /*!< PLLM = 48 */
#define RCC_PLLM_DIV_49                         (0x00000031U)                       /*!< PLLM = 49 */
#define RCC_PLLM_DIV_50                         (0x00000032U)                       /*!< PLLM = 50 */
#define RCC_PLLM_DIV_51                         (0x00000033U)                       /*!< PLLM = 51 */
#define RCC_PLLM_DIV_52                         (0x00000034U)                       /*!< PLLM = 52 */
#define RCC_PLLM_DIV_53                         (0x00000035U)                       /*!< PLLM = 53 */
#define RCC_PLLM_DIV_54                         (0x00000036U)                       /*!< PLLM = 54 */
#define RCC_PLLM_DIV_55                         (0x00000037U)                       /*!< PLLM = 55 */
#define RCC_PLLM_DIV_56                         (0x00000038U)                       /*!< PLLM = 56 */
#define RCC_PLLM_DIV_57                         (0x00000039U)                       /*!< PLLM = 57 */
#define RCC_PLLM_DIV_58                         (0x0000003AU)                       /*!< PLLM = 58 */
#define RCC_PLLM_DIV_59                         (0x0000003BU)                       /*!< PLLM = 59 */
#define RCC_PLLM_DIV_60                         (0x0000003CU)                       /*!< PLLM = 60 */
#define RCC_PLLM_DIV_61                         (0x0000003DU)                       /*!< PLLM = 61 */
#define RCC_PLLM_DIV_62                         (0x0000003EU)                       /*!< PLLM = 62 */
#define RCC_PLLM_DIV_63                         (0x0000003FU)                       /*!< PLLM = 63 */

/* Main PLL (PLL) multiplication factor for VCO */
#define RCC_PLLN_MUL_50                         (0x00000032U)                       /*!< PLLN = 50 */
#define RCC_PLLN_MUL_51                         (0x00000033U)                       /*!< PLLN = 51 */
#define RCC_PLLN_MUL_52                         (0x00000034U)                       /*!< PLLN = 52 */
#define RCC_PLLN_MUL_53                         (0x00000035U)                       /*!< PLLN = 53 */
#define RCC_PLLN_MUL_54                         (0x00000036U)                       /*!< PLLN = 54 */
#define RCC_PLLN_MUL_55                         (0x00000037U)                       /*!< PLLN = 55 */
#define RCC_PLLN_MUL_56                         (0x00000038U)                       /*!< PLLN = 56 */
#define RCC_PLLN_MUL_57                         (0x00000039U)                       /*!< PLLN = 57 */
#define RCC_PLLN_MUL_58                         (0x0000003AU)                       /*!< PLLN = 58 */
#define RCC_PLLN_MUL_59                         (0x0000003BU)                       /*!< PLLN = 59 */
#define RCC_PLLN_MUL_60                         (0x0000003CU)                       /*!< PLLN = 60 */
#define RCC_PLLN_MUL_61                         (0x0000003DU)                       /*!< PLLN = 61 */
#define RCC_PLLN_MUL_62                         (0x0000003EU)                       /*!< PLLN = 62 */
#define RCC_PLLN_MUL_63                         (0x0000003FU)                       /*!< PLLN = 63 */
#define RCC_PLLN_MUL_64                         (0x00000040U)                       /*!< PLLN = 64 */
#define RCC_PLLN_MUL_65                         (0x00000041U)                       /*!< PLLN = 65 */
#define RCC_PLLN_MUL_66                         (0x00000042U)                       /*!< PLLN = 66 */
#define RCC_PLLN_MUL_67                         (0x00000043U)                       /*!< PLLN = 67 */
#define RCC_PLLN_MUL_68                         (0x00000044U)                       /*!< PLLN = 68 */
#define RCC_PLLN_MUL_69                         (0x00000045U)                       /*!< PLLN = 69 */
#define RCC_PLLN_MUL_70                         (0x00000046U)                       /*!< PLLN = 70 */
#define RCC_PLLN_MUL_71                         (0x00000047U)                       /*!< PLLN = 71 */
#define RCC_PLLN_MUL_72                         (0x00000048U)                       /*!< PLLN = 72 */
#define RCC_PLLN_MUL_73                         (0x00000049U)                       /*!< PLLN = 73 */
#define RCC_PLLN_MUL_74                         (0x0000004AU)                       /*!< PLLN = 74 */
#define RCC_PLLN_MUL_75                         (0x0000004BU)                       /*!< PLLN = 75 */
#define RCC_PLLN_MUL_76                         (0x0000004CU)                       /*!< PLLN = 76 */
#define RCC_PLLN_MUL_77                         (0x0000004DU)                       /*!< PLLN = 77 */
#define RCC_PLLN_MUL_78                         (0x0000004EU)                       /*!< PLLN = 78 */
#define RCC_PLLN_MUL_79                         (0x0000004FU)                       /*!< PLLN = 79 */
#define RCC_PLLN_MUL_80                         (0x00000050U)                       /*!< PLLN = 80 */
#define RCC_PLLN_MUL_81                         (0x00000051U)                       /*!< PLLN = 81 */
#define RCC_PLLN_MUL_82                         (0x00000052U)                       /*!< PLLN = 82 */
#define RCC_PLLN_MUL_83                         (0x00000053U)                       /*!< PLLN = 83 */
#define RCC_PLLN_MUL_84                         (0x00000054U)                       /*!< PLLN = 84 */
#define RCC_PLLN_MUL_85                         (0x00000055U)                       /*!< PLLN = 85 */
#define RCC_PLLN_MUL_86                         (0x00000056U)                       /*!< PLLN = 86 */
#define RCC_PLLN_MUL_87                         (0x00000057U)                       /*!< PLLN = 87 */
#define RCC_PLLN_MUL_88                         (0x00000058U)                       /*!< PLLN = 88 */
#define RCC_PLLN_MUL_89                         (0x00000059U)                       /*!< PLLN = 89 */
#define RCC_PLLN_MUL_90                         (0x0000005AU)                       /*!< PLLN = 90 */
#define RCC_PLLN_MUL_91                         (0x0000005BU)                       /*!< PLLN = 91 */
#define RCC_PLLN_MUL_92                         (0x0000005CU)                       /*!< PLLN = 92 */
#define RCC_PLLN_MUL_93                         (0x0000005DU)                       /*!< PLLN = 93 */
#define RCC_PLLN_MUL_94                         (0x0000005EU)                       /*!< PLLN = 94 */
#define RCC_PLLN_MUL_95                         (0x0000005FU)                       /*!< PLLN = 95 */
#define RCC_PLLN_MUL_96                         (0x00000060U)                       /*!< PLLN = 96 */
#define RCC_PLLN_MUL_97                         (0x00000061U)                       /*!< PLLN = 97 */
#define RCC_PLLN_MUL_98                         (0x00000062U)                       /*!< PLLN = 98 */
#define RCC_PLLN_MUL_99                         (0x00000063U)                       /*!< PLLN = 99 */
#define RCC_PLLN_MUL_100                        (0x00000064U)                       /*!< PLLN = 100 */
#define RCC_PLLN_MUL_101                        (0x00000065U)                       /*!< PLLN = 101 */
#define RCC_PLLN_MUL_102                        (0x00000066U)                       /*!< PLLN = 102 */
#define RCC_PLLN_MUL_103                        (0x00000067U)                       /*!< PLLN = 103 */
#define RCC_PLLN_MUL_104                        (0x00000068U)                       /*!< PLLN = 104 */
#define RCC_PLLN_MUL_105                        (0x00000069U)                       /*!< PLLN = 105 */
#define RCC_PLLN_MUL_106                        (0x0000006AU)                       /*!< PLLN = 106 */
#define RCC_PLLN_MUL_107                        (0x0000006BU)                       /*!< PLLN = 107 */
#define RCC_PLLN_MUL_108                        (0x0000006CU)                       /*!< PLLN = 108 */
#define RCC_PLLN_MUL_109                        (0x0000006DU)                       /*!< PLLN = 109 */
#define RCC_PLLN_MUL_110                        (0x0000006EU)                       /*!< PLLN = 110 */
#define RCC_PLLN_MUL_111                        (0x0000006FU)                       /*!< PLLN = 111 */
#define RCC_PLLN_MUL_112                        (0x00000070U)                       /*!< PLLN = 112 */
#define RCC_PLLN_MUL_113                        (0x00000071U)                       /*!< PLLN = 113 */
#define RCC_PLLN_MUL_114                        (0x00000072U)                       /*!< PLLN = 114 */
#define RCC_PLLN_MUL_115                        (0x00000073U)                       /*!< PLLN = 115 */
#define RCC_PLLN_MUL_116                        (0x00000074U)                       /*!< PLLN = 116 */
#define RCC_PLLN_MUL_117                        (0x00000075U)                       /*!< PLLN = 117 */
#define RCC_PLLN_MUL_118                        (0x00000076U)                       /*!< PLLN = 118 */
#define RCC_PLLN_MUL_119                        (0x00000077U)                       /*!< PLLN = 119 */
#define RCC_PLLN_MUL_120                        (0x00000078U)                       /*!< PLLN = 120 */
#define RCC_PLLN_MUL_121                        (0x00000079U)                       /*!< PLLN = 121 */
#define RCC_PLLN_MUL_122                        (0x0000007AU)                       /*!< PLLN = 122 */
#define RCC_PLLN_MUL_123                        (0x0000007BU)                       /*!< PLLN = 123 */
#define RCC_PLLN_MUL_124                        (0x0000007CU)                       /*!< PLLN = 124 */
#define RCC_PLLN_MUL_125                        (0x0000007DU)                       /*!< PLLN = 125 */
#define RCC_PLLN_MUL_126                        (0x0000007EU)                       /*!< PLLN = 126 */
#define RCC_PLLN_MUL_127                        (0x0000007FU)                       /*!< PLLN = 127 */
#define RCC_PLLN_MUL_128                        (0x00000080U)                       /*!< PLLN = 128 */
#define RCC_PLLN_MUL_129                        (0x00000081U)                       /*!< PLLN = 129 */
#define RCC_PLLN_MUL_130                        (0x00000082U)                       /*!< PLLN = 130 */
#define RCC_PLLN_MUL_131                        (0x00000083U)                       /*!< PLLN = 131 */
#define RCC_PLLN_MUL_132                        (0x00000084U)                       /*!< PLLN = 132 */
#define RCC_PLLN_MUL_133                        (0x00000085U)                       /*!< PLLN = 133 */
#define RCC_PLLN_MUL_134                        (0x00000086U)                       /*!< PLLN = 134 */
#define RCC_PLLN_MUL_135                        (0x00000087U)                       /*!< PLLN = 135 */
#define RCC_PLLN_MUL_136                        (0x00000088U)                       /*!< PLLN = 136 */
#define RCC_PLLN_MUL_137                        (0x00000089U)                       /*!< PLLN = 137 */
#define RCC_PLLN_MUL_138                        (0x0000008AU)                       /*!< PLLN = 138 */
#define RCC_PLLN_MUL_139                        (0x0000008BU)                       /*!< PLLN = 139 */
#define RCC_PLLN_MUL_140                        (0x0000008CU)                       /*!< PLLN = 140 */
#define RCC_PLLN_MUL_141                        (0x0000008DU)                       /*!< PLLN = 141 */
#define RCC_PLLN_MUL_142                        (0x0000008EU)                       /*!< PLLN = 142 */
#define RCC_PLLN_MUL_143                        (0x0000008FU)                       /*!< PLLN = 143 */
#define RCC_PLLN_MUL_144                        (0x00000090U)                       /*!< PLLN = 144 */
#define RCC_PLLN_MUL_145                        (0x00000091U)                       /*!< PLLN = 145 */
#define RCC_PLLN_MUL_146                        (0x00000092U)                       /*!< PLLN = 146 */
#define RCC_PLLN_MUL_147                        (0x00000093U)                       /*!< PLLN = 147 */
#define RCC_PLLN_MUL_148                        (0x00000094U)                       /*!< PLLN = 148 */
#define RCC_PLLN_MUL_149                        (0x00000095U)                       /*!< PLLN = 149 */
#define RCC_PLLN_MUL_150                        (0x00000096U)                       /*!< PLLN = 150 */
#define RCC_PLLN_MUL_151                        (0x00000097U)                       /*!< PLLN = 151 */
#define RCC_PLLN_MUL_152                        (0x00000098U)                       /*!< PLLN = 152 */
#define RCC_PLLN_MUL_153                        (0x00000099U)                       /*!< PLLN = 153 */
#define RCC_PLLN_MUL_154                        (0x0000009AU)                       /*!< PLLN = 154 */
#define RCC_PLLN_MUL_155                        (0x0000009BU)                       /*!< PLLN = 155 */
#define RCC_PLLN_MUL_156                        (0x0000009CU)                       /*!< PLLN = 156 */
#define RCC_PLLN_MUL_157                        (0x0000009DU)                       /*!< PLLN = 157 */
#define RCC_PLLN_MUL_158                        (0x0000009EU)                       /*!< PLLN = 158 */
#define RCC_PLLN_MUL_159                        (0x0000009FU)                       /*!< PLLN = 159 */
#define RCC_PLLN_MUL_160                        (0x000000A0U)                       /*!< PLLN = 160 */
#define RCC_PLLN_MUL_161                        (0x000000A1U)                       /*!< PLLN = 161 */
#define RCC_PLLN_MUL_162                        (0x000000A2U)                       /*!< PLLN = 162 */
#define RCC_PLLN_MUL_163                        (0x000000A3U)                       /*!< PLLN = 163 */
#define RCC_PLLN_MUL_164                        (0x000000A4U)                       /*!< PLLN = 164 */
#define RCC_PLLN_MUL_165                        (0x000000A5U)                       /*!< PLLN = 165 */
#define RCC_PLLN_MUL_166                        (0x000000A6U)                       /*!< PLLN = 166 */
#define RCC_PLLN_MUL_167                        (0x000000A7U)                       /*!< PLLN = 167 */
#define RCC_PLLN_MUL_168                        (0x000000A8U)                       /*!< PLLN = 168 */
#define RCC_PLLN_MUL_169                        (0x000000A9U)                       /*!< PLLN = 169 */
#define RCC_PLLN_MUL_170                        (0x000000AAU)                       /*!< PLLN = 170 */
#define RCC_PLLN_MUL_171                        (0x000000ABU)                       /*!< PLLN = 171 */
#define RCC_PLLN_MUL_172                        (0x000000ACU)                       /*!< PLLN = 172 */
#define RCC_PLLN_MUL_173                        (0x000000ADU)                       /*!< PLLN = 173 */
#define RCC_PLLN_MUL_174                        (0x000000AEU)                       /*!< PLLN = 174 */
#define RCC_PLLN_MUL_175                        (0x000000AFU)                       /*!< PLLN = 175 */
#define RCC_PLLN_MUL_176                        (0x000000B0U)                       /*!< PLLN = 176 */
#define RCC_PLLN_MUL_177                        (0x000000B1U)                       /*!< PLLN = 177 */
#define RCC_PLLN_MUL_178                        (0x000000B2U)                       /*!< PLLN = 178 */
#define RCC_PLLN_MUL_179                        (0x000000B3U)                       /*!< PLLN = 179 */
#define RCC_PLLN_MUL_180                        (0x000000B4U)                       /*!< PLLN = 180 */
#define RCC_PLLN_MUL_181                        (0x000000B5U)                       /*!< PLLN = 181 */
#define RCC_PLLN_MUL_182                        (0x000000B6U)                       /*!< PLLN = 182 */
#define RCC_PLLN_MUL_183                        (0x000000B7U)                       /*!< PLLN = 183 */
#define RCC_PLLN_MUL_184                        (0x000000B8U)                       /*!< PLLN = 184 */
#define RCC_PLLN_MUL_185                        (0x000000B9U)                       /*!< PLLN = 185 */
#define RCC_PLLN_MUL_186                        (0x000000BAU)                       /*!< PLLN = 186 */
#define RCC_PLLN_MUL_187                        (0x000000BBU)                       /*!< PLLN = 187 */
#define RCC_PLLN_MUL_188                        (0x000000BCU)                       /*!< PLLN = 188 */
#define RCC_PLLN_MUL_189                        (0x000000BDU)                       /*!< PLLN = 189 */
#define RCC_PLLN_MUL_190                        (0x000000BEU)                       /*!< PLLN = 190 */
#define RCC_PLLN_MUL_191                        (0x000000BFU)                       /*!< PLLN = 191 */
#define RCC_PLLN_MUL_192                        (0x000000C0U)                       /*!< PLLN = 192 */
#define RCC_PLLN_MUL_193                        (0x000000C1U)                       /*!< PLLN = 193 */
#define RCC_PLLN_MUL_194                        (0x000000C2U)                       /*!< PLLN = 194 */
#define RCC_PLLN_MUL_195                        (0x000000C3U)                       /*!< PLLN = 195 */
#define RCC_PLLN_MUL_196                        (0x000000C4U)                       /*!< PLLN = 196 */
#define RCC_PLLN_MUL_197                        (0x000000C5U)                       /*!< PLLN = 197 */
#define RCC_PLLN_MUL_198                        (0x000000C6U)                       /*!< PLLN = 198 */
#define RCC_PLLN_MUL_199                        (0x000000C7U)                       /*!< PLLN = 199 */
#define RCC_PLLN_MUL_200                        (0x000000C8U)                       /*!< PLLN = 200 */
#define RCC_PLLN_MUL_201                        (0x000000C9U)                       /*!< PLLN = 201 */
#define RCC_PLLN_MUL_202                        (0x000000CAU)                       /*!< PLLN = 202 */
#define RCC_PLLN_MUL_203                        (0x000000CBU)                       /*!< PLLN = 203 */
#define RCC_PLLN_MUL_204                        (0x000000CCU)                       /*!< PLLN = 204 */
#define RCC_PLLN_MUL_205                        (0x000000CDU)                       /*!< PLLN = 205 */
#define RCC_PLLN_MUL_206                        (0x000000CEU)                       /*!< PLLN = 206 */
#define RCC_PLLN_MUL_207                        (0x000000CFU)                       /*!< PLLN = 207 */
#define RCC_PLLN_MUL_208                        (0x000000D0U)                       /*!< PLLN = 208 */
#define RCC_PLLN_MUL_209                        (0x000000D1U)                       /*!< PLLN = 209 */
#define RCC_PLLN_MUL_210                        (0x000000D2U)                       /*!< PLLN = 210 */
#define RCC_PLLN_MUL_211                        (0x000000D3U)                       /*!< PLLN = 211 */
#define RCC_PLLN_MUL_212                        (0x000000D4U)                       /*!< PLLN = 212 */
#define RCC_PLLN_MUL_213                        (0x000000D5U)                       /*!< PLLN = 213 */
#define RCC_PLLN_MUL_214                        (0x000000D6U)                       /*!< PLLN = 214 */
#define RCC_PLLN_MUL_215                        (0x000000D7U)                       /*!< PLLN = 215 */
#define RCC_PLLN_MUL_216                        (0x000000D8U)                       /*!< PLLN = 216 */
#define RCC_PLLN_MUL_217                        (0x000000D9U)                       /*!< PLLN = 217 */
#define RCC_PLLN_MUL_218                        (0x000000DAU)                       /*!< PLLN = 218 */
#define RCC_PLLN_MUL_219                        (0x000000DBU)                       /*!< PLLN = 219 */
#define RCC_PLLN_MUL_220                        (0x000000DCU)                       /*!< PLLN = 220 */
#define RCC_PLLN_MUL_221                        (0x000000DDU)                       /*!< PLLN = 221 */
#define RCC_PLLN_MUL_222                        (0x000000DEU)                       /*!< PLLN = 222 */
#define RCC_PLLN_MUL_223                        (0x000000DFU)                       /*!< PLLN = 223 */
#define RCC_PLLN_MUL_224                        (0x000000E0U)                       /*!< PLLN = 224 */
#define RCC_PLLN_MUL_225                        (0x000000E1U)                       /*!< PLLN = 225 */
#define RCC_PLLN_MUL_226                        (0x000000E2U)                       /*!< PLLN = 226 */
#define RCC_PLLN_MUL_227                        (0x000000E3U)                       /*!< PLLN = 227 */
#define RCC_PLLN_MUL_228                        (0x000000E4U)                       /*!< PLLN = 228 */
#define RCC_PLLN_MUL_229                        (0x000000E5U)                       /*!< PLLN = 229 */
#define RCC_PLLN_MUL_230                        (0x000000E6U)                       /*!< PLLN = 230 */
#define RCC_PLLN_MUL_231                        (0x000000E7U)                       /*!< PLLN = 231 */
#define RCC_PLLN_MUL_232                        (0x000000E8U)                       /*!< PLLN = 232 */
#define RCC_PLLN_MUL_233                        (0x000000E9U)                       /*!< PLLN = 233 */
#define RCC_PLLN_MUL_234                        (0x000000EAU)                       /*!< PLLN = 234 */
#define RCC_PLLN_MUL_235                        (0x000000EBU)                       /*!< PLLN = 235 */
#define RCC_PLLN_MUL_236                        (0x000000ECU)                       /*!< PLLN = 236 */
#define RCC_PLLN_MUL_237                        (0x000000EDU)                       /*!< PLLN = 237 */
#define RCC_PLLN_MUL_238                        (0x000000EEU)                       /*!< PLLN = 238 */
#define RCC_PLLN_MUL_239                        (0x000000EFU)                       /*!< PLLN = 239 */
#define RCC_PLLN_MUL_240                        (0x000000F0U)                       /*!< PLLN = 240 */
#define RCC_PLLN_MUL_241                        (0x000000F1U)                       /*!< PLLN = 241 */
#define RCC_PLLN_MUL_242                        (0x000000F2U)                       /*!< PLLN = 242 */
#define RCC_PLLN_MUL_243                        (0x000000F3U)                       /*!< PLLN = 243 */
#define RCC_PLLN_MUL_244                        (0x000000F4U)                       /*!< PLLN = 244 */
#define RCC_PLLN_MUL_245                        (0x000000F5U)                       /*!< PLLN = 245 */
#define RCC_PLLN_MUL_246                        (0x000000F6U)                       /*!< PLLN = 246 */
#define RCC_PLLN_MUL_247                        (0x000000F7U)                       /*!< PLLN = 247 */
#define RCC_PLLN_MUL_248                        (0x000000F8U)                       /*!< PLLN = 248 */
#define RCC_PLLN_MUL_249                        (0x000000F9U)                       /*!< PLLN = 249 */
#define RCC_PLLN_MUL_250                        (0x000000FAU)                       /*!< PLLN = 250 */
#define RCC_PLLN_MUL_251                        (0x000000FBU)                       /*!< PLLN = 251 */
#define RCC_PLLN_MUL_252                        (0x000000FCU)                       /*!< PLLN = 252 */
#define RCC_PLLN_MUL_253                        (0x000000FDU)                       /*!< PLLN = 253 */
#define RCC_PLLN_MUL_254                        (0x000000FEU)                       /*!< PLLN = 254 */
#define RCC_PLLN_MUL_255                        (0x000000FFU)                       /*!< PLLN = 255 */
#define RCC_PLLN_MUL_256                        (0x00000100U)                       /*!< PLLN = 256 */
#define RCC_PLLN_MUL_257                        (0x00000101U)                       /*!< PLLN = 257 */
#define RCC_PLLN_MUL_258                        (0x00000102U)                       /*!< PLLN = 258 */
#define RCC_PLLN_MUL_259                        (0x00000103U)                       /*!< PLLN = 259 */
#define RCC_PLLN_MUL_260                        (0x00000104U)                       /*!< PLLN = 260 */
#define RCC_PLLN_MUL_261                        (0x00000105U)                       /*!< PLLN = 261 */
#define RCC_PLLN_MUL_262                        (0x00000106U)                       /*!< PLLN = 262 */
#define RCC_PLLN_MUL_263                        (0x00000107U)                       /*!< PLLN = 263 */
#define RCC_PLLN_MUL_264                        (0x00000108U)                       /*!< PLLN = 264 */
#define RCC_PLLN_MUL_265                        (0x00000109U)                       /*!< PLLN = 265 */
#define RCC_PLLN_MUL_266                        (0x0000010AU)                       /*!< PLLN = 266 */
#define RCC_PLLN_MUL_267                        (0x0000010BU)                       /*!< PLLN = 267 */
#define RCC_PLLN_MUL_268                        (0x0000010CU)                       /*!< PLLN = 268 */
#define RCC_PLLN_MUL_269                        (0x0000010DU)                       /*!< PLLN = 269 */
#define RCC_PLLN_MUL_270                        (0x0000010EU)                       /*!< PLLN = 270 */
#define RCC_PLLN_MUL_271                        (0x0000010FU)                       /*!< PLLN = 271 */
#define RCC_PLLN_MUL_272                        (0x00000110U)                       /*!< PLLN = 272 */
#define RCC_PLLN_MUL_273                        (0x00000111U)                       /*!< PLLN = 273 */
#define RCC_PLLN_MUL_274                        (0x00000112U)                       /*!< PLLN = 274 */
#define RCC_PLLN_MUL_275                        (0x00000113U)                       /*!< PLLN = 275 */
#define RCC_PLLN_MUL_276                        (0x00000114U)                       /*!< PLLN = 276 */
#define RCC_PLLN_MUL_277                        (0x00000115U)                       /*!< PLLN = 277 */
#define RCC_PLLN_MUL_278                        (0x00000116U)                       /*!< PLLN = 278 */
#define RCC_PLLN_MUL_279                        (0x00000117U)                       /*!< PLLN = 279 */
#define RCC_PLLN_MUL_280                        (0x00000118U)                       /*!< PLLN = 280 */
#define RCC_PLLN_MUL_281                        (0x00000119U)                       /*!< PLLN = 281 */
#define RCC_PLLN_MUL_282                        (0x0000011AU)                       /*!< PLLN = 282 */
#define RCC_PLLN_MUL_283                        (0x0000011BU)                       /*!< PLLN = 283 */
#define RCC_PLLN_MUL_284                        (0x0000011CU)                       /*!< PLLN = 284 */
#define RCC_PLLN_MUL_285                        (0x0000011DU)                       /*!< PLLN = 285 */
#define RCC_PLLN_MUL_286                        (0x0000011EU)                       /*!< PLLN = 286 */
#define RCC_PLLN_MUL_287                        (0x0000011FU)                       /*!< PLLN = 287 */
#define RCC_PLLN_MUL_288                        (0x00000120U)                       /*!< PLLN = 288 */
#define RCC_PLLN_MUL_289                        (0x00000121U)                       /*!< PLLN = 289 */
#define RCC_PLLN_MUL_290                        (0x00000122U)                       /*!< PLLN = 290 */
#define RCC_PLLN_MUL_291                        (0x00000123U)                       /*!< PLLN = 291 */
#define RCC_PLLN_MUL_292                        (0x00000124U)                       /*!< PLLN = 292 */
#define RCC_PLLN_MUL_293                        (0x00000125U)                       /*!< PLLN = 293 */
#define RCC_PLLN_MUL_294                        (0x00000126U)                       /*!< PLLN = 294 */
#define RCC_PLLN_MUL_295                        (0x00000127U)                       /*!< PLLN = 295 */
#define RCC_PLLN_MUL_296                        (0x00000128U)                       /*!< PLLN = 296 */
#define RCC_PLLN_MUL_297                        (0x00000129U)                       /*!< PLLN = 297 */
#define RCC_PLLN_MUL_298                        (0x0000012AU)                       /*!< PLLN = 298 */
#define RCC_PLLN_MUL_299                        (0x0000012BU)                       /*!< PLLN = 299 */
#define RCC_PLLN_MUL_300                        (0x0000012CU)                       /*!< PLLN = 300 */
#define RCC_PLLN_MUL_301                        (0x0000012DU)                       /*!< PLLN = 301 */
#define RCC_PLLN_MUL_302                        (0x0000012EU)                       /*!< PLLN = 302 */
#define RCC_PLLN_MUL_303                        (0x0000012FU)                       /*!< PLLN = 303 */
#define RCC_PLLN_MUL_304                        (0x00000130U)                       /*!< PLLN = 304 */
#define RCC_PLLN_MUL_305                        (0x00000131U)                       /*!< PLLN = 305 */
#define RCC_PLLN_MUL_306                        (0x00000132U)                       /*!< PLLN = 306 */
#define RCC_PLLN_MUL_307                        (0x00000133U)                       /*!< PLLN = 307 */
#define RCC_PLLN_MUL_308                        (0x00000134U)                       /*!< PLLN = 308 */
#define RCC_PLLN_MUL_309                        (0x00000135U)                       /*!< PLLN = 309 */
#define RCC_PLLN_MUL_310                        (0x00000136U)                       /*!< PLLN = 310 */
#define RCC_PLLN_MUL_311                        (0x00000137U)                       /*!< PLLN = 311 */
#define RCC_PLLN_MUL_312                        (0x00000138U)                       /*!< PLLN = 312 */
#define RCC_PLLN_MUL_313                        (0x00000139U)                       /*!< PLLN = 313 */
#define RCC_PLLN_MUL_314                        (0x0000013AU)                       /*!< PLLN = 314 */
#define RCC_PLLN_MUL_315                        (0x0000013BU)                       /*!< PLLN = 315 */
#define RCC_PLLN_MUL_316                        (0x0000013CU)                       /*!< PLLN = 316 */
#define RCC_PLLN_MUL_317                        (0x0000013DU)                       /*!< PLLN = 317 */
#define RCC_PLLN_MUL_318                        (0x0000013EU)                       /*!< PLLN = 318 */
#define RCC_PLLN_MUL_319                        (0x0000013FU)                       /*!< PLLN = 319 */
#define RCC_PLLN_MUL_320                        (0x00000140U)                       /*!< PLLN = 320 */
#define RCC_PLLN_MUL_321                        (0x00000141U)                       /*!< PLLN = 321 */
#define RCC_PLLN_MUL_322                        (0x00000142U)                       /*!< PLLN = 322 */
#define RCC_PLLN_MUL_323                        (0x00000143U)                       /*!< PLLN = 323 */
#define RCC_PLLN_MUL_324                        (0x00000144U)                       /*!< PLLN = 324 */
#define RCC_PLLN_MUL_325                        (0x00000145U)                       /*!< PLLN = 325 */
#define RCC_PLLN_MUL_326                        (0x00000146U)                       /*!< PLLN = 326 */
#define RCC_PLLN_MUL_327                        (0x00000147U)                       /*!< PLLN = 327 */
#define RCC_PLLN_MUL_328                        (0x00000148U)                       /*!< PLLN = 328 */
#define RCC_PLLN_MUL_329                        (0x00000149U)                       /*!< PLLN = 329 */
#define RCC_PLLN_MUL_330                        (0x0000014AU)                       /*!< PLLN = 330 */
#define RCC_PLLN_MUL_331                        (0x0000014BU)                       /*!< PLLN = 331 */
#define RCC_PLLN_MUL_332                        (0x0000014CU)                       /*!< PLLN = 332 */
#define RCC_PLLN_MUL_333                        (0x0000014DU)                       /*!< PLLN = 333 */
#define RCC_PLLN_MUL_334                        (0x0000014EU)                       /*!< PLLN = 334 */
#define RCC_PLLN_MUL_335                        (0x0000014FU)                       /*!< PLLN = 335 */
#define RCC_PLLN_MUL_336                        (0x00000150U)                       /*!< PLLN = 336 */
#define RCC_PLLN_MUL_337                        (0x00000151U)                       /*!< PLLN = 337 */
#define RCC_PLLN_MUL_338                        (0x00000152U)                       /*!< PLLN = 338 */
#define RCC_PLLN_MUL_339                        (0x00000153U)                       /*!< PLLN = 339 */
#define RCC_PLLN_MUL_340                        (0x00000154U)                       /*!< PLLN = 340 */
#define RCC_PLLN_MUL_341                        (0x00000155U)                       /*!< PLLN = 341 */
#define RCC_PLLN_MUL_342                        (0x00000156U)                       /*!< PLLN = 342 */
#define RCC_PLLN_MUL_343                        (0x00000157U)                       /*!< PLLN = 343 */
#define RCC_PLLN_MUL_344                        (0x00000158U)                       /*!< PLLN = 344 */
#define RCC_PLLN_MUL_345                        (0x00000159U)                       /*!< PLLN = 345 */
#define RCC_PLLN_MUL_346                        (0x0000015AU)                       /*!< PLLN = 346 */
#define RCC_PLLN_MUL_347                        (0x0000015BU)                       /*!< PLLN = 347 */
#define RCC_PLLN_MUL_348                        (0x0000015CU)                       /*!< PLLN = 348 */
#define RCC_PLLN_MUL_349                        (0x0000015DU)                       /*!< PLLN = 349 */
#define RCC_PLLN_MUL_350                        (0x0000015EU)                       /*!< PLLN = 350 */
#define RCC_PLLN_MUL_351                        (0x0000015FU)                       /*!< PLLN = 351 */
#define RCC_PLLN_MUL_352                        (0x00000160U)                       /*!< PLLN = 352 */
#define RCC_PLLN_MUL_353                        (0x00000161U)                       /*!< PLLN = 353 */
#define RCC_PLLN_MUL_354                        (0x00000162U)                       /*!< PLLN = 354 */
#define RCC_PLLN_MUL_355                        (0x00000163U)                       /*!< PLLN = 355 */
#define RCC_PLLN_MUL_356                        (0x00000164U)                       /*!< PLLN = 356 */
#define RCC_PLLN_MUL_357                        (0x00000165U)                       /*!< PLLN = 357 */
#define RCC_PLLN_MUL_358                        (0x00000166U)                       /*!< PLLN = 358 */
#define RCC_PLLN_MUL_359                        (0x00000167U)                       /*!< PLLN = 359 */
#define RCC_PLLN_MUL_360                        (0x00000168U)                       /*!< PLLN = 360 */
#define RCC_PLLN_MUL_361                        (0x00000169U)                       /*!< PLLN = 361 */
#define RCC_PLLN_MUL_362                        (0x0000016AU)                       /*!< PLLN = 362 */
#define RCC_PLLN_MUL_363                        (0x0000016BU)                       /*!< PLLN = 363 */
#define RCC_PLLN_MUL_364                        (0x0000016CU)                       /*!< PLLN = 364 */
#define RCC_PLLN_MUL_365                        (0x0000016DU)                       /*!< PLLN = 365 */
#define RCC_PLLN_MUL_366                        (0x0000016EU)                       /*!< PLLN = 366 */
#define RCC_PLLN_MUL_367                        (0x0000016FU)                       /*!< PLLN = 367 */
#define RCC_PLLN_MUL_368                        (0x00000170U)                       /*!< PLLN = 368 */
#define RCC_PLLN_MUL_369                        (0x00000171U)                       /*!< PLLN = 369 */
#define RCC_PLLN_MUL_370                        (0x00000172U)                       /*!< PLLN = 370 */
#define RCC_PLLN_MUL_371                        (0x00000173U)                       /*!< PLLN = 371 */
#define RCC_PLLN_MUL_372                        (0x00000174U)                       /*!< PLLN = 372 */
#define RCC_PLLN_MUL_373                        (0x00000175U)                       /*!< PLLN = 373 */
#define RCC_PLLN_MUL_374                        (0x00000176U)                       /*!< PLLN = 374 */
#define RCC_PLLN_MUL_375                        (0x00000177U)                       /*!< PLLN = 375 */
#define RCC_PLLN_MUL_376                        (0x00000178U)                       /*!< PLLN = 376 */
#define RCC_PLLN_MUL_377                        (0x00000179U)                       /*!< PLLN = 377 */
#define RCC_PLLN_MUL_378                        (0x0000017AU)                       /*!< PLLN = 378 */
#define RCC_PLLN_MUL_379                        (0x0000017BU)                       /*!< PLLN = 379 */
#define RCC_PLLN_MUL_380                        (0x0000017CU)                       /*!< PLLN = 380 */
#define RCC_PLLN_MUL_381                        (0x0000017DU)                       /*!< PLLN = 381 */
#define RCC_PLLN_MUL_382                        (0x0000017EU)                       /*!< PLLN = 382 */
#define RCC_PLLN_MUL_383                        (0x0000017FU)                       /*!< PLLN = 383 */
#define RCC_PLLN_MUL_384                        (0x00000180U)                       /*!< PLLN = 384 */
#define RCC_PLLN_MUL_385                        (0x00000181U)                       /*!< PLLN = 385 */
#define RCC_PLLN_MUL_386                        (0x00000182U)                       /*!< PLLN = 386 */
#define RCC_PLLN_MUL_387                        (0x00000183U)                       /*!< PLLN = 387 */
#define RCC_PLLN_MUL_388                        (0x00000184U)                       /*!< PLLN = 388 */
#define RCC_PLLN_MUL_389                        (0x00000185U)                       /*!< PLLN = 389 */
#define RCC_PLLN_MUL_390                        (0x00000186U)                       /*!< PLLN = 390 */
#define RCC_PLLN_MUL_391                        (0x00000187U)                       /*!< PLLN = 391 */
#define RCC_PLLN_MUL_392                        (0x00000188U)                       /*!< PLLN = 392 */
#define RCC_PLLN_MUL_393                        (0x00000189U)                       /*!< PLLN = 393 */
#define RCC_PLLN_MUL_394                        (0x0000018AU)                       /*!< PLLN = 394 */
#define RCC_PLLN_MUL_395                        (0x0000018BU)                       /*!< PLLN = 395 */
#define RCC_PLLN_MUL_396                        (0x0000018CU)                       /*!< PLLN = 396 */
#define RCC_PLLN_MUL_397                        (0x0000018DU)                       /*!< PLLN = 397 */
#define RCC_PLLN_MUL_398                        (0x0000018EU)                       /*!< PLLN = 398 */
#define RCC_PLLN_MUL_399                        (0x0000018FU)                       /*!< PLLN = 399 */
#define RCC_PLLN_MUL_400                        (0x00000190U)                       /*!< PLLN = 400 */
#define RCC_PLLN_MUL_401                        (0x00000191U)                       /*!< PLLN = 401 */
#define RCC_PLLN_MUL_402                        (0x00000192U)                       /*!< PLLN = 402 */
#define RCC_PLLN_MUL_403                        (0x00000193U)                       /*!< PLLN = 403 */
#define RCC_PLLN_MUL_404                        (0x00000194U)                       /*!< PLLN = 404 */
#define RCC_PLLN_MUL_405                        (0x00000195U)                       /*!< PLLN = 405 */
#define RCC_PLLN_MUL_406                        (0x00000196U)                       /*!< PLLN = 406 */
#define RCC_PLLN_MUL_407                        (0x00000197U)                       /*!< PLLN = 407 */
#define RCC_PLLN_MUL_408                        (0x00000198U)                       /*!< PLLN = 408 */
#define RCC_PLLN_MUL_409                        (0x00000199U)                       /*!< PLLN = 409 */
#define RCC_PLLN_MUL_410                        (0x0000019AU)                       /*!< PLLN = 410 */
#define RCC_PLLN_MUL_411                        (0x0000019BU)                       /*!< PLLN = 411 */
#define RCC_PLLN_MUL_412                        (0x0000019CU)                       /*!< PLLN = 412 */
#define RCC_PLLN_MUL_413                        (0x0000019DU)                       /*!< PLLN = 413 */
#define RCC_PLLN_MUL_414                        (0x0000019EU)                       /*!< PLLN = 414 */
#define RCC_PLLN_MUL_415                        (0x0000019FU)                       /*!< PLLN = 415 */
#define RCC_PLLN_MUL_416                        (0x000001A0U)                       /*!< PLLN = 416 */
#define RCC_PLLN_MUL_417                        (0x000001A1U)                       /*!< PLLN = 417 */
#define RCC_PLLN_MUL_418                        (0x000001A2U)                       /*!< PLLN = 418 */
#define RCC_PLLN_MUL_419                        (0x000001A3U)                       /*!< PLLN = 419 */
#define RCC_PLLN_MUL_420                        (0x000001A4U)                       /*!< PLLN = 420 */
#define RCC_PLLN_MUL_421                        (0x000001A5U)                       /*!< PLLN = 421 */
#define RCC_PLLN_MUL_422                        (0x000001A6U)                       /*!< PLLN = 422 */
#define RCC_PLLN_MUL_423                        (0x000001A7U)                       /*!< PLLN = 423 */
#define RCC_PLLN_MUL_424                        (0x000001A8U)                       /*!< PLLN = 424 */
#define RCC_PLLN_MUL_425                        (0x000001A9U)                       /*!< PLLN = 425 */
#define RCC_PLLN_MUL_426                        (0x000001AAU)                       /*!< PLLN = 426 */
#define RCC_PLLN_MUL_427                        (0x000001ABU)                       /*!< PLLN = 427 */
#define RCC_PLLN_MUL_428                        (0x000001ACU)                       /*!< PLLN = 428 */
#define RCC_PLLN_MUL_429                        (0x000001ADU)                       /*!< PLLN = 429 */
#define RCC_PLLN_MUL_430                        (0x000001AEU)                       /*!< PLLN = 430 */
#define RCC_PLLN_MUL_431                        (0x000001AFU)                       /*!< PLLN = 431 */
#define RCC_PLLN_MUL_432                        (0x000001B0U)                       /*!< PLLN = 432 */

/* Main PLL division factor for main system clock */
#define RCC_PLLP_DIV_2                          (0x00000000U)                       /*!< PLLP = 2 */
#define RCC_PLLP_DIV_4                          (0x00000001U)                       /*!< PLLP = 4 */
#define RCC_PLLP_DIV_6                          (0x00000002U)                       /*!< PLLP = 6 */
#define RCC_PLLP_DIV_8                          (0x00000003U)                       /*!< PLLP = 8 */

/* Microcontroller clock output 1 */
#define RCC_MCO1_HSI_CLOCK_SOURCE               (0x00000000U)
#define RCC_MCO1_LSE_CLOCK_SOURCE               (0x00000001U)
#define RCC_MCO1_HSE_CLOCK_SOURCE               (0x00000002U)
#define RCC_MCO1_PLL_CLOCK_SOURCE               (0x00000003U)

/* Microcontroller clock output 1 MCO1 prescaler */

#define RCC_MCO1_PRESCALER_DIV_2                (0x00000004U)
#define RCC_MCO1_PRESCALER_DIV_3                (0x00000005U)
#define RCC_MCO1_PRESCALER_DIV_4                (0x00000006U)
#define RCC_MCO1_PRESCALER_DIV_5                (0x00000007U)

/* Microcontroller clock output 2 */
#define RCC_MCO2_SYSTEM_CLOCK_SOURCE            (0x00000000U)
#define RCC_MCO2_PLLI2S_CLOCK_SOURCE            (0x00000001U)
#define RCC_MCO2_HSEOSCILLATOR_CLOCK_SOURCE     (0x00000002U)
#define RCC_MCO2_PLL_CLOCK_SOURCE               (0x00000003U)

/* System clock switch status */
#define RCC_SWS_HSI_OSCILLATOR                  (0x00000000U)
#define RCC_SWS_HSE_OSCILLATOR                  (0x00000001U)
#define RCC_SWS_PLL                             (0x00000002U)

/* System clock switch */
#define RCC_SW_HSI_OSCILLATOR                   (0x00000000U)
#define RCC_SW_HSE_OSCILLATOR                   (0x00000001U)
#define RCC_SW_PLL                              (0x00000002U)

/* PLL clock source */
#define RCC_PLLSRC_HSI                          (0x00000000U)
#define RCC_PLLSRC_HSE                          (0x00000001U)

/* MCO Index */
#define RCC_MCO1                                (0x00000000U)
#define RCC_MCO2                                (0x00000001U)

/* Flags in the CR register */
#define RCC_FLAG_HSIRDY                         ((uint8_t)0x21U)
#define RCC_FLAG_HSERDY                         ((uint8_t)0x31U)
#define RCC_FLAG_PLLRDY                         ((uint8_t)0x39U)
#define RCC_FLAG_PLLI2SRDY                      ((uint8_t)0x3BU)

/*
 * RCC Flags in the BDCR register
 */

#define RCC_FLAG_LSERDY                         ((uint8_t)0x41U)

/*
 * RCC Flags in the CSR register
 */

#define RCC_FLAG_LSIRDY                         ((uint8_t)0x61U)
#define RCC_FLAG_BORRST                         ((uint8_t)0x79U)
#define RCC_FLAG_PINRST                         ((uint8_t)0x7AU)
#define RCC_FLAG_PORRST                         ((uint8_t)0x7BU)
#define RCC_FLAG_SFTRST                         ((uint8_t)0x7CU)
#define RCC_FLAG_IWDGRST                        ((uint8_t)0x7DU)
#define RCC_FLAG_WWDGRST                        ((uint8_t)0x7EU)
#define RCC_FLAG_LPWRRST                        ((uint8_t)0x7FU)

/*
 * RCC registers bit address in the alias region
 */

#define RCC_OFFSET                              (RCC_BASEADDR - PERIPH_BASEADDR)

/* CR Register */
#define RCC_CR_OFFSET                           (RCC_OFFSET + 0x00u)

/* BDCR Register */
#define RCC_BDCR_OFFSET                         (RCC_OFFSET + 0x70u)

/* CSR Register */
#define RCC_CSR_OFFSET                          (RCC_OFFSET + 0x74u)

/* Alias word address of HSION bit */
#define RCC_HSION_BIT_NUMBER                    (0u)
#define RCC_CR_HSION_BB                         (PERIPH_BB_BASEADDR + (RCC_CR_OFFSET * 32u) + (RCC_HSION_BIT_NUMBER * 4u))

/* Alias word address of PLLON bit */
#define RCC_PLLON_BIT_NUMBER                    (24u)
#define RCC_CR_PLLON_BB                         (PERIPH_BB_BASEADDR + (RCC_CR_OFFSET * 32u) + (RCC_PLLON_BIT_NUMBER * 4u))

/* Alias word address of LSEON bit */
#define RCC_LSEON_BIT_NUMBER                    (0u)
#define RCC_BDCR_LSEON_BB                       (PERIPH_BB_BASEADDR + (RCC_BDCR_OFFSET * 32u) + (RCC_LSEON_BIT_NUMBER * 4u))

/* Alias word address of LSION bit */
#define RCC_LSION_BIT_NUMBER                    (0u)
#define RCC_CSR_LSION_BB                        (PERIPH_BB_BASEADDR + (RCC_CSR_OFFSET * 32u) + (RCC_LSION_BIT_NUMBER * 4u))

/*
 * RCC HSI Configuration
 */

#define RCC_HSI_ENABLE()                        (*(volatile uint32_t *)RCC_CR_HSION_BB = ENABLE)
#define RCC_HSI_DISABLE()                       (*(volatile uint32_t *)RCC_CR_HSION_BB = DISABLE)

/*
 * RCC LSE Configuration
 */

#define RCC_LSE_ENABLE()                        (*(volatile uint32_t *)RCC_BDCR_LSEON_BB = ENABLE)
#define RCC_LSE_DISABLE()                       (*(volatile uint32_t *)RCC_BDCR_LSEON_BB = DISABLE)

/*
 * RCC LSI Configuration
 */

#define RCC_LSI_ENABLE()                        (*(volatile uint32_t *)RCC_CSR_LSION_BB = ENABLE)
#define RCC_LSI_DISABLE()                       (*(volatile uint32_t *)RCC_CSR_LSION_BB = DISABLE)

/*
 * RCC PLL Configuration
 */

#define RCC_PLL_ENABLE()                        (*(volatile uint32_t *)RCC_CR_PLLON_BB = ENABLE)
#define RCC_PLL_DISABLE()                       (*(volatile uint32_t *)RCC_CR_PLLON_BB = DISABLE)

#define RCC_AHB_PRESCALER(x) \
    ((x == RCC_SYSTEM_CLOCK_DIV_2)      ? 2u     : \
     (x == RCC_SYSTEM_CLOCK_DIV_4)      ? 4u     : \
     (x == RCC_SYSTEM_CLOCK_DIV_8)      ? 8u     : \
     (x == RCC_SYSTEM_CLOCK_DIV_16)     ? 16u    : \
     (x == RCC_SYSTEM_CLOCK_DIV_64)     ? 64u    : \
     (x == RCC_SYSTEM_CLOCK_DIV_128)    ? 128u   : \
     (x == RCC_SYSTEM_CLOCK_DIV_256)    ? 256u   : \
     (x == RCC_SYSTEM_CLOCK_DIV_512)    ? 512u   : 0)

#define RCC_APB_PRESCALER(x) \
    ((x == RCC_AHB_CLOCK_DIV_2)         ? 2u     : \
     (x == RCC_AHB_CLOCK_DIV_4)         ? 4u     : \
     (x == RCC_AHB_CLOCK_DIV_8)         ? 8u     : \
     (x == RCC_AHB_CLOCK_DIV_16)        ? 16u    : 0)

#define RCC_PLLP_DIV_FACTOR(x) \
    ((x == RCC_PLLP_DIV_2) ? 2u : \
     (x == RCC_PLLP_DIV_4) ? 4u : \
     (x == RCC_PLLP_DIV_6) ? 6u : \
     (x == RCC_PLLP_DIV_8) ? 8u : 2u)

/*
 * RCC PLL configuration structure definition
 */
typedef struct
{
    uint32_t PLLState;
    uint32_t PLLSource;
    uint32_t PLLM;
    uint32_t PLLN;
    uint32_t PLLP;
    uint32_t PLLQ;
} st_RCC_PLLInitTypeDef_t;

/*
 * RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition
 */

typedef struct
{
    uint32_t OscillatorType;
    uint32_t HSEState;
    uint32_t LSEState;
    uint32_t HSIState;
    uint32_t LSIState;
    uint32_t HSICalibrationValue;
    st_RCC_PLLInitTypeDef_t PLL;
} st_RCC_OscillatorInitTypeDef_t;

/*
 * RCC System, AHB and APB busses clock configuration structure definition
 */

typedef struct
{
    uint32_t ClockType;
    uint32_t SystemClockSource;
    uint32_t AHB_ClockDivider;
    uint32_t APB1_ClockDivider;
    uint32_t APB2_ClockDivider;
} st_RCC_ClockInitTypeDef_t;

e_StatusTypeDef_t RCC_OscillatorConfig(st_RCC_OscillatorInitTypeDef_t *pRCC_Oscillator);
e_StatusTypeDef_t RCC_ClockConfig(st_RCC_ClockInitTypeDef_t *pRCC_Clock);
void RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
uint32_t RCC_GetSysClockFreq(void);

#endif /* INC_STM32F411XE_RCC_DRIVER_H_ */