#ifndef FORCE_CALIBRATION_H
#define FORCE_CALIBRATION_H

#include <Arduino.h>

// =====================================================================
// 当前 main 标定：实测范围 0--200 g，超过实测范围后线性外推。
//
// 钳位设在物理上限之上：以最后一段斜率 0.006484 N/raw 计算，
// ADC 满量程 raw=4095 只对应约 24.4 N，因此 30 N 这个钳位
// 真实按压永远够不着，只用来挡接线故障导致的荒唐值。
// 数值会随按压持续增长，但 1.961 N 以上全部是外推，不可作为实测力引用。
// =====================================================================

namespace ForceCalibration {

// ---- 当前编译进固件的标定身份，开机横幅会打印它 ----
// ★ 每次改动下面的表或钳位值，都要同步改这句，否则横幅会撒谎。
constexpr const char* VERSION =
    "2026-08-17 PROVISIONAL | 0-200 g | 6 pts | extrapolating, clamp 30 N (ADC ceiling ~24.4 N)";

// PROVISIONAL stitched calibration for the 2026-08-17 experiment setup.
// Uses stable tail medians from the mechanically plausible parts of:
//   calibration_0_200g_trial1_20260817_173116/samples_corrected.csv
//   calibration_0_100_200g_check1_20260817_175040/samples.csv
// Rejected points that were non-monotonic or visibly inconsistent across runs.
//   0 g   -> raw 0
//   20 g  -> raw 175.0
//   30 g  -> raw 213.0
//   60 g  -> raw 336.0
//   100 g -> mean of check loading raw 464.5 and unloading raw 507.0 = 485.75
//   200 g -> check loading raw 637.0
//
// ⚠ 这份数据是在**第一块 ESP32**（2026-08-20 烧毁，已更换）上采的。
//    ESP32 芯片间 ADC 差异较大，换板后应重新验两点。
// ⚠ 表的上界是 raw 637 = 1.961 N。**再往上全是外推，不是实测。**
//    实测灵敏度在饱和：100→200 g 那一段只需 154 raw/N，而前面几段是 380~420。
//    线性外推很可能**低估**真实的力 —— 对力安全而言这是危险方向。
constexpr size_t POINT_COUNT = 6;
constexpr float RAW_POINTS[POINT_COUNT] = {
    0.0f,
    175.0f,
    213.0f,
    336.0f,
    485.75f,
    637.0f,
};
constexpr float FORCE_POINTS_N[POINT_COUNT] = {
    0.0f,
    0.196133f,
    0.2941995f,
    0.588399f,
    0.980665f,
    1.961330f,
};

// ---- 外推钳位 ----
// raw=4095（ADC 满量程）≈ 24.4 N < 30 N，所以这个钳位只挡故障值，不会压平真实读数。
constexpr float MAX_EXTRAPOLATED_FORCE_N = 30.0f;

}  // namespace ForceCalibration

#endif
