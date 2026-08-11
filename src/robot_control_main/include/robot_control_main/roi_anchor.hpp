// Hinh hoc ROI cho vision_decision_node — tach ra header de test duoc doc lap
// (khong keo theo rclcpp), xem test/roi_anchor_test.cpp.
#ifndef ROBOT_CONTROL_MAIN__ROI_ANCHOR_HPP_
#define ROBOT_CONTROL_MAIN__ROI_ANCHOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

struct Point2 {
    float x{}, y{};
};

struct Box { double x1, y1, x2, y2; };

struct ROIQuad {
    std::array<Point2, 4> pts;
    int min_x{}, max_x{}, min_y{}, max_y{};

    static ROIQuad FromCorners(const std::vector<std::pair<int, int>>& corners) {
        ROIQuad r{};
        for (size_t i = 0; i < 4; ++i) {
            r.pts[i] = Point2{static_cast<float>(corners[i].first),
                              static_cast<float>(corners[i].second)};
        }
        r.min_x = std::min({corners[0].first, corners[1].first,
                            corners[2].first, corners[3].first});
        r.max_x = std::max({corners[0].first, corners[1].first,
                            corners[2].first, corners[3].first});
        r.min_y = std::min({corners[0].second, corners[1].second,
                            corners[2].second, corners[3].second});
        r.max_y = std::max({corners[0].second, corners[1].second,
                            corners[2].second, corners[3].second});
        return r;
    }

    inline bool bbox_contains(float x, float y) const {
        return (x >= min_x && x <= max_x && y >= min_y && y <= max_y);
    }

    inline bool contains(float x, float y) const {
        if (!bbox_contains(x, y)) return false;
        auto cross = [](const Point2& a, const Point2& b, const Point2& c) {
            return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        };
        const Point2 p{x, y};
        float c0 = cross(pts[0], pts[1], p);
        float c1 = cross(pts[1], pts[2], p);
        float c2 = cross(pts[2], pts[3], p);
        float c3 = cross(pts[3], pts[0], p);
        
        bool all_nonneg = (c0 >= 0 && c1 >= 0 && c2 >= 0 && c3 >= 0);
        bool all_nonpos = (c0 <= 0 && c1 <= 0 && c2 <= 0 && c3 <= 0);
        return all_nonneg || all_nonpos;
    }
};

// ============================================================================
// ROI NEO THEO KHAY (tray-anchored ROI)
// ============================================================================
// ROI cham tay la toa do tuyet doi trong anh, nen camera xe dich vai pixel la
// tam cartridge roi sang row/slot ben canh — YOLO van detect dung, chi rieng
// buoc quy ROI bi sai. Camera tren may nay chi truot, khong xoay, nen bbox
// class-0 "tray" du de bu: no dich va phong to y het phan con lai cua khay.
//
// So voi ban Pi (robot_logic_node_dual.cpp) co ba khac biet co y:
//  1. Anchor la AABB cua chinh bbox tray luc cham ROI, cung loai primitive voi
//     thu detect duoc. Ban Pi lay canonical la hinh thang roi map sang hinh chu
//     nhat, nen H khac identity ngay ca khi camera dung yen.
//  2. Hai hinh chu nhat thang truc -> homography suy bien thanh scale+translate.
//     Bon phep nhan, khong can OpenCV, khong co ma tran suy bien.
//  3. bbox tray rung +-vai pixel moi khung; EMA truoc khi dung, neu khong ROI
//     rung theo va cartridge nhay qua lai giua hai row.

struct RoiTransform {
    float sx{1.0F}, sy{1.0F}, ox{0.0F}, oy{0.0F};
    bool identity{true};

    inline Point2 apply(const Point2& p) const {
        if (identity) return p;
        return Point2{p.x * sx + ox, p.y * sy + oy};
    }
};

// Hinh chu nhat tham chieu, doc tu `anchor:` trong vision_roi.yaml.
struct RoiAnchor {
    bool valid{false};
    float x1{}, y1{}, x2{}, y2{};

    float w() const { return x2 - x1; }
    float h() const { return y2 - y1; }
    float cx() const { return 0.5F * (x1 + x2); }
    float cy() const { return 0.5F * (y1 + y2); }
};

class TrayAnchorTracker {
public:
    void configure(const RoiAnchor& reference, double alpha, double max_scale_dev) {
        reference_ = reference;
        alpha_ = static_cast<float>(std::clamp(alpha, 0.01, 1.0));
        max_scale_dev_ = static_cast<float>(std::max(0.01, max_scale_dev));
        reset();
    }

    // Khay bien mat -> quen luon vi tri cu. Lan dat khay tiep theo phai hoi tu
    // lai tu dau thay vi keo ROI theo vi tri khay truoc do.
    void reset() {
        has_smoothed_ = false;
        rejected_streak_ = 0;
    }

    bool configured() const { return reference_.valid; }

    // observed == nullptr: khung hinh nay khong thay tray (bi che, hoac khay
    // vua duoc nhac ra). Giu nguyen transform cu — snap ve ROI tinh giua chung
    // se lam ROI giat manh hon nhieu so voi loi ma no dang bu.
    RoiTransform update(const Box* observed) {
        if (!reference_.valid) return RoiTransform{};

        if (observed != nullptr) {
            const float w = static_cast<float>(observed->x2 - observed->x1);
            const float h = static_cast<float>(observed->y2 - observed->y1);
            if (w > 1.0F && h > 1.0F) {
                const float sx = w / reference_.w();
                const float sy = h / reference_.h();
                // bbox tray sai (bat nham vat khac, hoac khay bi che mot nua)
                // se cho ti le lech han 1.0. Bo quan sat do, giu transform cu.
                if (std::fabs(sx - 1.0F) <= max_scale_dev_ &&
                    std::fabs(sy - 1.0F) <= max_scale_dev_)
                {
                    const float ocx = 0.5F * static_cast<float>(observed->x1 + observed->x2);
                    const float ocy = 0.5F * static_cast<float>(observed->y1 + observed->y2);
                    if (!has_smoothed_) {
                        s_cx_ = ocx; s_cy_ = ocy; s_w_ = w; s_h_ = h;
                        has_smoothed_ = true;
                    } else {
                        s_cx_ += alpha_ * (ocx - s_cx_);
                        s_cy_ += alpha_ * (ocy - s_cy_);
                        s_w_  += alpha_ * (w   - s_w_);
                        s_h_  += alpha_ * (h   - s_h_);
                    }
                    rejected_streak_ = 0;
                } else {
                    ++rejected_streak_;
                }
            } else {
                ++rejected_streak_;
            }
        }

        if (!has_smoothed_) return RoiTransform{};

        RoiTransform t;
        t.sx = s_w_ / reference_.w();
        t.sy = s_h_ / reference_.h();
        t.ox = s_cx_ - reference_.cx() * t.sx;
        t.oy = s_cy_ - reference_.cy() * t.sy;
        t.identity = false;
        return t;
    }

    int rejected_streak() const { return rejected_streak_; }

private:
    RoiAnchor reference_{};
    float alpha_{0.25F};
    float max_scale_dev_{0.25F};
    bool has_smoothed_{false};
    float s_cx_{}, s_cy_{}, s_w_{}, s_h_{};
    int rejected_streak_{0};
};

inline ROIQuad transformQuad(const ROIQuad& base, const RoiTransform& t) {
    if (t.identity) return base;
    std::vector<std::pair<int, int>> corners;
    corners.reserve(4);
    for (const auto& p : base.pts) {
        const Point2 q = t.apply(p);
        corners.emplace_back(static_cast<int>(std::lround(q.x)),
                             static_cast<int>(std::lround(q.y)));
    }
    return ROIQuad::FromCorners(corners);
}

#endif  // ROBOT_CONTROL_MAIN__ROI_ANCHOR_HPP_
