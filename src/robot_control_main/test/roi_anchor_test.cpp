// Kiem chung phan hinh hoc cua tray-anchored ROI bang chinh code trong node.

#include "robot_control_main/roi_anchor.hpp"


#include <cstdio>

static int failures = 0;

static void check(bool ok, const char* what) {
    std::printf("%-58s %s\n", what, ok ? "OK" : "FAIL");
    if (!ok) ++failures;
}

static bool near(float a, float b, float tol = 0.51F) {
    return std::fabs(a - b) <= tol;
}

// ROI row1 that cua cam0 (vision_roi.yaml, da scale sang 640x360)
static ROIQuad row1() {
    return ROIQuad::FromCorners({{157, 260}, {180, 98}, {237, 100}, {226, 260}});
}

static Box rect(double x1, double y1, double x2, double y2) {
    return Box{x1, y1, x2, y2};
}

int main() {
    const RoiAnchor ref{true, 133.0F, 83.0F, 530.0F, 270.0F};

    // 1. Camera dung yen -> transform phai la identity (day la cho ban Pi sai).
    {
        TrayAnchorTracker t;
        t.configure(ref, 1.0, 0.25);   // alpha=1 -> bam ngay, bo qua EMA
        Box obs = rect(ref.x1, ref.y1, ref.x2, ref.y2);
        RoiTransform tf = t.update(&obs);
        check(near(tf.sx, 1.0F, 1e-4F) && near(tf.sy, 1.0F, 1e-4F) &&
              near(tf.ox, 0.0F, 1e-3F) && near(tf.oy, 0.0F, 1e-3F),
              "camera dung yen -> transform identity");
        ROIQuad moved = transformQuad(row1(), tf);
        check(moved.min_x == row1().min_x && moved.min_y == row1().min_y,
              "  ROI khong xe dich khi camera dung yen");
    }

    // 2. Camera truot 12 px sang phai, 7 px xuong -> ROI phai di theo dung vay.
    {
        TrayAnchorTracker t;
        t.configure(ref, 1.0, 0.25);
        Box obs = rect(ref.x1 + 12, ref.y1 + 7, ref.x2 + 12, ref.y2 + 7);
        RoiTransform tf = t.update(&obs);
        ROIQuad moved = transformQuad(row1(), tf);
        check(near(tf.sx, 1.0F, 1e-4F) && near(tf.ox, 12.0F) && near(tf.oy, 7.0F),
              "truot (12,7) -> offset (12,7), khong scale");
        check(moved.min_x == row1().min_x + 12 && moved.min_y == row1().min_y + 7,
              "  ROI dich dung 12,7 px");
    }

    // 3. Khay lai gan camera 10% -> ROI phong to quanh tam khay.
    {
        TrayAnchorTracker t;
        t.configure(ref, 1.0, 0.25);
        const float cx = ref.cx(), cy = ref.cy();
        Box obs = rect(cx - ref.w() * 0.55, cy - ref.h() * 0.55,
                       cx + ref.w() * 0.55, cy + ref.h() * 0.55);
        RoiTransform tf = t.update(&obs);
        check(near(tf.sx, 1.1F, 1e-3F) && near(tf.sy, 1.1F, 1e-3F),
              "khay to them 10% -> scale 1.10");
        // Diem o tam khay phai dung yen khi chi phong to quanh tam.
        Point2 centre = tf.apply(Point2{cx, cy});
        check(near(centre.x, cx) && near(centre.y, cy),
              "  tam khay la diem bat dong cua phep phong to");
    }

    // 4. bbox tray sai (bat nham nua khay) -> bo quan sat, giu transform cu.
    {
        TrayAnchorTracker t;
        t.configure(ref, 1.0, 0.25);
        Box good = rect(ref.x1 + 5, ref.y1, ref.x2 + 5, ref.y2);
        RoiTransform before = t.update(&good);
        Box half = rect(ref.x1, ref.y1, ref.cx(), ref.y2);   // rong con mot nua
        RoiTransform after = t.update(&half);
        check(near(after.sx, before.sx, 1e-4F) && near(after.ox, before.ox, 1e-3F),
              "bbox lech qua nguong -> bo qua, giu transform cu");
        check(t.rejected_streak() == 1, "  dem duoc so quan sat bi loai");
    }

    // 5. Mat tray mot khung -> giu transform, khong snap ve ROI tinh.
    {
        TrayAnchorTracker t;
        t.configure(ref, 1.0, 0.25);
        Box obs = rect(ref.x1 + 9, ref.y1, ref.x2 + 9, ref.y2);
        RoiTransform before = t.update(&obs);
        RoiTransform after = t.update(nullptr);
        check(near(after.ox, before.ox, 1e-3F) && !after.identity,
              "mat tray 1 khung -> giu nguyen transform");
    }

    // 6. reset() (khay bi nhac ra) -> quay ve ROI tinh cho toi khi hoi tu lai.
    {
        TrayAnchorTracker t;
        t.configure(ref, 1.0, 0.25);
        Box obs = rect(ref.x1 + 9, ref.y1, ref.x2 + 9, ref.y2);
        t.update(&obs);
        t.reset();
        RoiTransform tf = t.update(nullptr);
        check(tf.identity, "sau reset -> ROI tinh cho toi khi thay khay moi");
    }

    // 7. Thieu anchor -> luon identity, khong bao gio bu bay.
    {
        TrayAnchorTracker t;
        t.configure(RoiAnchor{}, 1.0, 0.25);
        Box obs = rect(0, 0, 100, 100);
        check(t.update(&obs).identity, "khong co anchor -> luon identity");
    }

    // 8. EMA lam muot: mot cu nhay 20 px chi duoc di 1/4 quang duong.
    {
        TrayAnchorTracker t;
        t.configure(ref, 0.25, 0.25);
        Box settled = rect(ref.x1, ref.y1, ref.x2, ref.y2);
        t.update(&settled);
        Box jump = rect(ref.x1 + 20, ref.y1, ref.x2 + 20, ref.y2);
        RoiTransform tf = t.update(&jump);
        check(near(tf.ox, 5.0F, 0.01F), "EMA alpha=0.25: nhay 20 px -> di 5 px");
    }

    std::printf("\n%s (%d failure)\n", failures ? "CO LOI" : "TAT CA DAT", failures);
    return failures ? 1 : 0;
}
