#pragma once
#include "types.h"
#include <array>
namespace robot {
using Vec4 = std::array<double,4>;
using Mat4 = std::array<std::array<double,4>,4>;
Mat4 zero4();
Mat4 eye4();
Mat4 transpose4(const Mat4& a);
Mat4 mul4(const Mat4& a,const Mat4& b);
Vec4 mul4v(const Mat4& a,const Vec4& x);
Vec4 mulPB(const Mat4& P,const Vec4& B);
Vec4 BtP(const Vec4& B,const Mat4& P);
double dot4(const Vec4& a,const Vec4& b);
Mat4 make_state_weight(double wx,double wv,double wtheta,double wrate);
void build_velocity_model(const Config& cfg,double dt,double vel_ref_tps,double pitch_ref_deg,double vel_schedule_scale,double pitch_schedule_scale,Mat4& Ad,Vec4& Bd);
bool solve_discrete_lqr(const Mat4& A,const Vec4& B,const Mat4& Q,double R,Vec4& K,Mat4& P,int iterations=200);
bool solve_velocity_lqr_from_config(const Config& cfg,double dt,double vel_ref_tps,double pitch_ref_deg,double vel_schedule_scale,double pitch_schedule_scale,Vec4& K,Mat4& P,bool use_auto_lqr_weights=true);
}
