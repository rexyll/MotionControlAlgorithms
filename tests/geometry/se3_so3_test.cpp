#include <gtest/gtest.h>
#include <cmath>
#include "mca/geometry/se3_so3.hpp"
namespace mca::geometry {
TEST(Se3So3Test, Rotation){const Quaternion q=axis_angle({0,0,1},std::acos(-1.0)/2);const Vec3 p=rotate(q,{1,0,0});EXPECT_NEAR(p.x,0,1e-12);EXPECT_NEAR(p.y,1,1e-12);}
TEST(Se3So3Test, MatrixRoundTrip){const Quaternion q=axis_angle({1,2,3},1.1);const Quaternion r=from_rotation_matrix(to_rotation_matrix(q));const Vec3 p{.3,-.8,1.2};const Vec3 a=rotate(q,p),b=rotate(r,p);EXPECT_NEAR(a.x,b.x,1e-12);EXPECT_NEAR(a.y,b.y,1e-12);EXPECT_NEAR(a.z,b.z,1e-12);}
TEST(Se3So3Test, ExpLog){const Vec3 v{.2,-.3,.4};const Vec3 r=log_so3(exp_so3(v));EXPECT_NEAR(r.x,v.x,1e-12);EXPECT_NEAR(r.y,v.y,1e-12);EXPECT_NEAR(r.z,v.z,1e-12);}
TEST(Se3So3Test, PoseComposition){const Pose3D a{{1,2,3},axis_angle({0,1,0},.4)},b{{.4,-.2,.7},axis_angle({1,0,0},-.3)};const Pose3D c=compose(a,b);const Vec3 p{.2,.5,-.4};const Vec3 x=transform_point(c,p),y=transform_point(a,transform_point(b,p));EXPECT_NEAR(x.x,y.x,1e-12);EXPECT_NEAR(x.y,y.y,1e-12);EXPECT_NEAR(x.z,y.z,1e-12);}
TEST(Se3So3Test, ZeroRotation){const Vec3 p=rotate(exp_so3({0,0,0}),{1,2,3});EXPECT_NEAR(p.x,1,1e-15);EXPECT_NEAR(p.y,2,1e-15);EXPECT_NEAR(p.z,3,1e-15);}
}