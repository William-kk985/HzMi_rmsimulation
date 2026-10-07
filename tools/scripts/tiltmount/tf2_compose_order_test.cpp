#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cstdio>
#include <cmath>
int main() {
  tf2::Quaternion qa; qa.setRPY(0, 0, M_PI/2);          // A = 绕 z 转 90 度
  tf2::Transform A(qa, tf2::Vector3(0,0,0));
  tf2::Quaternion qb; qb.setRPY(M_PI/2, 0, 0);          // B = 绕 x 转 90 度
  tf2::Transform B(qb, tf2::Vector3(0,0,1));
  tf2::Transform C = A * B;
  tf2::Vector3 p(1,0,0);
  tf2::Vector3 c = C * p, ab = A * (B * p), ba = B * (A * p);
  printf("C*p     = (% .3f,% .3f,% .3f)\n", c.x(), c.y(), c.z());
  printf("A*(B*p) = (% .3f,% .3f,% .3f)\n", ab.x(), ab.y(), ab.z());
  printf("B*(A*p) = (% .3f,% .3f,% .3f)\n", ba.x(), ba.y(), ba.z());
  tf2::Matrix3x3 m(C.getRotation());
  double r1,p1,y1; m.getRPY(r1,p1,y1);
  printf("C rpy deg = (%.2f, %.2f, %.2f)\n", r1*180/M_PI, p1*180/M_PI, y1*180/M_PI);
  // 复刻 small_point_lio_node.cpp:104 的写法：T_ob = T_bl^{-1} * T_ol * T_bl
  // 复刻 small_point_lio_node.cpp:99 真正拿到的东西：
  //   base_link_to_lidar_frame_transform = lookupTransform(lidar_frame, "base_link")
  //                                      = T(livox<-base) = Rx(+30)
  tf2::Quaternion qbl; qbl.setRPY(M_PI/6, 0, 0);        // T(livox<-base) = Rx(+30)
  tf2::Transform Tbl(qbl, tf2::Vector3(0.0005617, 0.130916, 0.157028));
  tf2::Quaternion qol; qol.setRPY(0, 0, M_PI/20);       // T_ol = 平放、yaw 9 度
  tf2::Transform Tol(qol, tf2::Vector3(0,0,0));
  tf2::Transform Tob = Tbl.inverse() * Tol * Tbl;
  tf2::Matrix3x3 m2(Tob.getRotation()); double r2,p2,y2; m2.getRPY(r2,p2,y2);
  printf("node.cpp 写法 T_bl^-1*T_ol*T_bl 的 rpy = (%.3f, %.3f, %.3f) deg   （T_ol 本身 rpy = (0,0,9) 水平）\n", r2*180/M_PI, p2*180/M_PI, y2*180/M_PI);
  printf("物理正确的合成 T_ol*T_bl^-1 的 rpy = ");
  tf2::Transform Tob2 = Tol * Tbl.inverse();
  tf2::Matrix3x3 m3(Tob2.getRotation()); double r3,p3,y3; m3.getRPY(r3,p3,y3);
  printf("(%.3f, %.3f, %.3f) deg\n", r3*180/M_PI, p3*180/M_PI, y3*180/M_PI);
  return 0;
}
