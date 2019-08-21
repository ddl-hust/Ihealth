#ifndef Matrix_H
#define Matrix_H
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>


using namespace Eigen;

const double ShoulderWidth = 0.297;
const double ShoulderLength = 0.435;
const double UpperArmLength = 0.296;
const double LowerArmLength = 0.384;
const double guide = 0.07;
const double shouler_installationsite_to_coordinate4 = 0.140;
const double elbow_installationsite_to_coordinate5 = 0.150;
const double d1 = 0.23;
const double d2 = 0.4683;
const double d3 = 0.307;
const double d4 = 0.276 + guide;
const double d5 = 0.174;
const double r5 = 0.074;
const double dy_2 = 0.087;
const double dz_2 = 0.188;
const double sixdim_to_coordinate3_level = 0.047;
const double sixdim_to_coordinate3_vertiacl = 0.130;

const double InitAngle[5] = {0, 0, 0, 0, 15};

extern Vector3d AxisDirection[5];
extern Vector3d AxisPosition[5];

////#define LEFT_ARM 1
//#ifdef LEFT_ARM
////这里的AxisDirection就是定义了每个角速度方向上的单位矢量w
////这个就是根据我们的旋转怎么转决定的。
// Vector3d AxisDirection[5] = {
//	Vector3d(1.0,0,0),
//	Vector3d(0,0,1.0),
//	Vector3d(0,-1.0,0),
//	Vector3d(0,-1.0,0),
//	Vector3d(-1.0,0,0)
//};
// Vector3d AxisPosition[5] = {
//	Vector3d(-UpperArmLength - LowerArmLength,0,0),
//	Vector3d(-UpperArmLength - LowerArmLength,0,0),
//	Vector3d(-UpperArmLength - LowerArmLength,0,0),
//	Vector3d(-LowerArmLength,0,0),
//	Vector3d(-LowerArmLength,0,0)
//};
//#else
// static Vector3d AxisDirection[5] = {
//	Vector3d(-1,0,0),
//	Vector3d(0,0,-1),
//	Vector3d(0,-1,0),
//	Vector3d(0,-1,0),
//	Vector3d(1,0,0)
//};
// static Vector3d AxisPosition[5] = {
//	Vector3d(-UpperArmLength - LowerArmLength,0,0),
//	Vector3d(-UpperArmLength - LowerArmLength,0,0),
//	Vector3d(-UpperArmLength - LowerArmLength,0,0),
//	Vector3d(-LowerArmLength,0,0),
//	Vector3d(-LowerArmLength,0,0)
//};
//#endif

//这个函数实际上就是在计算运动旋量ζ
template <typename DerivedA, typename DerivedB, typename DerivedC>
void CalculateSpinor(const MatrixBase<DerivedA> &axis, const MatrixBase<DerivedB> &pos, MatrixBase<DerivedC> &twist)
{
    Matrix3d axis_hat;
    twist.setZero();
    axis_hat << 0, -axis(2), axis(1), axis(2), 0, -axis(0), -axis(1), axis(0), 0;
    //这里算出来的w_q就是运动w × q
    Vector3d w_q = axis.cross(pos);
    twist.block(0, 0, 3, 3) = axis_hat;
    twist.block(0, 3, 3, 1) = -w_q;
}

template <typename DerivedA, typename DerivedB, typename DerivedC>
void pinv(const MatrixBase<DerivedA> &A, const MatrixBase<DerivedB> &G, MatrixBase<DerivedC> &B)
{
    //这里就是在算这个投影矩阵p，最后的结果用B输出来，这里的A就是那个Γ，这里的G就是那个G
    MatrixXd A_temp(2, 2);
    A_temp = A.transpose() * G * A;
    B = (A_temp.inverse()) * A.transpose() * G;
}

template <typename DerivedA, typename DerivedB>
void pinv2(const MatrixBase<DerivedA> &A, MatrixBase<DerivedB> &B)
{
    MatrixXd A_temp(2, 2);
    A_temp = A.transpose() * A;
    B = (A_temp.inverse()) * A.transpose();
}

//将向量转成矩阵，从而化叉乘为点乘
template <typename DerivedA, typename DerivedB>
void VectorToMatrix(const MatrixBase<DerivedA> &X, MatrixBase<DerivedB> &Y)
{
    MatrixXd y(3, 3);
    y.setZero();
    y(1, 2) = -X(0);
    y(0, 2) = X(1);
    y(0, 1) = -X(2);
    Y = y - y.transpose();
}

//这个函数的意思居然是求伴随矩阵，从4*4的转换到6*6的
template <typename DerivedA, typename DerivedB>
void CalculateAdjointMatrix(const MatrixBase<DerivedA> &X, MatrixBase<DerivedB> &A)
{
    MatrixXd Y(3, 3);
    Vector3d h(3);
    A.setZero();
    A.block(0, 0, 3, 3) = X.block(0, 0, 3, 3);
    A.block(3, 3, 3, 3) = X.block(0, 0, 3, 3);
    h = X.block(0, 3, 3, 1);
    VectorToMatrix(h, Y);
    A.block(3, 0, 3, 3) = Y * X.block(0, 0, 3, 3);
}
template <typename DerivedA, typename DerivedB>
void MatrixToVector(const MatrixBase<DerivedA> &X, MatrixBase<DerivedB> &b)
{
    //这里出来的这个b就是旋转矢量ξ，只不过是因为我们之前的spinor_hat是个4*4，这里又把它弄回来了而已
    //这里的b就是6*1的了
    b(0) = -X(1, 2);
    b(1) = X(0, 2);
    b(2) = -X(0, 1);
    b.block(3, 0, 3, 1) = X.block(0, 3, 3, 1);
}

template <typename DerivedA, typename DerivedB>
void ActiveJointsAngleToAllJointsAngle(const MatrixBase<DerivedA> &U, MatrixBase<DerivedB> &theta)
{
    MatrixXd meta(5, 2);
    VectorXd thetab(5);

    // meta就是传动矩阵η
    meta << 1, 0, 0.88, 0, 0, 1, 0, 1.3214, 0, 0.6607;

    thetab << InitAngle[0], InitAngle[1], InitAngle[2], InitAngle[3], InitAngle[4];
    //电机的角度就是初始角度加上转换后的角度
    theta = thetab + meta * U;
}

template <typename DerivedA, typename DerivedB, typename DerivedC>
void damping_control(const MatrixBase<DerivedA> &Fh, MatrixBase<DerivedB> &U, MatrixBase<DerivedC> &Ub, double Fc, double a, double b)
{

    VectorXd theta(5);
    VectorXd theta_PI(5);

    MatrixXd con(2, 2);


    VectorXd d(5);
    VectorXd diag(6);

    MatrixXd jacobian(6, 5);
    MatrixXd p_X(2, 6);
    MatrixXd Co(2, 2);
    VectorXd Co_tem(6);
    MatrixXd G(6, 6);
    //这里的U是电机的输出角度2*1（我们可以通过获取当前角度直接获得），然后theta是5*1，分别是
    // 5个关节的角度。下面这个函数就是把电机的角度转换为关节的角度。
    ActiveJointsAngleToAllJointsAngle(U, theta);


    con << 360.0 / (2 * M_PI), 0, 0, 360.0 / (2 * M_PI);

    //这里的spinor_hat和spinor就是为了算J的每一列得到的
    Matrix4d spinor_hat[5];
    Matrix<double, 6, 1> spinor[5];
    for (size_t i = 0; i < 5; ++i) {
        // AxisDirection:3*3 AxisPosition:3*3 spinor_hat:4*4
        //这里的spinor_hat输出的是一个4*4的矩阵，在灿神的文本中这个叫做B_hat.然后MatrixToVector是把矩阵变成向量。
        //这里的spinor[i]是一个6*1的向量
        CalculateSpinor(AxisDirection[i], AxisPosition[i], spinor_hat[i]);
        MatrixToVector(spinor_hat[i], spinor[i]);
    }
    /*
    这里的m应该是ζhat，这一点我还不知道是什么东西。
    从这里看来，这里的计算只计算了四个关节的。
    也就是把θ5看为了0.这样就导致了最后一个直接就是关节旋量。
    */
    Matrix4d m[4];
    for (size_t i = 0; i < 4; ++i) {
        m[i] = -spinor_hat[i + 1] * (2 * M_PI / 360.0) * theta(i + 1);
    }
    Matrix4d exp_m[4];
    exp_m[3] = m[3].exp();
    exp_m[2] = exp_m[3] * (m[2].exp());
    exp_m[1] = exp_m[2] * (m[1].exp());
    exp_m[0] = exp_m[1] * (m[0].exp());
    Matrix<double, 6, 6> A[4];
    CalculateAdjointMatrix(exp_m[0], A[0]);
    CalculateAdjointMatrix(exp_m[1], A[1]);
    CalculateAdjointMatrix(exp_m[2], A[2]);
    CalculateAdjointMatrix(exp_m[3], A[3]);

    //这里对应MR中p202，因为一开始就是在体坐标系中表示的，
    //所以这里spinor本就指的是Bi
    jacobian.block(0, 0, 6, 1) = A[0] * spinor[0];
    jacobian.block(0, 1, 6, 1) = A[1] * spinor[1];
    jacobian.block(0, 2, 6, 1) = A[2] * spinor[2];
    jacobian.block(0, 3, 6, 1) = A[3] * spinor[3];
    jacobian.block(0, 4, 6, 1) = spinor[4];
    diag << a, a, a, b, b, b; //度量矩阵
    Co_tem << 20, 20, 20, 1, 1, 1; //六维力的比例系数
    G = diag.asDiagonal();
    Co = Co_tem.asDiagonal();

    MatrixXd meta(5, 2);
    meta << 1, 0, 0.88, 0, 0, 1, 0, 1.3214, 0, 0.6607;
    //这里就是Γ = J * η。所以说这个meta就是η，jacobian就是J
    Matrix<double, 6, 2> MapMatrix = jacobian * meta;
    pinv(MapMatrix, G, p_X); //投影矩阵
    // Fc-导纳系数，Fh-六维力，co-六维力单位转换矩阵，con-六维速度单位转换矩阵（从弧度转为度），Ub-电机转速
    //这里p_X就是这个投影矩阵了，最后的Vd = ACF,这里Co就是这个C，然后Fc就是这个导纳系数。最后多乘个0.1应该是太
    //灵敏了的调整。然后最后要转换，把电机的速度变为角度，因为我们外面是用的角度。
    Ub = con * (p_X * Co * Fh * Fc * 0.15);
}

//求伪逆
template <typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD<_Matrix_Type_> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal()
           * svd.matrixU().adjoint();
}

template <typename DerivedA>
void MomentBalance(const MatrixBase<DerivedA> &shoulderforcevector, double motorangle[2], double moment[3])
{
    Matrix3d axisdirection_hat[4];
    Matrix3d spinor_hat[4];
    Matrix3d so3[4];
    Matrix3d SO3[4];

    Matrix3d R32;
    Matrix3d R21;
    Matrix3d RF13;
    Matrix3d P1_3;
    Matrix3d to_zero;
    MatrixXd Tf1_3(6, 6);

    Vector3d pa1_3 = Vector3d(d3 - shouler_installationsite_to_coordinate4, 0, -dy_2);
    Vector3d f1_3;
    Vector3d n1_3;
    Vector3d p5_4 = Vector3d(0, -d5, -r5);
    Vector3d p4_3 = Vector3d(d3, 0, 0);
    Vector3d p3_2 = Vector3d(0, -dy_2, -dz_2);
    Vector3d p2_1 = Vector3d(0, d1, -d2);

    VectorXd Co_tem(6);
    VectorXd joint_angle(5);
    VectorXd shoulder_force_moment(6);
    VectorXd elbow_force_moment(6);

    //六维力相关的变量
    MatrixXd sixdim_transfer(6, 6);
    Matrix3d sixdim_rotation;
    Matrix3d Sixdim_To_Coordinate3;
    Vector3d sixdim_to_coordinate3 = Vector3d(sixdim_to_coordinate3_vertiacl, 0, -sixdim_to_coordinate3_level);

    MatrixXd Pos(2, 1);
    Pos(0, 0) = motorangle[0];
    Pos(1, 0) = motorangle[1];

    ActiveJointsAngleToAllJointsAngle(Pos, joint_angle);

    for (int i = 0; i < 5; ++i) {
        joint_angle(i) = (M_PI / 180) * joint_angle(i);
    }

    //各个关节间的旋转矩阵
    R32 << cos(joint_angle(2)), -sin(joint_angle(2)), 0, 0, 0, 1, sin(joint_angle(2)), cos(joint_angle(2)), 0;
    R21 << cos(joint_angle(1)), -sin(joint_angle(1)), 0, 0, 0, -1, sin(joint_angle(1)), cos(joint_angle(1)), 0;


    //力从套环到关节的旋转矩阵
    to_zero.setZero();
    RF13 << 1, 0, 0, 0, 0.8710, 0.4914, 0, -0.4914, 0.8710;
    sixdim_rotation << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    VectorToMatrix(pa1_3, P1_3);
    VectorToMatrix(sixdim_to_coordinate3, Sixdim_To_Coordinate3);
    Tf1_3 << RF13, to_zero, P1_3 * RF13, RF13;
    sixdim_transfer << sixdim_rotation, to_zero, Sixdim_To_Coordinate3 * sixdim_rotation, sixdim_rotation;

    Vector3d f3_3;
    Vector3d n3_3;
    Vector3d f2_2;
    Vector3d n2_2;
    Vector3d f1_1;
    Vector3d n1_1;

    //这里直接把套环的广义力矢量变换到关节空间中
    shoulder_force_moment = sixdim_transfer * shoulderforcevector;
    f1_3 << shoulder_force_moment(0), shoulder_force_moment(1), shoulder_force_moment(2);
    n1_3 << shoulder_force_moment(3), shoulder_force_moment(4), shoulder_force_moment(5);

    // AllocConsole();
    // freopen("CONOUT$", "w", stdout);
    // cout << "f1_3:" << f1_3 << "\n" << "n1_3:" << n1_3 << endl;
    // cout << "f2_5:" << f2_5 << "\n" << "n2_5:" << n2_5 << endl;
    // cout << "shoulder_force_moment:" << shoulder_force_moment << "\n" << "elbow_force_moment" << elbow_force_moment << endl;

    //力矩平衡公式
    f3_3 = f1_3;
    n3_3 = n1_3;
    f2_2 = R32 * f3_3;
    n2_2 = R32 * n3_3 + p3_2.cross(f2_2);
    f1_1 = R21 * f2_2;
    n1_1 = R21 * n2_2 + p2_1.cross(f1_1);

    // AllocConsole();
    // freopen("CONOUT$", "w", stdout);
    // cout << "f5_5:" << f5_5 << "\n" << "n5_5:" << n5_5 << endl;
    // cout << "f4_4:" << f4_4 << "\n" << "n4_4:" << n4_4 << endl;
    // cout << "f3_3:" << f3_3 << "\n" << "n3_3:" << n3_3 << endl;
    // cout << "f2_2:\n" << f2_2 << "\n" << "n2_2:\n" << n2_2 << endl;
    // cout << "f1_1:\n" << f1_1 << "\n" << "n1_1:\n" << n1_1 << endl;

    moment[0] = n1_1(2);
    moment[1] = n2_2(2);
    moment[2] = n3_3(2);
}

template <typename DerivedA, typename DerivedB>
void AdmittanceControl(const MatrixBase<DerivedA> &torque, MatrixBase<DerivedB> &vel)
{
    Vector2d ratio;
    Matrix2d m_ratio;
    Vector2d admittance;
    Matrix2d m_admittance;

    MatrixXd meta(5, 2);
    meta << 1, 0, 0.88, 0, 0, 1, 0, 1.3214, 0, 0.6607;

    MatrixXd projection(2, 5);

    pinv2(meta, projection);

    ratio << 0.9, 0.9;
    admittance << 3, 3.5;
    m_ratio = ratio.asDiagonal();
    m_admittance = admittance.asDiagonal();

    vel = m_admittance * (m_ratio * projection * torque);
}

template <typename DerivedA, typename DerivedB>
void fwd_geo_kineB(const MatrixBase<DerivedA> &theta, MatrixBase<DerivedB> &T0h)
{
    Matrix4d exp_m[5];
    Matrix4d Bh[5];
    for (size_t i = 0; i < 5; ++i) {
        CalculateSpinor(AxisDirection[i], AxisPosition[i], Bh[i]);
        exp_m[i] = Bh[i] * (2 * M_PI / 360) * theta(i);
    }
    Matrix4d T0h0;
    T0h0 << 1, 0, 0, ShoulderLength + UpperArmLength + LowerArmLength, 0, 1, 0, 0, 0, 0, 1, ShoulderWidth, 0, 0, 0, 1;
    T0h = T0h0 * (exp_m[0].exp()) * (exp_m[1].exp()) * (exp_m[2].exp()) * (exp_m[3].exp()) * (exp_m[4].exp());
}
template <typename DerivedA>
void Euler2RotMat(double IMU_yaw, double IMU_pitch, double IMU_roll, MatrixBase<DerivedA> &R)
{
    IMU_yaw = IMU_yaw * 2 * M_PI / 360;
    IMU_pitch = IMU_pitch * 2 * M_PI / 360;
    IMU_roll = IMU_roll * 2 * M_PI / 360;
    R(0, 0) = cos(IMU_yaw) * cos(IMU_pitch);
    R(0, 1) = cos(IMU_yaw) * sin(IMU_pitch) * sin(IMU_roll) - sin(IMU_yaw) * cos(IMU_roll);
    R(0, 2) = cos(IMU_yaw) * sin(IMU_pitch) * cos(IMU_roll) + sin(IMU_yaw) * sin(IMU_roll);
    R(1, 0) = sin(IMU_yaw) * cos(IMU_pitch);
    R(1, 1) = sin(IMU_yaw) * sin(IMU_pitch) * sin(IMU_roll) + cos(IMU_yaw) * cos(IMU_roll);
    R(1, 2) = sin(IMU_yaw) * sin(IMU_pitch) * cos(IMU_roll) - cos(IMU_yaw) * sin(IMU_roll);
    R(2, 0) = -sin(IMU_pitch);
    R(2, 1) = cos(IMU_pitch) * sin(IMU_roll);
    R(2, 2) = cos(IMU_pitch) * cos(IMU_roll);
}

template <typename DerivedA, typename DerivedB, typename DerivedC>
void handle_ori(const MatrixBase<DerivedA> &Rw0, MatrixBase<DerivedB> &Rwh, MatrixBase<DerivedC> &R0h)
{
    MatrixXd mat_convert(3, 3);
    MatrixXd R0h_temp(3, 3);
    mat_convert << 1, 0, 0, 0, 0, 1, 0, -1, 0;
    R0h_temp = (Rw0.transpose()) * Rwh;
    R0h = R0h_temp * mat_convert;
}
template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
void Cal_phg(const MatrixBase<DerivedA> &R0h, MatrixBase<DerivedB> &t, MatrixBase<DerivedC> &f, MatrixBase<DerivedD> &p)
{
    MatrixXd Rhs(3, 3);
    MatrixXd A(3, 3);
    MatrixXd A_temp(3, 3);
    MatrixXd A_tem(3, 3);
    MatrixXd A_te(3, 3);
    MatrixXd A_t(3, 3);
    MatrixXd N(3, 3);
    double m = 1;
    Vector3d phs(3);
    Vector3d g(3);
    Vector3d b(3);
    VectorXd Co_tem(3);

    Co_tem << 1, 1, 1;
    phs << -0.0447, -0.1055, 0;
    Rhs = Co_tem.asDiagonal();
    VectorToMatrix(-R0h.transpose() * m * g, A);
    VectorToMatrix(phs, N);
    b = Rhs * t + N * Rhs * f;
    /*A_temp = A.transpose()*A;
    A_tem = A_temp.inverse();
    A_te = A.transpose();
    A_t = A_tem*A_te;*/
    p = b;
}
template <typename DerivedA, typename DerivedB, typename DerivedC>
void Cal_CoAdFg(const MatrixBase<DerivedA> &phg, MatrixBase<DerivedB> &R0h, MatrixBase<DerivedC> &CoAdFg)
{


    Vector3d h(3);
    Vector3d n(3);
    MatrixXd v2(3, 3);
    g << 9.8, 0, 0;
    CoAdFg << 0, 0, 0, 0, 0, 0;
    VectorToMatrix(phg, v2);
    h = v2 * R0h.transpose() * m * g;
    n = R0h.transpose() * m * g;
    CoAdFg.head(3) = h;
    CoAdFg.tail(3) = n;
}
template <typename DerivedA, typename DerivedB>
void fwd(const MatrixBase<DerivedA> &theta_Pi, MatrixBase<DerivedB> &P0)
{
    double offset = 0.08;
    VectorXd d(5);
    VectorXd theta(5);
    d << 0.180, 0.434, 0.298, 0.2706 + 31 / 1000, 0.0614;
    double d1 = 0;
    double d2 = 0;
    double p[3] = {0};
    d1 = d(2);
    d2 = d(3);
    theta = theta_Pi * M_PI / 180;
    P0(0) = d1 * cos(theta(1)) * cos(theta(2)) - d2 * (cos(theta(1)) * sin(theta(2)) * sin(theta(2)) - cos(theta(1)) * cos(theta(2)) * cos(theta(3)))
            + 217.0 / 500.0;
    P0(1) = d1 * (sin(theta(0)) * sin(theta(2)) - cos(theta(0)) * cos(theta(2)) * sin(theta(1)))
            + d2
                  * (cos(theta(3)) * (sin(theta(0)) * sin(theta(2)) - cos(theta(0)) * cos(theta(2)) * sin(theta(1)))
                     + sin(theta(2)) * (cos(theta(2)) * sin(theta(0)) + cos(theta(0)) * sin(theta(1)) * sin(theta(2))));
    P0(2) = d1 * (cos(theta(0)) * sin(theta(2)) + cos(theta(2)) * sin(theta(0)) * sin(theta(1)))
            + d2
                  * (cos(theta(3)) * (cos(theta(0)) * sin(theta(2)) + cos(theta(2)) * sin(theta(0)) * sin(theta(1)))
                     + sin(theta(2)) * (cos(theta(0)) * cos(theta(2)) - sin(theta(0)) * sin(theta(1)) * sin(theta(2))))
            + 9.0 / 50.0;
}

#endif
