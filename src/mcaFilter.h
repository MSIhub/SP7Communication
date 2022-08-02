#ifndef MCA_FILTER_H
#define MCA_FILTER_H

#define _USE_MATH_DEFINES

#include <math.h>
#include <map>
#include <spdlog/spdlog.h>
#include "MemBuf.h"
#include "filtering.h"
#include <Eigen/Dense>
using namespace Eigen;

#define SOFTSATURATION
#define NUM_DATA 12
// Imported from common to be aggiusted
#ifdef M_PI
static constexpr double pi = M_PI;
#else
#error "undefined M_PI"
#endif
using Vector6d = Eigen::Matrix<double, 6, 1>;

constexpr double pi_180{pi / 180.};
constexpr double pi_180_inv{180. / pi};
constexpr double deg2rad(double angleInDegree)
{
  return angleInDegree * pi_180;
}
constexpr double rad2deg(double angleRad) { return angleRad * pi_180_inv; }

//

inline double linear_scaling(double x, double f) { return x / f; };

template <typename T> T clamp(const T &n, const T &lower, const T &upper)
{
  return std::max(lower, std::min(n, upper));
}

class McaFilter {
private:
    MemLogger mbf{ 1024 * 1024 * 20, "cueingLog_" };
    std::map<std::string, double> paramMap;

    double t{};     // actual sample time
    double tprev{}; // previous sample time
    double a_g{};
    double b_g{};

    Vector3d aprev{ 0, 0, 0 };
    Vector3d a{ 0, 0, 0 };
    Vector3d pos{ 0, 0, 0.005 };
    Vector3d vel{ 0, 0, 0 };
    Vector3d theta_dot_h{ 0, 0, 0 };


public:
    void loadParams()
    {
        std::string filename = "src/param.yaml";
        std::ifstream param;


        param.open(filename);
        if (!param.is_open())
        {
            std::cout << "file: " << filename << " could not open!" << std::endl;
            return;
        }

        while (param)
        {
            std::string key;
            double value;
            std::getline(param, key, ':');
            param >> value;
            param.get(); // catch empty line
            if (!param)
                return;
            paramMap[key] = value; //paramMap.insert(std::pair<std::string, float>(key, value));
        }
        param.close();
        return;
    }


    void printParams()
    {
        std::map<std::string, double>::iterator itr;
        for (itr = paramMap.begin(); itr != paramMap.end(); ++itr)
        {
            std::cout << itr->first << ": " << itr->second << std::endl;
        }
    }

    McaFilter() {loadParams();}
    McaFilter(McaFilter &c) = delete;

  void logCommit() { mbf.commit(); }

  Matrix3d angular_to_gimbal_matrix(double a1, double a2, double a3);
  Matrix3d get_R_process(double roll, double pitch, double yaw);
  Matrix3d get_T_process(double roll, double pitch, double yaw);
  Vector3d tilt_cord(Vector3d f_L);

  void filtering(float data[NUM_DATA])
  {
      tprev = t;
      t = data[0]; // TODO We need some origin to start and convert t to relative
                   // time
      aprev = a;

      double f_ggx = data[1]; // Specific force along x - it really is ax
      double f_ggy = data[2]; // Specific force along x - it really is ax
      double f_ggz = data[3]; // Specific force along x - it really is ax

      double roll = data[4] * pi_180;
      double pitch = data[5] * pi_180;
      double yaw = data[6] * pi_180;

      double vroll = data[7] * pi_180;
      double vpitch = data[8] * pi_180;
      double vyaw = data[9] * pi_180;

      /*Insert filter logic here*/

      pos[0] = 0.003;
      pos[1] = 0.003;
      pos[2] = 0.001 + 0.396;
      a[0]=0.000;
      a[1]=0.001;
      a[2]=0.001;

      vel[0]=0.400;
      vel[1]=0.400;
      vel[2]=0.000;
      theta_dot_h[0]=0.000;
      theta_dot_h[1]=0.000;
      theta_dot_h[2]=0.00;

      //std::cout << data[1] << ", " << data[2] << ", " << data[3] << ',' << data[4] << "," << data[5] << ',' << data[6] << "," << std::endl;

      ////Testing without filtering
      //pos[0] = data[1];
      //pos[1] = data[2];
      //pos[2] = data[3];
      //a[0]= data[4];
      //a[1]= data[5];
      //a[2]= data[6];

      //vel[0]=data[7];
      //vel[1]=data[8];
      //vel[2]=data[9];
      //theta_dot_h[0]=data[10];
      //theta_dot_h[1]=data[11];
      //theta_dot_h[2]=data[12];
  }

  void getData(double data[NUM_DATA])
  {
    data[0] = pos[0];
    data[1] = pos[1];
    data[2] = pos[2];
    data[3] = a[0];
    data[4] = a[1];
    data[5] = a[2];

    data[6] = vel[0];
    data[7] = vel[1];
    data[8] = vel[2];
    data[9] = theta_dot_h[0];
    data[10] = theta_dot_h[1];
    data[11] = theta_dot_h[2];
  }

  void reset()
  {
    t = 0.;
    tprev = 0.;
    a = Vector3d::Zero();
    aprev = Vector3d::Zero();
    pos = Vector3d::Zero();
    vel = Vector3d::Zero();
    theta_dot_h = Vector3d::Zero();

  }


};




namespace
{
    // static constexpr double fs {47.092}; // Should be 60

    static constexpr double g_acc{ 9.8 }; //< Gravity acceleration
    // scaling factor for linear scaling
    static double scale_x{ 50. };
    static double scale_y{ 50. };
    static double scale_z{ 100. };
    static double scale_wx{ 2. };
    static double scale_wy{ 2. };
    static double scale_wz{ 2. };
    // Saturation acceleration limits

    static double Sat_ax_Min{ -15. };
    static double Sat_ay_Min{ -15. };
    static double Sat_az_Min{ -15. };
    static double Sat_ax_Max{ 15. };
    static double Sat_ay_Max{ 15. };
    static double Sat_az_Max{ 15. };

    // Saturation angular velocity limits
    static double Sat_wx_Min{ -0.4 };
    static double Sat_wy_Min{ -0.4 };
    static double Sat_wz_Min{ -0.4 };
    static double Sat_wx_Max{ 0.4 };
    static double Sat_wy_Max{ 0.4 };
    static double Sat_wz_Max{ 0.4 };

    // Complex way to do zeroPose = [0 0 0.401 0 0 0];
    static const Vector6d
        zeroPose((Vector6d() << 0.0, 0.0, 0.401, 0.0, 0.0, 0.0).finished());

    // Boundary translation limits

    static double xMinLimit{ -0.05 };
    static double yMinLimit{ -0.05 };
    static double zMinLimit{ 0.396 };
    static double xMaxLimit{ 0.05 };
    static double yMaxLimit{ 0.05 };
    static double zMaxLimit{ 0.451 };

    // Boundary rotational limits
    static double thetaxMinLimit{ deg2rad(-5.) };
    static double thetayMinLimit{ deg2rad(-5.) };
    static double thetazMinLimit{ deg2rad(-180.) };
    static double thetaxMaxLimit{ deg2rad(5.) };
    static double thetayMaxLimit{ deg2rad(5.) };
    static double thetazMaxLimit{ deg2rad(180.) };

    // Mid point of Z axis
    static double offset{ (zMaxLimit + zMinLimit) / 2. };

} // namespace McaParam

#endif
