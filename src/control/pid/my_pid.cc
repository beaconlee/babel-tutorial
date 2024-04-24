#include <string>
#include <thread>
#include <vector>
#include <random>
#include <ostream>
#include <iostream>
#include <sstream>
#include <algorithm>


class PidControl
{
public:
  PidControl() = default;

  void Init()
  {
    integrator_enabled_ = true;
  }

  double Control(double error, double dt)
  {
    double diff{0.};
    if(!first_hit_)
    {
      // diff = (previous_error_ - error) / dt;
      diff = (error - previous_error_) / dt;
    }
    else
    {
      first_hit_ = false;
    }
    previous_error_ = error;

    if(integrator_enabled_)
    {
      if(integrator_holder_)
      {
        integrator_ = 0;
      }
      else
      {
        integrator_ += ki_ * error;
        if(integrator_ > integrator_saturation_high_)
        {
          integrator_ = integrator_saturation_high_;
          integrator_saturation_status_ = 1;
        }
        else if(integrator_ < integrator_saturation_low_)
        {
          integrator_ = integrator_saturation_low_;
          integrator_saturation_status_ = -1;
        }
        else
        {
          integrator_saturation_status_ = 0;
        }
        std::cout << "integrator: " << integrator_ << "  ";
      }
    }

    std::cout << "error : " << error << "   diff: " << diff << '\n';
    // output_ = error * kp_ - integrator_ + diff * kd_;
    output_ = error * kp_ + integrator_ + diff * kd_;
    std::cout << "output: " << output_ << "\n";
    return output_;
  }

  void SetPID(double kp, double ki, double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    integrator_saturation_high_ = 100;
    integrator_saturation_low_ = -100;
  }

private:
  double kp_{0.};
  double ki_{0.};
  double kd_{0.};
  double yaw_{0.};

  double integrator_{0.};

  double integrator_saturation_high_{std::numeric_limits<double>::max()};
  double integrator_saturation_low_{std::numeric_limits<double>::min()};
  // 积分饱和状态 之前没有添加
  double integrator_saturation_status_{0};

  bool first_hit_{true};

  bool integrator_enabled_{true};
  bool integrator_holder_{false};

  double previous_error_{0.};
  double previous_otput_{0.};


  double output_{0.};
};


int main()
{
  std::cout << "当前水温：" << 0 << "  目标水温：" << 100 << "\n";
  double temple = 0.;

  PidControl pid;
  pid.Init();
  pid.SetPID(0.6, 0.4, 0.5);
  while(true)
  {
    // std::this_thread::sleep_for(std::chrono::duration<std::chrono::seconds, std::ratio<1, 1>>(1));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "当前的 temple: " << temple << "\n";
    // temple = pid.Control(temple - 100, 1);
    // 犯了一个非常重大的错误，pid 的 error 的计算是 target - real
    temple += pid.Control(100 - temple, 1);
  }
  return 0;
}

// 问题 1. 没有设置对是否会进入 init
//     2. 积分项累积过大，变化特别明显
//     3. 波动非常大，没有向目标靠拢
//     4. 直接将 pid 输出的值作为最终的结果，没有将其附加到控制变量上

//     5. 当出现 控制值变得异常大时，可能时 pid 中的 d 过大了
//     6. 当需要较长时间才能进入到一个稳定状态时：可能是 i 相关
//
//
//
//    💌 犯了一个非常重大的错误，pid 的 error 的计算是 target - real
