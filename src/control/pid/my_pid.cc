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

  //# 8. 没有设置初始化选项
  void Init()
  {
    // previous_error_ = 0.;

    // 在三个应该是从配置文件中读取
    integrator_enabled_ = true;
    integrator_saturation_high_ = 50;
    integrator_saturation_low_ = -50;

    integrator_saturation_status_ = 0;
    output_saturation_status_ = 0;

    // pid 具体的值也应该是从配置文件中读取
    SetPID(0.5, 0.4, 0.5);
  }

  double Control(double error, double dt)
  {
    //# 1. 应该增加对 dt 的判断
    if(dt <= 0)
    {
      std::cerr << "dt < 0";
      return preview_output_;
    }

    double diff{0.};

    if(first_hit_)
    {
      first_hit_ = false;
    }
    else
    {
      //# 2. 这样计算才是误差变化率
      // diff = (previous_error_ - error) / dt;
      diff = (error - previous_error_) / dt;
    }

    previous_error_ = error;

    //# 4. 这个逻辑还是错误的
    // if(integrator_enabled_)
    // {
    //   if(integrator_holder_)
    //   {
    //     integrator_ = 0;
    //   }
    //   else
    //   {
    //     integrator_ += ki_ * error;
    //     if(integrator_ > integrator_saturation_high_)
    //     {
    //       integrator_ = integrator_saturation_high_;
    //       integrator_saturation_status_ = 1;
    //     }
    //     else if(integrator_ < integrator_saturation_low_)
    //     {
    //       integrator_ = integrator_saturation_low_;
    //       integrator_saturation_status_ = -1;
    //     }
    //     else
    //     {
    //       integrator_saturation_status_ = 0;
    //     }
    //     std::cout << "integrator: " << integrator_ << "  ";
    //   }
    // }
    // //# 3. 如果不使用积分项
    // else
    // {
    //   integrator_ = 0;
    // }

    if(!integrator_enabled_)
    {
      integrator_ = 0;
    }
    else if(!integrator_holder_)
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
      // std::cout << "integrator: " << integrator_ << "  ";
    }

    // std::cout << "error : " << error << "   diff: " << diff << '\n';
    // output_ = error * kp_ - integrator_ + diff * kd_;
    preview_output_ = error * kp_ + integrator_ + diff * kd_;
    // std::cout << "output: " << output_ << "\n";
    return preview_output_;
  }

  void SetPID(double kp, double ki, double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    kaw_ = 0.;
  }

  //# 5. 没有重置积分累计的接口
  void ResetIntegral()
  {
    integrator_ = 0;
    integrator_saturation_status_ = 0;
  }

  //# 6. 没有重置积分的接口
  void Reset()
  {
    previous_error_ = 0.;
    preview_output_ = 0.;
    first_hit_ = true;
    integrator_ = 0;
    integrator_saturation_status_ = 0;
    output_saturation_status_ = 0;
  }

  void SetIntegratorHold(bool hold)
  {
    integrator_holder_ = hold;
  }

  bool IntegratorHold() const
  {
    return integrator_holder_;
  }

private:
  double kp_{0.};
  double ki_{0.};
  double kd_{0.};
  // 就是抗积分饱和系数，会在 bc 控制器中使用
  double kaw_{0.};

  double integrator_{0.};

  double integrator_saturation_high_{std::numeric_limits<double>::max()};
  double integrator_saturation_low_{std::numeric_limits<double>::min()};
  // 积分饱和状态 之前没有添加
  int integrator_saturation_status_{0};

  bool first_hit_{true};

  // 积分使能就是是否启用积分功能
  bool integrator_enabled_{true};
  // 积分保持就是是否保持当前积分状态
  bool integrator_holder_{false};

  double previous_error_{0.};
  double preview_output_{0.};

  //# 7. 没有设置输出饱和
  double output_saturation_high_{std::numeric_limits<double>::max()};
  double output_saturation_low_{std::numeric_limits<double>::min()};
  int output_saturation_status_{0};
};


int main()
{
  std::cout << "当前水温：" << 0 << "  目标水温：" << 100 << "\n";
  double temple = 0.;

  PidControl pid;
  pid.Init();
  pid.SetPID(0.6, 0.25, 0.5);
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
