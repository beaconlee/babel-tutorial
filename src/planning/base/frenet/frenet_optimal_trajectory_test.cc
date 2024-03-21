int main(int argc, char** argv)
{
  vector<double> wx = {0.0, 10.0, 20.5, 35.0, 70.5};
  vector<double> wy = {0.0, -6.0, 5.0, 6.5, 0.0};
  vector<vector<double>> obs = {{20., 30., 30., 35., 50.},
                                {10., 6., 8., 8., 3.}};
  vector<vector<double>> spline =
      CubicSpline2D::calc_spline_course(wx, wy, 0.1);
  CubicSpline2D csp = CubicSpline2D(wx, wy);

  double c_speed = 10.0 / 3.6;
  double c_accel = 0.0;
  double c_d = 2.0;
  double c_d_d = 0.0;
  double c_d_dd = 0.0;
  double s0 = 0.0;
  double area = 20.0;
  utils::VehicleConfig vc(0.9);
  size_t iter = 0;
  while(iter++ < SIM_LOOP)
  {
    FrenetPath path = frenet_optimal_planning(
        csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, obs);

    s0 = path.s[1];
    c_d = path.d[1];
    c_d_d = path.d_d[1];
    c_d_dd = path.d_dd[1];
    c_speed = path.s_d[1];
    c_accel = path.s_dd[1];

    if(hypot(path.x[1] - spline[0].back(), path.y[1] - spline[1].back()) <= 1.)
    {
      break;
    }
    if(show_animation)
    {
      plt::cla();
      plt::named_plot("The planned spline path", spline[0], spline[1]);
      plt::plot(obs[0], obs[1], "xk");
      plt::named_plot("The optimal trajectory", path.x, path.y, "-r");
      // The steer here is not strictly calculated by vehicle kinematics,
      // but is only visualized based on curvature scaling.
      utils::draw_vehicle({path.x[0], path.y[0], path.yaw[0]},
                          utils::pi_2_pi(5 * path.c[0]),
                          vc);

      plt::xlim(path.x[1] - area, path.x[1] + area);
      plt::ylim(path.y[1] - area, path.y[1] + area);
      plt::title("Frenet Optimal Trajectory V[km/h]:" +
                 std::to_string(c_speed * 3.6).substr(0, 4));
      plt::legend({{"loc", "upper left"}});
      plt::grid(true);
      plt::pause(0.0001);
    }
  }

  if(show_animation)
  {
    plt::grid(true);
    plt::show();
  }

  return 0;
}
