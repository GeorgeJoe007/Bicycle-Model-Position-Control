#include <tuple>
#include <iostream>
#include <cmath>

class BicycleModel {
private:
    double L; // Distance between front and rear axle

public:
    BicycleModel(double L) : L(L) {}

    std::tuple<double, double, double> update(double x, double y, double theta, double v, double delta) {
       
    	  // TODO: implement the kinematics update
        const double dt = 0.1; // Time step for discrete integration

        // Clamp steering angle to prevent numerical instability
        double max_delta = M_PI / 3; // 60 degrees max for safety
        if (delta > max_delta) delta = max_delta;
        if (delta < -max_delta) delta = -max_delta;

        x += v * std::cos(theta) * dt;
        y += v * std::sin(theta) * dt;
        theta += (v / L) * std::tan(delta) * dt;

        // Normalize theta to the range [-pi, pi]
        theta = atan2(sin(theta), cos(theta));
    	
        return std::make_tuple(x, y, theta);
    }
};

class PositionController {
private:
    double Kp;

public:
    PositionController(double Kp) : Kp(Kp) {}

    double compute(double x, double y, double theta, double x_d, double y_d) {
        
        // TODO: implement PID control
        // Calculate distance to target
        double dx = x_d - x;
        double dy = y_d - y;
        double dist = hypot(dx, dy);

        const double EPS = 1e-6;
        if (dist < EPS) {
            return 0.0; // Already at the target
        }

        // Calculate the desired heading angle to reach the target
        double desired_angle = std::atan2(dy, dx);

        // Calculate the heading error
        double heading_error = desired_angle - theta;

        // Angle normalization
        heading_error = atan2(sin(heading_error), cos(heading_error));

        // Proportional control for steering angle
        double delta = Kp * heading_error;

        // Limit steering angle
        double max_steering_angle = M_PI / 4; // 45 degrees max for safety
        if (delta > max_steering_angle) delta = max_steering_angle;
        if (delta < -max_steering_angle) delta = -max_steering_angle;

        return delta;
    }
};

int main() {
    BicycleModel bicycle(1.0);
    PositionController controller(0.1);

    double x = 0.0, y = 0.0, theta = 0.0;
    double x_d = 5.0, y_d = 5.0;
    double v = 1.0;

    for (int i = 0; i < 100; i++) {
        double delta = controller.compute(x, y, theta, x_d, y_d);
        std::tie(x, y, theta) = bicycle.update(x, y, theta, v, delta);
        std::cout << "Position: (" << x << ", " << y << ", " << theta << "), Steering: " << delta << std::endl;
    }

    return 0;
}
