#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>

#define GRAVITY 9.8// 定义重力加速度

//角度转弧度
const double dag_to_rad = (180 / M_PI);
//容许误差范围
const double error_rangeX = 0.1;
const double error_rangeY = 0.1;

//弹丸建模
const double dragCoefficient = 0.4303; // 球体的空气阻力系数（根据实际情况调整）
//https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22597
const double airDensity = 1.225;     // kg/m^3
const double ballRadius = 0.02135;   // 高尔夫球半径，单位：米
const double ballArea = M_PI * ballRadius * ballRadius;
const double ballMass = 0.04593;     // 高尔夫球质量，单位：千克

//相机角度差
const double camera_angle_offset = 15;

// 结构体定义
struct BallisticParams{
    double distance;            // 目标距离，单位：米
    double altitude_angle;      // 目标高度角，单位：度 
    double velocity;            // 速度，单位：米/秒
};

double horizontal_distance; // 水平距离，单位：米
double vertical_distance;   // 垂直距离，单位：米

// 函数声明
double calculateTime(BallisticParams params, double angle);
double calculateDrag(double velocity);


int main() {
    BallisticParams params;
    double shoot_angle;     //射击角度
    double TOT_time;        //命中时间(time on target)
    
    
    // 用户输入参数
    // printf("请输入小车与目标的水平距离（米）: ");
    // scanf("%lf", &params.horizontal_distance);

    // printf("请输入目标比小车高的垂直距离（米）: ");
    // scanf("%lf", &params.vertical_distance);
    
    printf("Please enter the distance between the car and the target in meters:");
    scanf("%lf", &params.distance);
    
    printf("Please enter the height angle (in degrees) between the car and the target: ");
    scanf("%lf", &params.altitude_angle);

    printf("Please enter the launch speed of the projectile (meters/second): ");
    scanf("%lf", &params.velocity);

    //距离分解
    vertical_distance = params.distance * sin(params.altitude_angle / (180 / M_PI));
    horizontal_distance = params.distance * cos(params.altitude_angle / (180 / M_PI));

    // 使用二分法查找合适的发射角度
    double lower = 0, upper = M_PI / 2, mid;
    while (upper - lower > 0.0001) {
        mid = (lower + upper) / 2;
        vertical_distance = fabs(vertical_distance); // 取绝对值以防输入为负数
        if (calculateTime(params, mid) < 0) {
            lower = mid;
        } else {
            upper = mid;
        }
       
    }
    shoot_angle = mid;

    
    //结果正确性判断（45度）
    if(mid > (45 / (180 / M_PI))){
        printf("Unable to hit");
        return -1;
    }
    
    // 计算时间
    TOT_time = fabs(calculateTime(params, shoot_angle));

    // 输出结果
    printf("Shoot angle:%.2f°\n", shoot_angle * (180 / M_PI));
    printf("Time on target:%.2fs\n", TOT_time);

    return 0;
}

// 计算在给定参数下，命中目标所需的时间
double calculateTime(BallisticParams params, double angle) {
    double time = 0.0;
    double x = 0.0, y = 0.0;
    double vx = params.velocity * cos(angle);
    double vy = params.velocity * sin(angle);

    // 时间步长
    double dt = 0.0001;

    // 模拟弹道
    while (x < horizontal_distance && y >= 0) {
        // 更新位置
        x += vx * dt;
        y += vy * dt;

        // 更新速度
        double drag = calculateDrag(params.velocity);
        vx -= drag * vx / params.velocity * dt;
        vy -= (GRAVITY + drag * vy / params.velocity) * dt;

        // 更新总速度
        params.velocity = sqrt(vx * vx + vy * vy);

        // 更新时间
        time += dt;

        // 检查是否达到目标高度
        if ((vertical_distance - y) <= error_rangeY && (horizontal_distance - x) <= error_rangeX) {
            return time;
        }
    }
    // 如果解的精度不够高，返回最后收敛的负时间
    if (abs(time) <= 3){
        time = -time;
        return time;
    }  
    //无解返回0
    else 
        return 0;
}

// 计算空气阻力
double calculateDrag(double velocity) {
    return 0.5 * dragCoefficient * airDensity * ballArea * velocity * velocity / ballMass;
}