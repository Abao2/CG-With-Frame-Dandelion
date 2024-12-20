// Function to perform a single Runge-Kutta step
KineticState runge_kutta_step([[maybe_unused]] const KineticState& previous,
                              const KineticState& current)
{
    // 4阶龙格库塔
    Vector3f position = current.position;
    Vector3f velocity = current.velocity;
    Vector3f acceleration = current.acceleration;

    Vector3f k1 = time_step * velocity;
    Vector3f k2 = time_step * (velocity + time_step * acceleration / 2);
    Vector3f k3 = time_step * (velocity + time_step * acceleration / 2);
    Vector3f k4 = time_step * (velocity + time_step * acceleration);

    velocity += time_step * acceleration;   //加速度始终不变
    position += (k1 + 2*k2 + 2*k3 + k4) / 6.0f;

    KineticState next(position, velocity, acceleration);

    return next;
}

// Function to perform a single Backward Euler step
KineticState backward_euler_step([[maybe_unused]] const KineticState& previous,
                                 const KineticState& current)
{
    //后向欧拉
    Vector3f position = current.position;
    Vector3f velocity = current.velocity;
    Vector3f acceleration = current.acceleration;

    //均采用下一时刻的微分值进行更新
    velocity += time_step * acceleration;
    position += time_step * velocity;

    KineticState next(position, velocity, acceleration);

    return next;
}


// Function to perform a single Symplectic Euler step
KineticState symplectic_euler_step(const KineticState& previous, const KineticState& current)
{
    //偶对欧拉
    Vector3f position = current.position;
    Vector3f velocity = current.velocity;
    Vector3f acceleration = current.acceleration;

    //位置采用下一时刻的微分值进行更新

    //速度采用当前时刻的微分值进行更新
    velocity += time_step * acceleration;
    position += time_step * velocity;

    KineticState next(position, velocity, acceleration);

    previous;   //for compile
    return next;
}
