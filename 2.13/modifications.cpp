void Scene::simulation_update()
{
    // 这次模拟的总时长不是上一帧的时长，而是上一帧时长与之前帧剩余时长的总和，
    // 即上次调用 simulation_update 到现在过了多久。

    // 以固定的时间步长 (time_step) 循环模拟物体运动，每模拟一步，模拟总时长就减去一个
    // time_step ，当总时长不够一个 time_step 时停止模拟。

    // 根据刚才模拟时间步的数量，更新最后一次调用 simulation_update 的时间 (last_update)。
    for(const auto& group : groups) {
        for(const auto& object : group->objects) {
            //initiate
            object; //for compile
        }
    }
    // 计算剩余时间
    duration frame_duration = std::chrono::steady_clock::now() - last_update;
    duration remain_duration = frame_duration;
    float remain_time = remain_duration.count();

    while (remain_time > time_step) {
        for(const auto& group : groups) {
            for(const auto& object : group->objects) {
                object->update(all_objects);
            }
        }

        remain_time -= time_step;  // 减去时间步长
    }

    float consumed_time_within_frame = frame_duration.count() - remain_time;
    duration consumed_duration(consumed_time_within_frame);
    last_update = last_update + std::chrono::duration_cast<std::chrono::steady_clock::duration>(consumed_duration);
}

void Object::update(vector<Object*>& all_objects)
{
    // 首先调用 step 函数计下一步该物体的运动学状态。
    KineticState current_state{center, velocity, force / mass};
    KineticState next_state = step(prev_state, current_state);
    (void)next_state;
    // 将物体的位置移动到下一步状态处，但暂时不要修改物体的速度。
    center = next_state.position;
    // 遍历 all_objects，检查该物体在下一步状态的位置处是否会与其他物体发生碰撞。
    for (auto object : all_objects) {
        (void)object;

        // 检测该物体与另一物体是否碰撞的方法是：
        // 遍历该物体的每一条边，构造与边重合的射线去和另一物体求交，如果求交结果非空、
        // 相交处也在这条边的两个端点之间，那么该物体与另一物体发生碰撞。
        // 请时刻注意：物体 mesh 顶点的坐标都在模型坐标系下，你需要先将其变换到世界坐标系。
        for (size_t i = 0; i < mesh.edges.count(); ++i) {
            array<size_t, 2> v_indices = mesh.edge(i);
            (void)v_indices;
            // v_indices 中是这条边两个端点的索引，以这两个索引为参数调用 GL::Mesh::vertex
            // 方法可以获得它们的坐标，进而用于构造射线。
            if (BVH_for_collision) {
            } else {
            }
            // 根据求交结果，判断该物体与另一物体是否发生了碰撞。
            // 如果发生碰撞，按动量定理计算两个物体碰撞后的速度，并将下一步状态的位置设为
            // current_state.position ，以避免重复碰撞。
        }
    }
    // 将上一步状态赋值为当前状态，并将物体更新到下一步状态。
    prev_state = current_state;
    velocity = next_state.velocity;
}
// Function to perform a single Forward Euler step
KineticState forward_euler_step([[maybe_unused]] const KineticState& previous,
                                const KineticState& current)
{
    Vector3f position = current.position;
    Vector3f velocity = current.velocity;
    Vector3f acceleration = current.acceleration;

    position += time_step * velocity;
    velocity += time_step * acceleration;

    KineticState next(position, velocity, acceleration);

    return next;
}