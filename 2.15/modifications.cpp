optional<Intersection> naive_intersect(const Ray& ray, const GL::Mesh& mesh, const Matrix4f model)
{
    Intersection result;
    result.t = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < mesh.faces.count(); ++i) {
        // Vertex a, b and c are assumed to be in counterclockwise order.
        Vector4f a = {mesh.vertex(mesh.face(i)[0]).x(), mesh.vertex(mesh.face(i)[0]).y(), mesh.vertex(mesh.face(i)[0]).z(), 1.0f};
        Vector4f b = {mesh.vertex(mesh.face(i)[1]).x(), mesh.vertex(mesh.face(i)[1]).y(), mesh.vertex(mesh.face(i)[1]).z(), 1.0f};
        Vector4f c = {mesh.vertex(mesh.face(i)[2]).x(), mesh.vertex(mesh.face(i)[2]).y(), mesh.vertex(mesh.face(i)[2]).z(), 1.0f};
    
        // Transform the vertices to world space.
        a = model * a;
        b = model * b;
        c = model * c;

        a /= a.w();
        b /= b.w();
        c /= c.w();
        //fetch 3D coordinates
        Vector3f v_a = {a.x(), a.y(), a.z()};
        Vector3f v_b  = {b.x(), b.y(), b.z()};
        Vector3f v_c  = {c.x(), c.y(), c.z()};

        // Construct matrix A = [d, a - b, a - c] and solve Ax = (a - origin)
        Vector3f edge_ba = v_a - v_b;
        Vector3f edge_ca = v_a - v_c;
        Eigen::Matrix3f A;
        //ray.direction = ray.direction.normalized();
        A.col(0) = ray.direction.normalized();
        A.col(1) = edge_ba;
        A.col(2) = edge_ca;
        // Matrix A is not invertible, indicating the ray is parallel with the triangle.
        float det = A.determinant();
        if (std::fabs(det) < eps) {
            continue;
        }
        Vector3f vec_b = v_a - ray.origin;
        Eigen::ColPivHouseholderQR<Eigen::Matrix3f> qr(A);
        Vector3f x = qr.solve(vec_b);
        float alpha = x.y();
        float beta  = x.z();
        float gamma = 1.0f - alpha - beta;
        //auto [alpha, beta, gamma] = [x.y(), x.z(), 1.0f - x.y() - x.z()];
        // Test if alpha, beta and gamma are all between 0 and 1.
        if (alpha >= 0 && beta >= 0 && gamma >= 0) {
            float t = x.x(); // intersection's t
            if(t < eps) continue;        //t < 0
            if(t < result.t) {
                result.t = t;
                std::cout << "t = " << t << std::endl;
                result.normal = ((edge_ba).cross(edge_ca)).normalized();
                Vector3f bary_coord = {alpha, beta, gamma};
                result.barycentric_coord = bary_coord;
                result.face_index = i;
            }
        }
    }
    // Ensure result.t is strictly less than the constant `infinity`.
    if (result.t < std::numeric_limits<float>::infinity()) {
        #ifdef DEBUG
            std::cout << "Crashed!\n" << std::endl;
            std::cout << "Result.t = " << result.t << std::endl;
            std::cout << "result.t - infinity" << result.t - infinity << std::endl;
        #endif
        return result;
    }else{
        //std::cout << "Result.t = " << result.t << std::endl;
        //std::cout << "result.t - infinity" << result.t - infinity << std::endl;
    }
    // nan intersection
    return std::nullopt;
}
void Object::update(vector<Object*>& all_objects)
{
    // 首先调用 step 函数计下一步该物体的运动学状态。
    KineticState current_state{center, velocity, force / mass};
    KineticState next_state = step(prev_state, current_state);
    // 将物体的位置移动到下一步状态处，但暂时不要修改物体的速度。
    center = next_state.position;
    // 遍历 all_objects，检查该物体在下一步状态的位置处是否会与其他物体发生碰撞。
    for (auto object : all_objects) {
        // 检测该物体与另一物体是否碰撞的方法是：
        // 遍历该物体的每一条边，构造与边重合的射线去和另一物体求交，如果求交结果非空、
        // 相交处也在这条边的两个端点之间，那么该物体与另一物体发生碰撞。
        // 请时刻注意：物体 mesh 顶点的坐标都在模型坐标系下，你需要先将其变换到世界坐标系。
        if( object->id == this->id )    continue;
        for (size_t i = 0; i < mesh.edges.count(); ++i) {
            array<size_t, 2> v_indices = mesh.edge(i);
            // v_indices 中是这条边两个端点的索引，以这两个索引为参数调用 GL::Mesh::vertex
            // 方法可以获得它们的坐标，进而用于构造射线。

            Vector3f v1 = mesh.vertex(v_indices[0]);
            Vector3f v2 = mesh.vertex(v_indices[1]);

            Eigen::Vector4f hv1 = {v1.x(), v1.y(), v1.z(), 1.0f};
            Eigen::Vector4f hv2 = {v2.x(), v2.y(), v2.z(), 1.0f};

            //get world coordinate
            //model()是this object的模型矩阵
            hv1 = model() * hv1;
            hv2 = model() * hv2;
            hv1 /= hv1.w();
            hv2 /= hv2.w();
            v1 = {hv1.x(), hv1.y(), hv1.z()};
            v2 = {hv2.x(), hv2.y(), hv2.z()};
            //generate ray
            Ray ray(v1, (v2-v1).normalized());
            //object->model() correspond to object inside the loop' model matrix
            //i.e. object to be detected
            std::optional<Intersection> intersection_opt = naive_intersect(ray, object->mesh, object->model());
            // 根据求交结果，判断该物体与另一物体是否发生了碰撞。
            if(intersection_opt.has_value()) {
                //have intersection
                Intersection intersection = intersection_opt.value();
                float t = intersection.t;
                float distance = (v2-v1).norm();
                if(t >= 0 && t <= distance) {
                    // 如果发生碰撞，按动量定理计算两个物体碰撞后的速度，并将下一步状态的位置设为
                    // current_state.position ，以避免重复碰撞。
                    // 完全弹性碰撞
                    //std::cout << "Collision detected!" << std::endl;
                    next_state.position = current_state.position;
                    if((next_state.velocity - object->velocity).dot(intersection.normal) == 0)    continue;
                    float momentum = (next_state.velocity - object->velocity).dot(intersection.normal);
                    momentum = momentum * (-2.0f) / (1.0f / mass + 1.0f / object->mass);
                    next_state.velocity = next_state.velocity + momentum / mass * intersection.normal;
                    object->velocity = object->velocity - momentum / object->mass * intersection.normal;
                    //更新状态
                    //center = current_state.position;

                    #ifdef DEBUG
                        std::cout << "Momentum: " << momentum << std::endl;
                        std::cout << "Object velocity: " << object->velocity << std::endl;
                        std::cout << "t: " << t << "   distance: " << distance << std::endl;
                        std::cout << "Next Velocity: " << std::endl << next_state.velocity << std::endl << " Normal: " << std::endl << intersection.normal << std::endl;
                    #endif

                    Vector3f temp = next_state.velocity;
                    next_state = step(current_state, next_state);
                    next_state.velocity = temp;
                    break;
                }
            }
        }
    }
    // 将上一步状态赋值为当前状态，并将物体更新到下一步状态。
    prev_state = current_state;
    center = next_state.position;
    velocity = next_state.velocity;
}