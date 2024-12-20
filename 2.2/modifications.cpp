// 对当前三角形进行光栅化
void Rasterizer::rasterize_triangle(Triangle& t)
{

    // 存储w坐标
    Vector3f weight = {t.viewport_pos[0].w(), t.viewport_pos[1].w(), t.viewport_pos[2].w()};

    float x_min = t.viewport_pos[0].x();
    float x_max = t.viewport_pos[0].x();
    float y_min = t.viewport_pos[0].y();
    float y_max = t.viewport_pos[0].y();

    for(int i = 0; i < 3; i++) {
        x_min = std::max(0.0f, std::min(x_min, t.viewport_pos[i].x()));
        x_max = std::min(Context::frame_buffer.width-1.0f, std::max(x_max, t.viewport_pos[i].x()));
        y_min = std::max(0.0f, std::min(y_min, t.viewport_pos[i].y()));
        y_max = std::min(Context::frame_buffer.height-1.0f, std::max(y_max, t.viewport_pos[i].y()));
    }
    int int_x_min = static_cast<int>(x_min);
    int int_x_max = static_cast<int>(x_max);
    int int_y_min = static_cast<int>(y_min);
    int int_y_max = static_cast<int>(y_max);

    //遍历屏幕所有像素
    for(int i = int_x_min; i <= int_x_max; i++){
        for(int j = int_y_min; j <= int_y_max; j++){
                // if current pixel is in current triange:
            if(inside_triangle(i, j, t.viewport_pos)){

                // get barycentric coordinates
                float fi = static_cast<float>(i);
                float fj = static_cast<float>(j);
                auto [alpha, beta, gamma] = compute_barycentric_2d(fi, fj, t.viewport_pos);

                // compute Z_t
                float Z = 1 / (alpha / weight[0] + beta / weight[1] + gamma / weight[2]);

                //给片元各属性进行插值
                FragmentShaderPayload payload;
                payload.x = i;  payload.y = j;  //payload.color = NULL;

                // 1. interpolate depth(use projection correction algorithm)
                payload.depth = alpha * t.viewport_pos[0].z() / weight[0] + beta * t.viewport_pos[1].z() / weight[1] + gamma * t.viewport_pos[2].z() / weight[2];
                payload.depth *= Z;

                // 2. interpolate vertex positon & normal(use function:interpolate())
                payload.world_normal = interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], weight, Z);
                payload.world_normal.normalized();
                
                payload.world_pos = interpolate(alpha, beta, gamma, t.world_pos[0].head(3), t.world_pos[1].head(3), t.world_pos[2].head(3), weight, Z);
                
                // 3. push primitive into fragment queue
                std::unique_lock<std::mutex> lock(Context::rasterizer_queue_mutex);
                Context::rasterizer_output_queue.push(payload);
            }
        }
    }
}

// 给定坐标(x,y)以及三角形的三个顶点坐标，计算(x,y)对应的重心坐标[alpha, beta, gamma]
tuple<float, float, float> Rasterizer::compute_barycentric_2d(float x, float y, const Vector4f* v)
{
    float c1 = 0.f, c2 = 0.f, c3 = 0.f;
    //存储三角形顶点坐标
    Vector3f tri[3];
    for (int i = 0; i < 3; i++) tri[i] = {v[i].x(), v[i].y(), 1.0};

    //计算坐标
    // 计算 c3 (gamma)
    c3 = ((tri[0].y() - tri[1].y()) * x + (tri[1].x() - tri[0].x()) * y + tri[0].x() * tri[1].y() - tri[1].x() * tri[0].y()) /
         ((tri[0].y() - tri[1].y()) * tri[2].x() + (tri[1].x() - tri[0].x()) * tri[2].y() + tri[0].x() * tri[1].y() - tri[1].x() * tri[0].y());

    // 计算 c2 (beta)
    c2 = ((tri[0].y() - tri[2].y()) * x + (tri[2].x() - tri[0].x()) * y + tri[0].x() * tri[2].y() - tri[2].x() * tri[0].y()) /
         ((tri[0].y() - tri[2].y()) * tri[1].x() + (tri[2].x() - tri[0].x()) * tri[1].y() + tri[0].x() * tri[2].y() - tri[2].x() * tri[0].y());

    // 计算 c1 (alpha)
    c1 = 1 - c2 - c3;

    return {c1, c2, c3};
}
// 给定坐标(x,y)以及三角形的三个顶点坐标，判断(x,y)是否在三角形的内部
bool Rasterizer::inside_triangle(int x, int y, const Vector4f* vertices)
{
    Vector3f v[3];
    for (int i = 0; i < 3; i++) v[i] = {vertices[i].x(), vertices[i].y(), 1.0};

    float fx = static_cast<float>(x);
    float fy = static_cast<float>(y);

    //重心法求解
    auto [alpha, beta, gamma] = compute_barycentric_2d(fx, fy, vertices);
    return (alpha >= 0) && (beta >= 0) && (gamma >= 0);
}
Vector3f phong_fragment_shader(const FragmentShaderPayload& payload, const GL::Material& material,
                               const std::list<Light>& lights, const Camera& camera)
{
    Vector3f result = {0, 0, 0};

    // ka,kd,ks can be got from material.ambient,material.diffuse,material.specular
    Vector3f ka = material.ambient;
    Vector3f kd = material.diffuse;
    Vector3f ks = material.specular;
    // set ambient light intensity
    float shininess = material.shininess;
    // Ambient
    float ambient_intensity = 0.4f;
    Vector3f ambient = ka * ambient_intensity;
    result += ambient;
    for(const Light& light : lights){
        // normal vector
        Vector3f normal = payload.world_normal.head<3>();
        // Light Direction
        Vector3f light_direction = (light.position - payload.world_pos);
        float distance = light_direction.norm();
        light_direction.normalize();
        // View Direction
        Vector3f view_direction = (camera.position - payload.world_pos).normalized();
        // Half Vector
        Vector3f half_vec = (light_direction + view_direction).normalized();
        // Light Attenuation
        float attenuated_light = light.intensity / (distance * distance);
        // Diffuse
        Vector3f diffuse = kd * attenuated_light * std::max(0.0f, payload.world_normal.dot(light_direction));
        // Specular
        Vector3f specular = ks * attenuated_light *
                std::pow(std::max(0.0f, normal.dot(half_vec)),
                shininess);
        result +=  diffuse + specular;
    }
    // set rendering result max threshold to 255
    if(result.x() > 1.0f)  result.x() = 1.0f; 
    if(result.y() > 1.0f)  result.y() = 1.0f; 
    if(result.z() > 1.0f)  result.z() = 1.0f; 
    return result * 255.f;
}
// vertex shader    1、将顶点坐标变换到投影平面     2、再进行视口变换将法线向量变换到相机坐标系用于后续插值。
VertexShaderPayload vertex_shader(const VertexShaderPayload& payload)
{
    VertexShaderPayload output_payload = payload;

    // Vertex position transformation
    //模型坐标系 -> NDC 
    output_payload.viewport_position = Uniforms::MVP * output_payload.world_position;
    //透视除法
    output_payload.viewport_position /= output_payload.viewport_position.w();

    // Viewport transformation

    // NDC -> 屏幕坐标系
    output_payload.viewport_position.x() = output_payload.viewport_position.x() * Uniforms::width * 0.5f + Uniforms::width * 0.5f;
    output_payload.viewport_position.y() = output_payload.viewport_position.y() * Uniforms::height * 0.5f + Uniforms::height * 0.5f;

    // Vertex normal transformation
    output_payload.normal = (Uniforms::inv_trans_M * Vector4f(output_payload.normal.x(), output_payload.normal.y(), output_payload.normal.z(), 0.f)).head<3>();
    output_payload.normal.normalized();
    return output_payload;
}
// 光栅化渲染器的渲染调用接口
void RasterizerRenderer::render(const Scene& scene)
{
    Uniforms::width       = static_cast<int>(width);
    Uniforms::height      = static_cast<int>(height);
    Context::frame_buffer = FrameBuffer(Uniforms::width, Uniforms::height);
    // clear Color Buffer & Depth Buffer & rendering_res
    Context::frame_buffer.clear(BufferType::Color | BufferType::Depth);
    this->rendering_res.clear();
    // run time statistics
    time_point begin_time                  = steady_clock::now();
    Camera cam                             = scene.camera;
    //初始化函数指针
    vertex_processor.vertex_shader_ptr     = vertex_shader;
    fragment_processor.fragment_shader_ptr = phong_fragment_shader;
    //遍历所有物体
    for (const auto& group : scene.groups) {
        for (const auto& object : group->objects) {
            //初始化三个线程的完成标志
            Context::vertex_finish     = false;
            Context::rasterizer_finish = false;
            Context::fragment_finish   = false;



            // set Uniforms for vertex shader
            Uniforms::MVP         = cam.projection() * cam.view() * object->model();
            Uniforms::inv_trans_M = object->model().inverse().transpose();
            Uniforms::width       = static_cast<int>(this->width);
            Uniforms::height      = static_cast<int>(this->height);

            // To do: 同步
            Uniforms::material = object->mesh.material;
            Uniforms::lights   = scene.lights;
            Uniforms::camera   = scene.camera;

            // input object->mesh's vertices & faces & normals data
            const std::vector<float>& vertices     = object->mesh.vertices.data;
            const std::vector<unsigned int>& faces = object->mesh.faces.data;
            const std::vector<float>& normals      = object->mesh.normals.data;
            size_t num_faces                       = faces.size();

            // process vertices
            for (size_t i = 0; i < num_faces; i += 3) {
                for (size_t j = 0; j < 3; j++) {
                    size_t idx = faces[i + j];
                    vertex_processor.input_vertices(
                        Vector4f(vertices[3 * idx], vertices[3 * idx + 1], vertices[3 * idx + 2],
                                 1.0f),
                        Vector3f(normals[3 * idx], normals[3 * idx + 1], normals[3 * idx + 2]));
                }
            }
            //顶点输入完毕
            vertex_processor.input_vertices(Eigen::Vector4f(0, 0, 0, -1.0f),
                                            Eigen::Vector3f::Zero());
                        std::vector<std::thread> workers;
            for (int i = 0; i < n_vertex_threads; ++i) {
                workers.emplace_back(&VertexProcessor::worker_thread, &vertex_processor);
            }
            for (int i = 0; i < n_rasterizer_threads; ++i) {
                workers.emplace_back(&Rasterizer::worker_thread, &rasterizer);
            }
            for (int i = 0; i < n_fragment_threads; ++i) {
                workers.emplace_back(&FragmentProcessor::worker_thread, &fragment_processor);
            }
            for (auto& worker : workers) {
                if (worker.joinable()) {
                    worker.join();
                }
            }
        }
    }

    //日志信息
    time_point end_time         = steady_clock::now();
    duration rendering_duration = end_time - begin_time;

    /*
    if( n_vertex_threads == n_fragment_threads == n_rasterizer_threads == 1) {
        this->logger->info("rendering (single thread) takes {:.6f} seconds",
                       rendering_duration.count());
    }else{
        this->logger->info("rendering ({} threads) takes {:.6f} seconds",
                       n_vertex_threads + n_rasterizer_threads + n_fragment_threads,
                       rendering_duration.count());
    }   */
    this->logger->info("rendering (single thread) takes {:.6f} seconds",
                       rendering_duration.count());

    for (long unsigned int i = 0; i < Context::frame_buffer.depth_buffer.size(); i++) {
        rendering_res.push_back(
            static_cast<unsigned char>(Context::frame_buffer.color_buffer[i].x()));
        rendering_res.push_back(
            static_cast<unsigned char>(Context::frame_buffer.color_buffer[i].y()));
        rendering_res.push_back(
            static_cast<unsigned char>(Context::frame_buffer.color_buffer[i].z()));
    }
}