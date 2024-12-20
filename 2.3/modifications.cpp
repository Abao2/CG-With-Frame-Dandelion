void RasterizerRenderer::render_mt(const Scene& scene){
    // 多线程实现
    // 注意：需要保证所有线程都执行到同一时刻，并保证线程安全
    // 注意：需要保证正确的交换顺序，保证正确的深度测试
    // 注意：需要保证正确的片元着色器的执行
    // 注意：需要保证正确的片元着色器的输出
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
    
    this->logger->info("rendering ({} threads) takes {:.6f} seconds",
                       n_vertex_threads + n_rasterizer_threads + n_fragment_threads,
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
void VertexProcessor::worker_thread()
{
    while (true) {
        if(Context::vertex_finish)  return;
        int num = 0;
        /* 一次性对3个顶点进行操作*/
        VertexShaderPayload payload[3];
        {
            if (vertex_queue.empty()) {
                continue;
            }
            //获取顶点队列的锁
            std::unique_lock<std::mutex> lock(queue_mutex);
            if (vertex_queue.empty()) {
                continue;
            }
            else if (vertex_queue.size() == 1){
                payload[0] = vertex_queue.front();
                vertex_queue.pop();
                num = 1;
            }
            else if(vertex_queue.size() == 2){
                payload[0] = vertex_queue.front();
                vertex_queue.pop();
                payload[1] = vertex_queue.front();
                vertex_queue.pop();
                num = 2;
            }
            else if(vertex_queue.size() >= 3){
                payload[0] = vertex_queue.front();
                vertex_queue.pop();
                payload[1] = vertex_queue.front();
                vertex_queue.pop();
                payload[2] = vertex_queue.front();
                vertex_queue.pop();
                num = 3;
            }
        }
        VertexShaderPayload output_payload[3];
        for(int i = 0; i < num; i++){
            if(payload[i].world_position.w() == -1.0f){
                Context::vertex_finish = true;
                num = i;
                //return;
                break;
            }
            output_payload[i] = vertex_shader_ptr(payload[i]);
        }
        {
            std::unique_lock<std::mutex> lock(Context::vertex_queue_mutex);
            for(int i = 0; i < num; i++){
                Context::vertex_shader_output_queue.push(output_payload[i]);
            }
        }
    }
}
class RenderEngine
{
public:
    RenderEngine();

    /*! \~chinese 渲染的结果（以无符号字符型变量进行存储）*/
    std::vector<unsigned char> rendering_res;
    /*! \~chinese 根据aspect_ratio对渲染出图片的长和宽进行设置 */
    float width, height;
    /*! \~chinese whitted renderer使用多线程时的线程数设置 */
    int n_threads;
    int n_vertex_threads;
    int n_rasterizer_threads;
    int n_fragment_threads;
    /*! \~chinese 动态更新线程个数 */
    void update_threads();
    /*!
     * \~chinese
     * \brief 离线渲染入口，负责调用渲染器的渲染函数
     *
     * 可以选择不同的渲染方式，在设置好场景之后，执行该函数能够得到
     * 使用选定方式对当前场景的渲染结果
     *
     * \param scene 场景
     * \param type 渲染器类型
     */
    void render(Scene& scene, RendererType type);
    /*! \~chinese 渲染结果预览的背景颜色 */
    static Eigen::Vector3f background_color;

    /*! \~chinese 光栅化渲染器 */
    std::unique_ptr<RasterizerRenderer> rasterizer_render;
    /*! \~chinese whitted style渲染器 */
    std::unique_ptr<WhittedRenderer> whitted_render;
};
enum class RendererType
{
    RASTERIZER,
    RASTERIZER_MT,
    // RASTERIZER_MT,
    WHITTED_STYLE
};
class RasterizerRenderer
{
public:
    RasterizerRenderer(RenderEngine& engine, int num_vertex_threads = 1, int num_rasterizer_threads = 1,
                       int num_fragment_threads = 1);

    /*! \~chinese 光栅化渲染器的渲染调用接口*/
    void render(const Scene& scene);

    /*! \~chinese 多线程光栅化渲染器的渲染调用接口*/
     void render_mt(const Scene& scene);
    float& width;
    float& height;
    int n_vertex_threads;
    int n_rasterizer_threads;
    int n_fragment_threads;

    // initialize vertex processor
    VertexProcessor vertex_processor;

    // initialize rasterizer
    Rasterizer rasterizer;

    // initialize fragment processor
    FragmentProcessor fragment_processor;

    std::vector<unsigned char>& rendering_res;

private:
    std::shared_ptr<spdlog::logger> logger;
};
void RenderEngine::update_threads() {
    rasterizer_render -> n_vertex_threads = n_vertex_threads;
    rasterizer_render -> n_rasterizer_threads = n_rasterizer_threads;
    rasterizer_render -> n_fragment_threads = n_fragment_threads;
    printf("Updated threads: %d, %d, %d\n", n_vertex_threads, n_rasterizer_threads, n_fragment_threads);
}
tuple<float, float, float> Rasterizer::compute_barycentric_2d(float x, float y, const Vector4f* v)
{

    // 构造矩阵：ABC是三角形的顶点坐标矩阵，ABP、PBC、APC是各自的替代矩阵
    Eigen::Matrix3f matABC, matABP, matPBC, matAPC;
    matABC << 1, v[0].x(), v[0].y(),
              1, v[1].x(), v[1].y(),
              1, v[2].x(), v[2].y();

    matABP << 1, v[0].x(), v[0].y(),
              1, v[1].x(), v[1].y(),
              1, x, y;

    matPBC << 1, x, y,
              1, v[1].x(), v[1].y(),
              1, v[2].x(), v[2].y();

    matAPC << 1, v[0].x(), v[0].y(),
              1, x, y,
              1, v[2].x(), v[2].y();

    float alpha, beta, gamma;
    float detABC = matABC.determinant();
    alpha = matPBC.determinant() / detABC;
    beta = matAPC.determinant() / detABC;
    gamma = matABP.determinant() / detABC;

    return {alpha, beta, gamma};
}
void Rasterizer::worker_thread()        //光栅化线程：vertex(output) -> fragment
{
    while (true) {
        VertexShaderPayload payload[3];
        //VertexShaderPayload payload;
        Triangle triangle;
        {
            // printf("vertex_finish = %d\n vertex_shader_output_queue.size = %ld\n",
            // Context::vertex_finish, Context::vertex_shader_output_queue.size());

            //先判断是否完成
            if (Context::vertex_finish && Context::vertex_shader_output_queue.empty()) {
                Context::rasterizer_finish = true;
                return;
            }
            if (Context::vertex_shader_output_queue.size() < 3) {
                continue;
            }
            std::unique_lock<std::mutex> lock(Context::vertex_queue_mutex);
            if (Context::vertex_shader_output_queue.size() < 3) {
                continue;
            }
            for (int i = 0; i < 3; i++) {
                payload[i] = Context::vertex_shader_output_queue.front();
                Context::vertex_shader_output_queue.pop();
            }
        }//unlock
            //把三个顶点处理成三角形fragment
            for (size_t vertex_count = 0; vertex_count < 3; vertex_count++) {
                if (vertex_count == 0) {
                    triangle.world_pos[0]    = payload[vertex_count].world_position;
                    triangle.viewport_pos[0] = payload[vertex_count].viewport_position;
                    triangle.normal[0]       = payload[vertex_count].normal;
                } else if (vertex_count == 1) {
                    triangle.world_pos[1]    = payload[vertex_count].world_position;
                    triangle.viewport_pos[1] = payload[vertex_count].viewport_position;
                    triangle.normal[1]       = payload[vertex_count].normal;
                } else {
                    triangle.world_pos[2]    = payload[vertex_count].world_position;
                    triangle.viewport_pos[2] = payload[vertex_count].viewport_position;
                    triangle.normal[2]       = payload[vertex_count].normal;
                }
            }
        rasterize_triangle(triangle);
    }
}
RenderEngine::RenderEngine()
{
    // default setting of number of threads(if use multi-threads edition)
    n_threads = 4;
    n_vertex_threads = 1;
    n_rasterizer_threads = 1;
    n_fragment_threads = 1;
    // unique pointer to Rasterizer Renderer
    printf("engine's number: %d, %d, %d \n", n_vertex_threads, n_rasterizer_threads, n_rasterizer_threads);
    rasterizer_render = std::make_unique<RasterizerRenderer>(*this, n_vertex_threads, n_rasterizer_threads, n_fragment_threads);
    printf("num of threads: %d, %d, %d\n", rasterizer_render->n_vertex_threads, rasterizer_render->n_rasterizer_threads, rasterizer_render->n_fragment_threads);
    // unique pointer to Whitted Style Renderer
    whitted_render = std::make_unique<WhittedRenderer>(*this);    
}
void RenderEngine::render(Scene& scene, RendererType type)
{
    switch (type) {
    case RendererType::RASTERIZER: rasterizer_render->render(scene); break;
    // case RendererType::RASTERIZER_MT: rasterizer_render->render_mt(scene); break;
    case RendererType::WHITTED_STYLE: whitted_render->render(scene); break;
    case RendererType::RASTERIZER_MT: rasterizer_render->render_mt(scene); break;
    default: break;
    }
}
