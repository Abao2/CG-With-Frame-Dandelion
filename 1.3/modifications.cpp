Matrix4f Camera::projection()
{
    const float fov_y = radians(fov_y_degrees);     //y轴视角
    const float top   = near * std::tan(fov_y / 2.0f);
                                                    //近平面高度
    const float right = top * aspect_ratio;         //近平面宽度

    Matrix4f projection = Matrix4f::Zero();

    projection(0, 0) = near / right;
    projection(1, 1) = near / top;
    projection(2, 2) = -(far + near) / (far - near);
    projection(2, 3) = -2.0f * far * near / (far - near);
    projection(3, 2) = -1.0f;
    projection(3, 3) = 0.0f;

    return projection;
}
