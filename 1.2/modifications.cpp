Matrix4f Object::model()
{
    //Scaling Matrix
    Matrix4f Scaling = Matrix4f::Identity();
    Scaling(0,0) = scaling.x();
    Scaling(1,1) = scaling.y();
    Scaling(2,2) = scaling.z();

    //Rotation Matrix
    const Quaternionf& r = rotation;
    auto [x_angle, y_angle, z_angle] = quaternion_to_ZYX_euler(r.w(), r.x(),r.y(), r.z());
// Then construct the rotation matrix with euler angles.

    float theta_x = radians(x_angle);
    float theta_y = radians(y_angle);
    float theta_z = radians(z_angle);

    //Rz
    Matrix4f Rz;
    Rz << std::cos(theta_z), -std::sin(theta_z), 0, 0,
          std::sin(theta_z),  std::cos(theta_z), 0, 0,
          0,                 0,                 1, 0,
          0,                 0,                 0, 1;

    //Ry
    Matrix4f Ry;
    Ry << std::cos(theta_y), 0, std::sin(theta_y), 0,
          0,                 1, 0,                 0,
         -std::sin(theta_y), 0, std::cos(theta_y), 0,
          0,                 0, 0,                 1;

    //Rx
    Matrix4f Rx;
    Rx << 1, 0,                 0,                0,
          0, std::cos(theta_x), -std::sin(theta_x), 0,
          0, std::sin(theta_x),  std::cos(theta_x), 0,
          0, 0,                 0,                1;

    Matrix4f Rotation = Rx * Ry * Rz;

    //Center Matrix
    Matrix4f Center = Matrix4f::Identity();
    Center(0,3) = center.x();
    Center(1,3) = center.y();
    Center(2,3) = center.z();

    //Transformation Matrix
    Matrix4f Trans = Center * Rotation * Scaling;


    return Trans;
}