#ifndef UTILS_HPP_
#include "geometry_msgs/msg/vector3.hpp"
inline auto operator+(const geometry_msgs::msg::Vector3& a,const geometry_msgs::msg::Vector3& b)
{
    geometry_msgs::msg::Vector3 result;
    result.set__x( a.x + b.x);
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

inline std::ostream& operator << ( std::ostream& os, const geometry_msgs::msg::Vector3& v)
{
    os << "Vec["
        << " x[" << v.x
        << "] y[" << v.y
        << "] z[" << v.z
        << "] ]";
    return os;
}

inline auto create_vector3(const double& x,const double& y, const double& z)
{
    geometry_msgs::msg::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}
#endif /* UTILS_HPP_ */
