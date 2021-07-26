#ifndef RCG_ARMSTONE_JOINT_DATA_MAP_H_
#define RCG_ARMSTONE_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace armstone {
namespace rcg {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[MOTOR_WHEEL_JOINT_FL] = rhs[MOTOR_WHEEL_JOINT_FL];
    data[MOTOR_WHEEL_JOINT_FR] = rhs[MOTOR_WHEEL_JOINT_FR];
    data[MOTOR_WHEEL_JOINT_BL] = rhs[MOTOR_WHEEL_JOINT_BL];
    data[MOTOR_WHEEL_JOINT_BR] = rhs[MOTOR_WHEEL_JOINT_BR];
    data[XARMJOINT1] = rhs[XARMJOINT1];
    data[XARMJOINT2] = rhs[XARMJOINT2];
    data[XARMJOINT3] = rhs[XARMJOINT3];
    data[XARMJOINT4] = rhs[XARMJOINT4];
    data[XARMJOINT5] = rhs[XARMJOINT5];
    data[XARMJOINT6] = rhs[XARMJOINT6];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[MOTOR_WHEEL_JOINT_FL] = value;
    data[MOTOR_WHEEL_JOINT_FR] = value;
    data[MOTOR_WHEEL_JOINT_BL] = value;
    data[MOTOR_WHEEL_JOINT_BR] = value;
    data[XARMJOINT1] = value;
    data[XARMJOINT2] = value;
    data[XARMJOINT3] = value;
    data[XARMJOINT4] = value;
    data[XARMJOINT5] = value;
    data[XARMJOINT6] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   motor_wheel_joint_fl = "
    << map[MOTOR_WHEEL_JOINT_FL]
    << "   motor_wheel_joint_fr = "
    << map[MOTOR_WHEEL_JOINT_FR]
    << "   motor_wheel_joint_bl = "
    << map[MOTOR_WHEEL_JOINT_BL]
    << "   motor_wheel_joint_br = "
    << map[MOTOR_WHEEL_JOINT_BR]
    << "   xarmjoint1 = "
    << map[XARMJOINT1]
    << "   xarmjoint2 = "
    << map[XARMJOINT2]
    << "   xarmjoint3 = "
    << map[XARMJOINT3]
    << "   xarmjoint4 = "
    << map[XARMJOINT4]
    << "   xarmjoint5 = "
    << map[XARMJOINT5]
    << "   xarmjoint6 = "
    << map[XARMJOINT6]
    ;
    return out;
}

}
}
#endif
