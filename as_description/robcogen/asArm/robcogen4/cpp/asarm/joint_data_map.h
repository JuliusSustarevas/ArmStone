#ifndef IIT_ASARM_JOINT_DATA_MAP_H_
#define IIT_ASARM_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace asArm {

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
    data[XARMJOINT1] = rhs[XARMJOINT1];
    data[XARMJOINT2] = rhs[XARMJOINT2];
    data[XARMJOINT3] = rhs[XARMJOINT3];
    data[XARMJOINT4] = rhs[XARMJOINT4];
    data[XARMJOINT5] = rhs[XARMJOINT5];
    data[XARMJOINT6] = rhs[XARMJOINT6];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
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
